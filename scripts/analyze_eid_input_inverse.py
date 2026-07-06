#!/usr/bin/env python3
"""Compute and replay the local EID input inverse for configured joints."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml

from report_paths import analysis_report_dir, repo_relpath


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "h1_real_p4_ku_u3_hip_knee_eid.yaml"


@dataclass(frozen=True)
class JointInverse:
    config_path: Path
    joint_id: int
    joint_name: str
    dt: float
    Jeff: float
    g_q: float
    g_dq: float
    pinv_q: float
    pinv_dq: float
    weighted_pinv_q: float
    weighted_pinv_dq: float
    w_q: float
    w_dq: float
    ko_q: float
    ko_dq: float
    ku_q: float
    ku_dq: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Compute g=[Ts^2/J, Ts/J]^T and g+ for EID controller configs. "
            "Optionally replay logs to compare fixed Ku eta_u with g+ Ko(x-x_hat)."
        )
    )
    parser.add_argument(
        "--config",
        action="append",
        type=Path,
        default=[],
        help="Controller YAML config. May be repeated. Defaults to the P4 hip-knee EID config.",
    )
    parser.add_argument(
        "--log",
        action="append",
        type=Path,
        default=[],
        help="Optional EID log CSV to replay. May be repeated.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=ROOT / "analysis_artifacts" / "eid_input_inverse",
        help="Directory for CSV artifacts.",
    )
    parser.add_argument(
        "--write-detail",
        action="store_true",
        help="Write per-sample replay details in addition to summary CSVs.",
    )
    parser.add_argument(
        "--high-freq-cutoff-hz",
        type=float,
        default=10.0,
        help="Cutoff used for high-frequency power ratio in replay summaries.",
    )
    return parser.parse_args()


def resolve_path(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def read_yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def yaml_joint(joints: dict, joint_id: int) -> dict | None:
    return joints.get(joint_id, joints.get(str(joint_id)))


def merged_controller_params(cfg: dict, joint: dict) -> dict:
    params = dict(cfg.get("controller", {}).get("defaults", {}))
    for key, value in joint.items():
        if key not in {"name", "enabled", "plant"}:
            params[key] = value
    return params


def active_joint_ids(cfg: dict) -> list[int]:
    joints = cfg.get("controller", {}).get("joints", {})
    ids = []
    for key, joint in joints.items():
        try:
            joint_id = int(key)
        except (TypeError, ValueError):
            continue
        if bool(joint.get("enabled", True)):
            ids.append(joint_id)
    return sorted(ids)


def finite_rms(values: Iterable[float]) -> float:
    x = np.asarray(list(values), dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def finite_peak(values: Iterable[float]) -> float:
    x = np.asarray(list(values), dtype=float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return float("nan")
    return float(np.max(np.abs(x)))


def correlation(a: np.ndarray, b: np.ndarray) -> float:
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    mask = np.isfinite(aa) & np.isfinite(bb)
    aa = aa[mask]
    bb = bb[mask]
    if aa.size < 3:
        return float("nan")
    aa = aa - np.mean(aa)
    bb = bb - np.mean(bb)
    den = float(np.linalg.norm(aa) * np.linalg.norm(bb))
    if den <= 1.0e-12:
        return float("nan")
    return float(np.dot(aa, bb) / den)


def sign_agreement(a: np.ndarray, b: np.ndarray) -> float:
    aa = np.asarray(a, dtype=float)
    bb = np.asarray(b, dtype=float)
    mask = np.isfinite(aa) & np.isfinite(bb) & (np.abs(aa) > 1.0e-12) & (np.abs(bb) > 1.0e-12)
    if not np.any(mask):
        return float("nan")
    return float(np.mean(np.sign(aa[mask]) == np.sign(bb[mask])))


def spectrum_metrics(values: np.ndarray, t: np.ndarray, cutoff_hz: float) -> dict[str, float]:
    vv = np.asarray(values, dtype=float)
    tt = np.asarray(t, dtype=float)
    mask = np.isfinite(vv) & np.isfinite(tt)
    vv = vv[mask]
    tt = tt[mask]
    if vv.size < 4:
        return {"dominant_freq_hz": float("nan"), "high_freq_power_ratio": float("nan")}
    dt = float(np.median(np.diff(tt)))
    if not np.isfinite(dt) or dt <= 0.0:
        return {"dominant_freq_hz": float("nan"), "high_freq_power_ratio": float("nan")}
    vv = vv - np.mean(vv)
    freqs = np.fft.rfftfreq(vv.size, d=dt)
    power = np.abs(np.fft.rfft(vv)) ** 2
    if power.size <= 1 or float(np.sum(power[1:])) <= 1.0e-18:
        return {"dominant_freq_hz": 0.0, "high_freq_power_ratio": 0.0}
    dominant_idx = 1 + int(np.argmax(power[1:]))
    total_power = float(np.sum(power[1:]))
    high_power = float(np.sum(power[(freqs >= cutoff_hz) & (freqs > 0.0)]))
    return {"dominant_freq_hz": float(freqs[dominant_idx]), "high_freq_power_ratio": high_power / total_power}


def weighted_pinv(aq: float, adq: float, wq: float, wdq: float) -> tuple[float, float]:
    den = wq * aq * aq + wdq * adq * adq
    if den <= 1.0e-18:
        return float("nan"), float("nan")
    return wq * aq / den, wdq * adq / den


def compute_config_inverse(config_path: Path) -> list[JointInverse]:
    cfg = read_yaml(config_path)
    top_dt = float(cfg.get("control_dt", 0.002))
    rows: list[JointInverse] = []
    joints = cfg["controller"]["joints"]
    for joint_id in active_joint_ids(cfg):
        joint = yaml_joint(joints, joint_id)
        if not joint or "plant" not in joint:
            continue
        params = merged_controller_params(cfg, joint)
        plant = joint["plant"]
        dt = float(params.get("control_dt", top_dt) or top_dt)
        Jeff = float(plant["Jeff"])
        aq = dt * dt / Jeff
        adq = dt / Jeff
        pinv_q, pinv_dq = weighted_pinv(aq, adq, 1.0, 1.0)
        wq = float(params.get("inverse_q_weight", 0.0) or 0.0)
        wdq = float(params.get("inverse_dq_weight", 0.0) or 0.0)
        if wq <= 0.0:
            wq = 0.5 / (dt * dt)
        if wdq <= 0.0:
            wdq = 1.0
        weighted_q, weighted_dq = weighted_pinv(aq, adq, wq, wdq)
        rows.append(
            JointInverse(
                config_path=config_path,
                joint_id=joint_id,
                joint_name=str(joint.get("name", f"joint_{joint_id}")),
                dt=dt,
                Jeff=Jeff,
                g_q=aq,
                g_dq=adq,
                pinv_q=pinv_q,
                pinv_dq=pinv_dq,
                weighted_pinv_q=weighted_q,
                weighted_pinv_dq=weighted_dq,
                w_q=wq,
                w_dq=wdq,
                ko_q=float(params.get("observer_gain_q", 0.0)),
                ko_dq=float(params.get("observer_gain_dq", 0.0)),
                ku_q=float(params.get("ku_q", 0.0)),
                ku_dq=float(params.get("ku_dq", 0.0)),
            )
        )
    return rows


def inverse_rows_to_records(rows: list[JointInverse]) -> list[dict[str, object]]:
    records: list[dict[str, object]] = []
    for r in rows:
        records.append(
            {
                "config_path": repo_relpath(r.config_path),
                "joint_id": r.joint_id,
                "joint_name": r.joint_name,
                "dt": r.dt,
                "Jeff": r.Jeff,
                "g_q": r.g_q,
                "g_dq": r.g_dq,
                "pinv_q": r.pinv_q,
                "pinv_dq": r.pinv_dq,
                "weighted_pinv_q": r.weighted_pinv_q,
                "weighted_pinv_dq": r.weighted_pinv_dq,
                "w_q": r.w_q,
                "w_dq": r.w_dq,
                "observer_gain_q": r.ko_q,
                "observer_gain_dq": r.ko_dq,
                "ku_q": r.ku_q,
                "ku_dq": r.ku_dq,
                "pinv_q_over_ku_q": r.pinv_q / r.ku_q if abs(r.ku_q) > 1.0e-12 else float("nan"),
                "pinv_dq_over_ku_dq": r.pinv_dq / r.ku_dq if abs(r.ku_dq) > 1.0e-12 else float("nan"),
                "weighted_pinv_q_over_ku_q": (
                    r.weighted_pinv_q / r.ku_q if abs(r.ku_q) > 1.0e-12 else float("nan")
                ),
                "weighted_pinv_dq_over_ku_dq": (
                    r.weighted_pinv_dq / r.ku_dq if abs(r.ku_dq) > 1.0e-12 else float("nan")
                ),
            }
        )
    return records


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if rows:
        fieldnames = list(rows[0].keys())
    else:
        fieldnames = []
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def config_for_log(log_path: Path, fallback: Path) -> Path:
    rows = read_csv_rows(log_path)
    first = rows[0] if rows else {}
    raw = first.get("config_path", "")
    if raw:
        candidate = resolve_path(Path(raw))
        if candidate.exists():
            return candidate
    return fallback


def to_float(row: dict[str, str], key: str) -> float:
    return float(row[key])


def replay_log(
    log_path: Path,
    inverses: dict[int, JointInverse],
    cutoff_hz: float,
    write_detail: bool,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    rows = read_csv_rows(log_path)
    if not rows:
        return [], []
    required = {"joint_id", "q", "dq", "t", "debug_9", "debug_10", "debug_11", "debug_12", "debug_32"}
    missing = sorted(required - set(rows[0].keys()))
    if missing:
        raise RuntimeError(f"{repo_relpath(log_path)} is missing required replay columns: {', '.join(missing)}")

    summary_rows: list[dict[str, object]] = []
    detail_rows: list[dict[str, object]] = []
    for joint_id, inv in sorted(inverses.items()):
        joint_rows = [r for r in rows if int(float(r["joint_id"])) == joint_id]
        if not joint_rows:
            continue
        if "cycle" in joint_rows[0]:
            joint_rows.sort(key=lambda r: int(float(r["cycle"])))

        t = np.asarray([to_float(r, "t") for r in joint_rows], dtype=float)
        t_rel = t - float(t[0])
        q = np.asarray([to_float(r, "q") for r in joint_rows], dtype=float)
        dq = np.asarray([to_float(r, "dq") for r in joint_rows], dtype=float)
        eta_q = np.asarray([to_float(r, "debug_9") for r in joint_rows], dtype=float)
        eta_dq = np.asarray([to_float(r, "debug_10") for r in joint_rows], dtype=float)
        x_hat_q = np.asarray([to_float(r, "debug_11") for r in joint_rows], dtype=float)
        x_hat_dq = np.asarray([to_float(r, "debug_12") for r in joint_rows], dtype=float)
        old_logged = np.asarray([to_float(r, "debug_32") for r in joint_rows], dtype=float)
        old_from_eta = inv.ku_q * eta_q + inv.ku_dq * eta_dq

        state_dist_q = inv.ko_q * (q - x_hat_q)
        state_dist_dq = inv.ko_dq * (dq - x_hat_dq)
        pinv_eta_u = inv.pinv_q * state_dist_q + inv.pinv_dq * state_dist_dq
        weighted_eta_u = inv.weighted_pinv_q * state_dist_q + inv.weighted_pinv_dq * state_dist_dq

        old_rms = finite_rms(old_logged)
        pinv_rms = finite_rms(pinv_eta_u)
        weighted_rms = finite_rms(weighted_eta_u)
        old_spec = spectrum_metrics(old_logged, t_rel, cutoff_hz)
        pinv_spec = spectrum_metrics(pinv_eta_u, t_rel, cutoff_hz)
        weighted_spec = spectrum_metrics(weighted_eta_u, t_rel, cutoff_hz)
        dt = float(np.median(np.diff(t_rel))) if t_rel.size > 2 else float("nan")
        summary_rows.append(
            {
                "log_path": repo_relpath(log_path),
                "config_path": repo_relpath(inv.config_path),
                "joint_id": joint_id,
                "joint_name": inv.joint_name,
                "n": int(len(joint_rows)),
                "dt_median": dt,
                "old_eta_u_rms": old_rms,
                "old_eta_u_peak_abs": finite_peak(old_logged),
                "old_from_eta_minus_logged_rms": finite_rms(old_from_eta - old_logged),
                "pinv_eta_u_rms": pinv_rms,
                "pinv_eta_u_peak_abs": finite_peak(pinv_eta_u),
                "weighted_eta_u_rms": weighted_rms,
                "weighted_eta_u_peak_abs": finite_peak(weighted_eta_u),
                "pinv_over_old_rms": pinv_rms / old_rms if old_rms > 1.0e-12 else float("nan"),
                "weighted_over_old_rms": weighted_rms / old_rms if old_rms > 1.0e-12 else float("nan"),
                "corr_old_pinv": correlation(old_logged, pinv_eta_u),
                "corr_old_weighted": correlation(old_logged, weighted_eta_u),
                "sign_agree_old_pinv": sign_agreement(old_logged, pinv_eta_u),
                "sign_agree_old_weighted": sign_agreement(old_logged, weighted_eta_u),
                "old_dominant_freq_hz": old_spec["dominant_freq_hz"],
                "pinv_dominant_freq_hz": pinv_spec["dominant_freq_hz"],
                "weighted_dominant_freq_hz": weighted_spec["dominant_freq_hz"],
                "old_high_freq_power_ratio": old_spec["high_freq_power_ratio"],
                "pinv_high_freq_power_ratio": pinv_spec["high_freq_power_ratio"],
                "weighted_high_freq_power_ratio": weighted_spec["high_freq_power_ratio"],
            }
        )

        if write_detail:
            for i in range(len(joint_rows)):
                detail_rows.append(
                    {
                        "log_path": repo_relpath(log_path),
                        "joint_id": joint_id,
                        "joint_name": inv.joint_name,
                        "t_rel": float(t_rel[i]),
                        "old_eta_u": float(old_logged[i]),
                        "old_eta_u_from_eta": float(old_from_eta[i]),
                        "pinv_eta_u": float(pinv_eta_u[i]),
                        "weighted_eta_u": float(weighted_eta_u[i]),
                        "state_dist_q": float(state_dist_q[i]),
                        "state_dist_dq": float(state_dist_dq[i]),
                    }
                )

    return summary_rows, detail_rows


def markdown_table(rows: list[dict[str, object]], columns: list[str], floatfmt: str = ".6g") -> str:
    if not rows:
        return "_No rows._"
    body_rows = []
    for row in rows:
        values = []
        for column in columns:
            value = row.get(column, "")
            if isinstance(value, (float, np.floating)):
                values.append("nan" if not math.isfinite(float(value)) else format(float(value), floatfmt))
            else:
                values.append(str(value))
        body_rows.append(values)
    header = "| " + " | ".join(columns) + " |"
    sep = "| " + " | ".join("---" for _ in columns) + " |"
    body = ["| " + " | ".join(values) + " |" for values in body_rows]
    return "\n".join([header, sep, *body])


def write_report(
    report_path: Path,
    out_dir: Path,
    config_rows: list[dict[str, object]],
    replay_rows: list[dict[str, object]],
) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    config_cols = [
        "joint_id",
        "joint_name",
        "Jeff",
        "g_q",
        "g_dq",
        "pinv_q",
        "pinv_dq",
        "weighted_pinv_q",
        "weighted_pinv_dq",
        "ku_q",
        "ku_dq",
    ]
    replay_cols = [
        "joint_id",
        "joint_name",
        "old_eta_u_rms",
        "pinv_eta_u_rms",
        "weighted_eta_u_rms",
        "pinv_over_old_rms",
        "weighted_over_old_rms",
        "corr_old_pinv",
        "old_high_freq_power_ratio",
        "pinv_high_freq_power_ratio",
    ]
    lines = [
        "# EID 输入逆分析",
        "",
        "本报告按当前单关节半隐式欧拉模型计算输入矩阵和伪逆：",
        "",
        "$$",
        "g=\\begin{bmatrix}T_s^2/J_\\mathrm{eff}\\\\T_s/J_\\mathrm{eff}\\end{bmatrix},\\qquad",
        "g^+=(g^Tg)^{-1}g^T",
        "$$",
        "",
        "离线重放项使用",
        "",
        "$$",
        "\\eta_u^{\\mathrm{pinv}}=g^+K_o(x-\\hat{x})",
        "$$",
        "",
        "该结果只用于离线量级检查，不直接替换实时控制器中的 `ku_q/ku_dq`。",
        "",
        "## Config Input Inverse",
        "",
        markdown_table(config_rows, config_cols),
        "",
    ]
    if replay_rows:
        lines.extend(["## Log Replay", "", markdown_table(replay_rows, replay_cols), ""])
    lines.extend(["## Artifacts", "", f"- `{repo_relpath(out_dir / 'eid_input_inverse_config_summary.csv')}`"])
    if replay_rows:
        lines.append(f"- `{repo_relpath(out_dir / 'eid_input_inverse_log_summary.csv')}`")
    detail_path = out_dir / "eid_input_inverse_detail.csv"
    if detail_path.exists():
        lines.append(f"- `{repo_relpath(detail_path)}`")
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    args = parse_args()
    config_paths = [resolve_path(p) for p in (args.config or [DEFAULT_CONFIG])]
    log_paths = [resolve_path(p) for p in args.log]
    out_dir = resolve_path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_inverse_rows: list[JointInverse] = []
    for config_path in config_paths:
        all_inverse_rows.extend(compute_config_inverse(config_path))
    if not all_inverse_rows:
        raise RuntimeError("No active configured joints with plant models were found.")

    config_records = inverse_rows_to_records(all_inverse_rows)
    config_csv = out_dir / "eid_input_inverse_config_summary.csv"
    write_csv(config_csv, config_records)

    replay_records: list[dict[str, object]] = []
    detail_records: list[dict[str, object]] = []
    fallback_config = config_paths[0]
    for log_path in log_paths:
        cfg_path = config_for_log(log_path, fallback_config)
        inverse_map = {r.joint_id: r for r in compute_config_inverse(cfg_path)}
        summary, detail = replay_log(log_path, inverse_map, args.high_freq_cutoff_hz, args.write_detail)
        replay_records.extend(summary)
        detail_records.extend(detail)

    if replay_records:
        write_csv(out_dir / "eid_input_inverse_log_summary.csv", replay_records)
    if detail_records:
        write_csv(out_dir / "eid_input_inverse_detail.csv", detail_records)

    report_path = analysis_report_dir(out_dir) / "eid_input_inverse_report.md"
    write_report(report_path, out_dir, config_records, replay_records)

    print(f"Wrote {repo_relpath(config_csv)}")
    if replay_records:
        print(f"Wrote {repo_relpath(out_dir / 'eid_input_inverse_log_summary.csv')}")
    if detail_records:
        print(f"Wrote {repo_relpath(out_dir / 'eid_input_inverse_detail.csv')}")
    print(f"Wrote {repo_relpath(report_path)}")


if __name__ == "__main__":
    main()
