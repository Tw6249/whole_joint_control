#!/usr/bin/env python3
"""Slowly increase P1/P2 hip-knee speed and amplitude for real-robot tuning.

The script generates temporary configs from the checked-in P1/P2 base configs.
It does not overwrite the base configs. Dry-run is the default.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p1p2_tuning"


@dataclass(frozen=True)
class BaseSpec:
    experiment: str
    method: str
    config: Path
    condition_base: str


BASE_SPECS = {
    ("p1", "pd"): BaseSpec(
        experiment="p1",
        method="pd",
        config=REPO_ROOT / "config" / "h1_real_p1_lite_hip_knee_pd.yaml",
        condition_base="P1_PD_same_phase_no_disturbance",
    ),
    ("p1", "eid"): BaseSpec(
        experiment="p1",
        method="eid",
        config=REPO_ROOT / "config" / "h1_real_p1_lite_hip_knee_eid.yaml",
        condition_base="P1_EID_same_phase_no_disturbance",
    ),
    ("p2", "pd"): BaseSpec(
        experiment="p2",
        method="pd",
        config=REPO_ROOT / "config" / "h1_real_p2_anti_hip_knee_pd.yaml",
        condition_base="P2_PD_anti_phase_no_disturbance_tuning",
    ),
    ("p2", "eid"): BaseSpec(
        experiment="p2",
        method="eid",
        config=REPO_ROOT / "config" / "h1_real_p2_anti_hip_knee_eid.yaml",
        condition_base="P2_EID_anti_phase_no_disturbance_tuning",
    ),
}


@dataclass(frozen=True)
class GeneratedRun:
    spec: BaseSpec
    scale: float
    config: Path
    condition: str
    repeat: str
    frequency_hz: float
    hip_amplitude: float
    knee_amplitude: float
    hip_vmax: float
    knee_vmax: float


def parse_csv_floats(text: str) -> list[float]:
    values: list[float] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        value = float(part)
        if not math.isfinite(value) or value <= 0.0:
            raise argparse.ArgumentTypeError("stages must be positive finite numbers")
        values.append(value)
    if not values:
        raise argparse.ArgumentTypeError("at least one stage is required")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate and optionally run P1/P2 tuning configs with scaled speed and amplitude."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip initial RUN_TUNING confirmation.")
    parser.add_argument("--auto-continue", action="store_true", help="Do not pause after each stage.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument(
        "--experiments",
        choices=["p1", "p2", "both"],
        default="both",
        help="Experiment set to tune.",
    )
    parser.add_argument(
        "--methods",
        choices=["pd", "eid", "both"],
        default="both",
        help="Controller methods to test.",
    )
    parser.add_argument(
        "--stages",
        type=parse_csv_floats,
        default=parse_csv_floats("1.0,1.1,1.2,1.3,1.4"),
        help="Comma-separated multipliers applied to both frequency and amplitude.",
    )
    parser.add_argument("--duration", type=float, default=6.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument(
        "--speed-trip-ratio",
        type=float,
        default=0.85,
        help="Reject stages whose sine reference peak speed exceeds this fraction of measured_speed_trip.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT_DIR,
        help="Generated config and manifest output directory.",
    )
    return parser.parse_args()


def selected_experiments(args: argparse.Namespace) -> list[str]:
    return ["p1", "p2"] if args.experiments == "both" else [args.experiments]


def selected_methods(args: argparse.Namespace) -> list[str]:
    return ["pd", "eid"] if args.methods == "both" else [args.methods]


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0.0:
        raise SystemExit("--pause must be non-negative")
    if not (0.0 < args.speed_trip_ratio <= 1.0):
        raise SystemExit("--speed-trip-ratio must be in (0, 1]")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for exp in selected_experiments(args):
        for method in selected_methods(args):
            spec = BASE_SPECS[(exp, method)]
            if not spec.config.exists():
                raise SystemExit(f"config not found: {spec.config}")


def load_yaml(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise RuntimeError(f"{path} is not a YAML mapping")
    return data


def format_scale(scale: float) -> str:
    text = f"{scale:.3f}".rstrip("0").rstrip(".")
    return text.replace(".", "p")


def joint_amplitude(data: dict[str, Any], joint_id: int) -> float:
    return float(data["controller"]["joints"][joint_id]["policy_amplitude"])


def joint_center(data: dict[str, Any], joint_id: int) -> float:
    return float(data["controller"]["joints"][joint_id]["policy_center"])


def joint_limit(data: dict[str, Any], joint_id: int, key: str) -> float:
    limits = data.get("joint_limits", {}).get(joint_id)
    if limits and key in limits:
        return float(limits[key])
    return float(data["controller"]["joints"][joint_id]["plant"][key])


def reference_vmax(frequency_hz: float, amplitude: float) -> float:
    return 2.0 * math.pi * frequency_hz * amplitude


def validate_scaled_config(data: dict[str, Any], source: Path, speed_trip_ratio: float) -> None:
    freq = float(data["controller"]["defaults"]["policy_frequency_hz"])
    speed_trip = float(data["safe_hold"]["measured_speed_trip"])
    speed_limit = speed_trip_ratio * speed_trip
    for joint_id in [1, 2]:
        center = joint_center(data, joint_id)
        amp = joint_amplitude(data, joint_id)
        q_min = joint_limit(data, joint_id, "q_min")
        q_max = joint_limit(data, joint_id, "q_max")
        if center - amp < q_min or center + amp > q_max:
            raise RuntimeError(
                f"{source}: joint {joint_id} reference range "
                f"[{center - amp:.4f}, {center + amp:.4f}] exceeds [{q_min:.4f}, {q_max:.4f}]"
            )
        vmax = reference_vmax(freq, amp)
        if vmax > speed_limit:
            raise RuntimeError(
                f"{source}: joint {joint_id} sine reference peak speed {vmax:.3f} rad/s "
                f"exceeds {speed_trip_ratio:.2f} * measured_speed_trip = {speed_limit:.3f} rad/s"
            )


def make_scaled_config(spec: BaseSpec, scale: float, out_dir: Path) -> GeneratedRun:
    data = load_yaml(spec.config)
    data["controller"]["defaults"]["policy_frequency_hz"] = (
        float(data["controller"]["defaults"]["policy_frequency_hz"]) * scale
    )
    for joint_id in [1, 2]:
        data["controller"]["joints"][joint_id]["policy_amplitude"] = (
            float(data["controller"]["joints"][joint_id]["policy_amplitude"]) * scale
        )
    scale_id = format_scale(scale)
    condition = f"{spec.condition_base}_scale_x{scale_id}"
    data["experiment"]["condition"] = condition
    data["experiment"]["repeat"] = "tune01"
    data["experiment"]["disturbance_target"] = "none"
    data["experiment"]["disturbance_method"] = "none"
    config_path = out_dir / "configs" / f"{spec.config.stem}_scale_x{scale_id}.yaml"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    validate_scaled_config(data, spec.config, speed_trip_ratio=make_scaled_config.speed_trip_ratio)
    with config_path.open("w", encoding="utf-8") as fh:
        yaml.safe_dump(data, fh, sort_keys=False, allow_unicode=True)
    freq = float(data["controller"]["defaults"]["policy_frequency_hz"])
    hip_amp = joint_amplitude(data, 1)
    knee_amp = joint_amplitude(data, 2)
    return GeneratedRun(
        spec=spec,
        scale=scale,
        config=config_path,
        condition=condition,
        repeat="tune01",
        frequency_hz=freq,
        hip_amplitude=hip_amp,
        knee_amplitude=knee_amp,
        hip_vmax=reference_vmax(freq, hip_amp),
        knee_vmax=reference_vmax(freq, knee_amp),
    )


make_scaled_config.speed_trip_ratio = 0.85  # type: ignore[attr-defined]


def build_plan(args: argparse.Namespace) -> list[GeneratedRun]:
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir / stamp
    make_scaled_config.speed_trip_ratio = args.speed_trip_ratio  # type: ignore[attr-defined]
    plan: list[GeneratedRun] = []
    for scale in args.stages:
        for exp in selected_experiments(args):
            for method in selected_methods(args):
                plan.append(make_scaled_config(BASE_SPECS[(exp, method)], scale, out_dir))
    return plan


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def command_for(args: argparse.Namespace, run: GeneratedRun) -> list[str]:
    cmd = [
        str(args.direct),
        str(run.config.relative_to(REPO_ROOT)),
        "--duration",
        f"{args.duration:g}",
        "--condition",
        run.condition,
        "--repeat",
        run.repeat,
        "--disturbance-target",
        "none",
        "--disturbance-method",
        "none",
    ]
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def config_log_name(config_path: Path) -> str:
    data = load_yaml(config_path)
    return Path(str(data["log_path"])).name


def recent_completed_log(run: GeneratedRun,
                         started_at: dt.datetime,
                         expected_duration: float) -> Path | None:
    log_name = config_log_name(run.config)
    candidates = sorted((REPO_ROOT / "data").glob(f"*/{log_name}"),
                        key=lambda p: p.stat().st_mtime,
                        reverse=True)
    min_mtime = started_at.timestamp() - 2.0
    for path in candidates:
        if path.stat().st_mtime < min_mtime:
            break
        try:
            with path.open(newline="", encoding="utf-8") as fh:
                reader = csv.DictReader(fh)
                first_cycle = None
                last_cycle = None
                dts: list[float] = []
                repeat_seen = ""
                condition_seen = ""
                rows = 0
                for row in reader:
                    rows += 1
                    if rows == 1:
                        repeat_seen = row.get("repeat_id", "")
                        condition_seen = row.get("condition_id", "")
                    cycle = int(row["cycle"])
                    first_cycle = cycle if first_cycle is None else min(first_cycle, cycle)
                    last_cycle = cycle if last_cycle is None else max(last_cycle, cycle)
                    if len(dts) < 2000:
                        dts.append(float(row["dt"]))
        except Exception as exc:
            print(f"Warning: could not inspect {path}: {exc}")
            continue
        if rows == 0 or first_cycle is None or last_cycle is None:
            continue
        if repeat_seen != run.repeat or condition_seen != run.condition:
            continue
        median_dt = statistics.median(dts) if dts else 0.0
        duration = max(0, last_cycle - first_cycle) * median_dt
        if duration >= 0.90 * expected_duration:
            return path
    return None


def print_plan(args: argparse.Namespace, plan: list[GeneratedRun]) -> None:
    print(
        f"P1/P2 tuning plan: {len(plan)} runs, duration={args.duration:g}s, "
        f"pause={args.pause:g}s, speed_trip_ratio={args.speed_trip_ratio:.2f}"
    )
    last_scale: float | None = None
    for i, run in enumerate(plan, start=1):
        if run.scale != last_scale:
            print(f"\nStage x{run.scale:g}")
            last_scale = run.scale
        print(
            f"{i:02d}. {run.spec.experiment.upper()} {run.spec.method.upper()} "
            f"freq={run.frequency_hz:.4f}Hz hip_amp={run.hip_amplitude:.5f} "
            f"knee_amp={run.knee_amplitude:.5f} vmax=({run.hip_vmax:.3f},{run.knee_vmax:.3f})"
        )
        print(f"    {run.config.relative_to(REPO_ROOT)}")
    if not args.execute:
        print("\nDry-run only. Commands:")
        for run in plan:
            print(shell_join(command_for(args, run)))
        print("\nAdd --execute to command the real robot.")


def confirm_or_exit(args: argparse.Namespace) -> None:
    if not args.execute or args.yes:
        return
    print("\nThis will command the real robot. Ensure suspension/limits, E-stop, and a watcher are ready.")
    print("After each scale stage, the script will ask whether to continue.")
    answer = input("Type RUN_TUNING to start: ").strip()
    if answer != "RUN_TUNING":
        raise SystemExit("confirmation failed; tuning not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace, first_config: Path) -> tuple[Path, csv.DictWriter, object]:
    run_dir = first_config.parent.parent
    manifest_path = run_dir / "p1p2_tuning_manifest.csv"
    fh = manifest_path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "run_time",
            "experiment",
            "method",
            "scale",
            "frequency_hz",
            "hip_amplitude",
            "knee_amplitude",
            "hip_vmax",
            "knee_vmax",
            "config",
            "condition",
            "repeat",
            "duration_s",
            "command",
            "start_time",
            "end_time",
            "returncode",
            "status",
            "log_path",
        ],
    )
    writer.writeheader()
    return manifest_path, writer, fh


def prompt_continue(args: argparse.Namespace, scale: float) -> None:
    if args.auto_continue:
        return
    answer = input(
        f"\nStage x{scale:g} finished. Press Enter for next stage, or type STOP if satisfied/not safe: "
    ).strip()
    if answer.upper() == "STOP":
        raise SystemExit("stopped after user review")


def run_plan(args: argparse.Namespace, plan: list[GeneratedRun]) -> Path | None:
    if not args.execute:
        return None
    ensure_sudo(args)
    manifest_path, writer, fh = open_manifest(args, plan[0].config)
    run_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        previous_scale: float | None = None
        for i, run in enumerate(plan, start=1):
            if previous_scale is not None and run.scale != previous_scale:
                prompt_continue(args, previous_scale)
            previous_scale = run.scale
            cmd = command_for(args, run)
            print(
                f"\n[{i}/{len(plan)}] {run.spec.experiment.upper()} {run.spec.method.upper()} "
                f"stage x{run.scale:g}"
            )
            print(shell_join(cmd))
            start = dt.datetime.now()
            proc = subprocess.run(cmd, input="\n", text=True, cwd=REPO_ROOT)
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = recent_completed_log(run, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print(
                    "Warning: h1_direct returned -6 after a complete log was written; "
                    f"continuing. log={completed_log}"
                )
            writer.writerow(
                {
                    "run_time": run_time,
                    "experiment": run.spec.experiment.upper(),
                    "method": run.spec.method.upper(),
                    "scale": run.scale,
                    "frequency_hz": run.frequency_hz,
                    "hip_amplitude": run.hip_amplitude,
                    "knee_amplitude": run.knee_amplitude,
                    "hip_vmax": run.hip_vmax,
                    "knee_vmax": run.knee_vmax,
                    "config": str(run.config.relative_to(REPO_ROOT)),
                    "condition": run.condition,
                    "repeat": run.repeat,
                    "duration_s": args.duration,
                    "command": shell_join(cmd),
                    "start_time": start.isoformat(timespec="seconds"),
                    "end_time": end.isoformat(timespec="seconds"),
                    "returncode": proc.returncode,
                    "status": status,
                    "log_path": str(completed_log.relative_to(REPO_ROOT)) if completed_log else "",
                }
            )
            fh.flush()
            if proc.returncode != 0 and status != "accepted_after_complete_abort":
                raise SystemExit(f"run failed with return code {proc.returncode}; manifest={manifest_path}")
            if i < len(plan) and args.pause > 0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
        if previous_scale is not None:
            prompt_continue(args, previous_scale)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    validate_args(args)
    plan = build_plan(args)
    print_plan(args, plan)
    confirm_or_exit(args)
    manifest_path = run_plan(args, plan)
    if manifest_path:
        print(f"\nTuning complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
