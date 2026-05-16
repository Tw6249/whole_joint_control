#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def finite_float(value, default=math.nan):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def rms(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return math.nan
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def choose_error_reference(rows):
    shaped = [row["q_error_shaped"] for row in rows]
    raw = [row["q_error_raw"] for row in rows]
    if any(math.isfinite(value) for value in raw) and rms(shaped) < 1.0e-9 and rms(raw) > 1.0e-9:
        return "raw"
    return "shaped"


def load_rows(path):
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["t"])
                q = float(row["q"])
                dq = float(row["dq"])
                tau_cmd = float(row["tau_cmd"])
                q_ref = float(row["debug_0"])
                dq_ref = float(row["debug_1"])
            except (KeyError, ValueError):
                continue

            rows.append({
                "cycle": int(float(row.get("cycle", 0))),
                "t": t,
                "dt": finite_float(row.get("dt")),
                "lowstate_age": finite_float(row.get("lowstate_age")),
                "joint_id": int(float(row.get("joint_id", 0))),
                "q": q,
                "dq": dq,
                "tau_est": finite_float(row.get("tau_est")),
                "tau_cmd": tau_cmd,
                "flags": int(float(row.get("flags", 0))),
                "q_ref": q_ref,
                "dq_ref": dq_ref,
                "q_error_shaped": q_ref - q,
                "dq_error_shaped": dq_ref - dq,
                "u_raw": finite_float(row.get("debug_25")),
                "e_q": finite_float(row.get("debug_21")),
                "e_dq": finite_float(row.get("debug_22")),
                "q_ref_raw": finite_float(row.get("debug_26")),
                "dq_ref_raw": finite_float(row.get("debug_27")),
                "q_error_raw": math.nan,
                "dq_error_raw": math.nan,
            })
    if not rows:
        raise RuntimeError(f"no EID rows parsed from {path}")

    for row in rows:
        if not math.isfinite(row["q_error_raw"]) and math.isfinite(row["q_ref_raw"]):
            row["q_error_raw"] = row["q_ref_raw"] - row["q"]
        if not math.isfinite(row["dq_error_raw"]) and math.isfinite(row["dq_ref_raw"]):
            row["dq_error_raw"] = row["dq_ref_raw"] - row["dq"]

    error_reference = choose_error_reference(rows)
    for row in rows:
        row["error_reference"] = error_reference
        if error_reference == "raw":
            row["q_error"] = row["q_error_raw"]
            row["dq_error"] = row["dq_error_raw"]
        else:
            row["q_error"] = row["q_error_shaped"]
            row["dq_error"] = row["dq_error_shaped"]

    t0 = rows[0]["t"]
    for row in rows:
        row["time_s"] = row["t"] - t0
    return rows


def write_clean_csv(rows, path):
    columns = [
        "time_s",
        "cycle",
        "joint_id",
        "q_ref_raw",
        "q_ref",
        "q",
        "q_error",
        "q_error_raw",
        "q_error_shaped",
        "dq_ref_raw",
        "dq_ref",
        "dq",
        "dq_error",
        "dq_error_raw",
        "dq_error_shaped",
        "e_q",
        "e_dq",
        "error_reference",
        "tau_cmd",
        "u_raw",
        "tau_est",
        "lowstate_age",
        "dt",
        "flags",
    ]
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows({key: row.get(key, "") for key in columns} for row in rows)


def plot_rows(rows, path):
    t = [row["time_s"] for row in rows]
    has_raw = any(math.isfinite(row["q_ref_raw"]) for row in rows)
    error_reference = rows[0].get("error_reference", "shaped")

    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True, constrained_layout=True)

    if has_raw:
        axes[0].plot(t, [row["q_ref_raw"] for row in rows], label="q_ref raw policy", linewidth=1.0, alpha=0.7)
    axes[0].plot(t, [row["q_ref"] for row in rows], label="q_ref shaped", linewidth=1.4)
    axes[0].plot(t, [row["q"] for row in rows], label="q measured", linewidth=1.1)
    axes[0].set_ylabel("q (rad)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    error_label = "raw_ref - q" if error_reference == "raw" else "shaped_ref - q"
    axes[1].plot(t, [row["q_error"] for row in rows], label=error_label, color="#d62728", linewidth=1.0)
    if error_reference == "raw":
        axes[1].plot(t, [row["q_error_shaped"] for row in rows],
                     label="shaped_ref - q", color="#7f7f7f", linewidth=0.8, alpha=0.55)
    elif has_raw:
        axes[1].plot(t, [row["q_error_raw"] for row in rows],
                     label="raw_ref - q", color="#7f7f7f", linewidth=0.8, alpha=0.55)
    if any(math.isfinite(row["e_q"]) for row in rows):
        axes[1].plot(t, [row["e_q"] for row in rows],
                     label="controller e_q", color="#1f77b4", linewidth=0.8, alpha=0.7)
    axes[1].set_ylabel(error_label)
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    if has_raw:
        axes[2].plot(t, [row["dq_ref_raw"] for row in rows], label="dq_ref raw", linewidth=0.9, alpha=0.7)
    axes[2].plot(t, [row["dq_ref"] for row in rows], label="dq_ref shaped", linewidth=1.2)
    axes[2].plot(t, [row["dq"] for row in rows], label="dq measured", linewidth=1.0)
    axes[2].set_ylabel("dq (rad/s)")
    axes[2].legend(loc="best")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, [row["tau_cmd"] for row in rows], label="tau_cmd", color="#9467bd", linewidth=1.0)
    if any(math.isfinite(row["u_raw"]) for row in rows):
        axes[3].plot(t, [row["u_raw"] for row in rows], label="u_raw before limit", color="#8c564b", linewidth=0.8, alpha=0.6)
    axes[3].set_ylabel("torque (N m)")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)

    axes[4].plot(t, [row["lowstate_age"] for row in rows], label="LowState age", linewidth=0.9)
    axes[4].plot(t, [row["flags"] for row in rows], label="flags", linewidth=0.8, alpha=0.7)
    axes[4].set_ylabel("age / flags")
    axes[4].set_xlabel("time since log start (s)")
    axes[4].legend(loc="best")
    axes[4].grid(True, alpha=0.3)

    fig.suptitle("H1 EID Reference vs Measured Knee State")
    fig.savefig(path, dpi=160)


def summarize(rows):
    def values(key):
        return [row[key] for row in rows if isinstance(row.get(key), (int, float)) and math.isfinite(row[key])]

    q_err = values("q_error")
    q_err_raw = values("q_error_raw")
    q_err_shaped = values("q_error_shaped")
    summary = {
        "samples": len(rows),
        "duration_s": rows[-1]["time_s"],
        "q_error_reference": rows[0].get("error_reference", "shaped"),
        "q_ref_min": min(values("q_ref")),
        "q_ref_max": max(values("q_ref")),
        "q_min": min(values("q")),
        "q_max": max(values("q")),
        "q_error_rmse": math.sqrt(sum(e * e for e in q_err) / len(q_err)),
        "tau_cmd_min": min(values("tau_cmd")),
        "tau_cmd_max": max(values("tau_cmd")),
        "lowstate_age_max": max(values("lowstate_age")),
        "flags": sorted(set(row["flags"] for row in rows)),
    }
    raw = values("q_ref_raw")
    if raw:
        summary["q_ref_raw_min"] = min(raw)
        summary["q_ref_raw_max"] = max(raw)
    if q_err_raw:
        summary["q_error_raw_rmse"] = math.sqrt(sum(e * e for e in q_err_raw) / len(q_err_raw))
    if q_err_shaped:
        summary["q_error_shaped_rmse"] = math.sqrt(sum(e * e for e in q_err_shaped) / len(q_err_shaped))
    e_q = values("e_q")
    if e_q:
        summary["controller_e_q_rmse"] = math.sqrt(sum(e * e for e in e_q) / len(e_q))
    return summary


def main():
    parser = argparse.ArgumentParser(description="Plot H1 EID CSV log reference and measured knee state.")
    parser.add_argument("log", nargs="?", default="data/h1_mock_log.csv", type=Path)
    parser.add_argument("--out", default=None, type=Path,
                        help="Output plot path (default: same dir as log / h1_eid_latest_plot.png)")
    parser.add_argument("--clean-csv", default=None, type=Path,
                        help="Output clean CSV path (default: same dir as log / h1_eid_latest_clean.csv)")
    args = parser.parse_args()

    log_dir = args.log.parent
    out_path = args.out if args.out is not None else log_dir / "h1_eid_latest_plot.png"
    clean_csv_path = args.clean_csv if args.clean_csv is not None else log_dir / "h1_eid_latest_clean.csv"

    rows = load_rows(args.log)
    write_clean_csv(rows, clean_csv_path)
    plot_rows(rows, out_path)

    for key, value in summarize(rows).items():
        if isinstance(value, float):
            print(f"{key}={value:.6f}")
        else:
            print(f"{key}={value}")
    print(f"clean_csv={clean_csv_path}")
    print(f"plot={out_path}")


if __name__ == "__main__":
    main()
