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
                "q_error": q_ref - q,
                "dq_error": dq_ref - dq,
                "u_raw": finite_float(row.get("debug_25")),
                "q_ref_raw": finite_float(row.get("debug_26")),
                "dq_ref_raw": finite_float(row.get("debug_27")),
            })
    if not rows:
        raise RuntimeError(f"no EID rows parsed from {path}")

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
        "dq_ref_raw",
        "dq_ref",
        "dq",
        "dq_error",
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

    fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=True, constrained_layout=True)

    if has_raw:
        axes[0].plot(t, [row["q_ref_raw"] for row in rows], label="q_ref raw policy", linewidth=1.0, alpha=0.7)
    axes[0].plot(t, [row["q_ref"] for row in rows], label="q_ref shaped", linewidth=1.4)
    axes[0].plot(t, [row["q"] for row in rows], label="q measured", linewidth=1.1)
    axes[0].set_ylabel("q (rad)")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, [row["q_error"] for row in rows], color="#d62728", linewidth=1.0)
    axes[1].set_ylabel("q_ref - q")
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
    summary = {
        "samples": len(rows),
        "duration_s": rows[-1]["time_s"],
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
    return summary


def main():
    parser = argparse.ArgumentParser(description="Plot H1 EID CSV log reference and measured knee state.")
    parser.add_argument("log", nargs="?", default="h1_mock_log.csv", type=Path)
    parser.add_argument("--out", default="h1_eid_latest_plot.png", type=Path)
    parser.add_argument("--clean-csv", default="h1_eid_latest_clean.csv", type=Path)
    args = parser.parse_args()

    rows = load_rows(args.log)
    write_clean_csv(rows, args.clean_csv)
    plot_rows(rows, args.out)

    for key, value in summarize(rows).items():
        if isinstance(value, float):
            print(f"{key}={value:.6f}")
        else:
            print(f"{key}={value}")
    print(f"clean_csv={args.clean_csv}")
    print(f"plot={args.out}")


if __name__ == "__main__":
    main()
