#!/usr/bin/env python3
"""Run real H1 PD three-point MPC vs four-point velocity-MPC experiments.

Dry-run by default. Add --execute to command the real robot.
The manifest includes pure MPC solve-time statistics from mpc_solve_s.
"""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


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


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p3_mpc_pd_batch"
DEFAULT_DEADLINE_MS = 2.0


@dataclass(frozen=True)
class MethodSpec:
    token: str
    label: str
    condition: str
    config: Path
    mpc_kind: int


METHODS = {
    "three": MethodSpec(
        token="three",
        label="three_point_mpc",
        condition="P3_PD_preview_mpc_3ref_real",
        config=REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p3_selected_mpc_hip_knee_pd.yaml",
        mpc_kind=1,
    ),
    "four": MethodSpec(
        token="four",
        label="four_point_velocity_mpc",
        condition="P3_PD_velocity_mpc_4ref_real",
        config=REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p3_velocity_mpc_hip_knee_pd.yaml",
        mpc_kind=2,
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run P3 real H1 PD three-point MPC vs four-point velocity-MPC batch."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_P3_MPC confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--repeats", type=int, default=5, help="Repeats per method.")
    parser.add_argument("--duration", type=float, default=15.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument(
        "--methods",
        choices=["both", "three", "four"],
        default="both",
        help="Subset of methods to run.",
    )
    parser.add_argument(
        "--order",
        choices=["alternating", "three-then-four", "four-then-three"],
        default="alternating",
        help="Batch order. Default alternates three-point then four-point per repeat.",
    )
    parser.add_argument(
        "--deadline-ms",
        type=float,
        default=DEFAULT_DEADLINE_MS,
        help="MPC solve-time deadline in ms for manifest over-deadline counts.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if args.deadline_ms <= 0:
        raise SystemExit("--deadline-ms must be positive")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for spec in METHODS.values():
        if not spec.config.exists():
            raise SystemExit(f"config not found: {spec.config}")


def selected_methods(args: argparse.Namespace) -> list[str]:
    if args.methods == "three":
        return ["three"]
    if args.methods == "four":
        return ["four"]
    return ["three", "four"]


def build_plan(args: argparse.Namespace) -> list[tuple[int, MethodSpec]]:
    repeats = range(args.start_index, args.start_index + args.repeats)
    methods = selected_methods(args)
    plan: list[tuple[int, MethodSpec]] = []

    if args.order == "alternating":
        ordered_methods = [m for m in ["three", "four"] if m in methods]
        for repeat in repeats:
            for method in ordered_methods:
                plan.append((repeat, METHODS[method]))
        return plan

    ordered_methods = ["three", "four"] if args.order == "three-then-four" else ["four", "three"]
    for method in ordered_methods:
        if method not in methods:
            continue
        for repeat in repeats:
            plan.append((repeat, METHODS[method]))
    return plan


def command_for(args: argparse.Namespace, repeat: int, spec: MethodSpec) -> list[str]:
    cmd = [
        str(args.direct),
        str(spec.config.relative_to(REPO_ROOT)),
        "--duration",
        f"{args.duration:g}",
        "--condition",
        spec.condition,
        "--repeat",
        f"r{repeat:02d}",
        "--disturbance-target",
        "none",
        "--disturbance-method",
        "none",
    ]
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def config_log_name(spec: MethodSpec) -> str:
    for line in spec.config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            value = line.split(":", 1)[1].strip()
            return Path(value).name
    raise RuntimeError(f"{spec.config} missing log_path")


def finite_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        out = float(value)
    except ValueError:
        return math.nan
    return out if math.isfinite(out) else math.nan


def percentile(values: list[float], p: float) -> float:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return math.nan
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * p
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return clean[lo]
    alpha = rank - lo
    return (1.0 - alpha) * clean[lo] + alpha * clean[hi]


def mean(values: list[float]) -> float:
    clean = [v for v in values if math.isfinite(v)]
    return sum(clean) / len(clean) if clean else math.nan


def ms(value_s: float) -> float:
    return 1.0e3 * value_s if math.isfinite(value_s) else math.nan


def summarize_mpc_solve(log_path: Path, deadline_ms: float) -> dict[str, object]:
    deadline_s = deadline_ms * 1.0e-3
    rows_by_key: dict[tuple[int, int], dict[str, object]] = {}
    with log_path.open(newline="", encoding="utf-8-sig") as fh:
        reader = csv.DictReader(fh)
        if "mpc_solve_s" not in (reader.fieldnames or []):
            return {
                "mpc_log_schema": "missing_mpc_solve_s",
                "mpc_solve_count": 0,
                "mpc_failure_count": "",
                "mpc_solve_mean_ms": "",
                "mpc_solve_p95_ms": "",
                "mpc_solve_p99_ms": "",
                "mpc_solve_max_ms": "",
                "mpc_over_deadline_count": "",
            }
        for row in reader:
            try:
                cycle = int(row.get("cycle", ""))
                joint_id = int(row.get("joint_id", ""))
                ran = int(float(row.get("mpc_solve_ran", "0") or "0"))
                success = int(float(row.get("mpc_solve_success", "0") or "0"))
            except ValueError:
                continue
            solve_s = finite_float(row.get("mpc_solve_s"))
            if ran <= 0 or not math.isfinite(solve_s):
                continue
            rows_by_key[(cycle, joint_id)] = {
                "solve_s": solve_s,
                "success": success,
            }

    solves = list(rows_by_key.values())
    values = [float(r["solve_s"]) for r in solves]
    failures = [r for r in solves if int(r["success"]) == 0]
    over_deadline = [v for v in values if v > deadline_s]
    return {
        "mpc_log_schema": "ok",
        "mpc_solve_count": len(values),
        "mpc_failure_count": len(failures),
        "mpc_solve_mean_ms": ms(mean(values)),
        "mpc_solve_p95_ms": ms(percentile(values, 0.95)),
        "mpc_solve_p99_ms": ms(percentile(values, 0.99)),
        "mpc_solve_max_ms": ms(max(values) if values else math.nan),
        "mpc_over_deadline_count": len(over_deadline),
    }


def inspect_log_for_completion(
    path: Path,
    spec: MethodSpec,
    repeat: int,
    expected_duration: float,
) -> bool:
    expected_repeat = f"r{repeat:02d}"
    try:
        with path.open(newline="", encoding="utf-8-sig") as fh:
            reader = csv.DictReader(fh)
            first_cycle = None
            last_cycle = None
            dts: list[float] = []
            repeat_seen = ""
            condition_seen = ""
            target_seen = ""
            method_seen = ""
            rows = 0
            for row in reader:
                rows += 1
                if rows == 1:
                    repeat_seen = row.get("repeat_id", "")
                    condition_seen = row.get("condition_id", "")
                    target_seen = row.get("disturbance_target", "")
                    method_seen = row.get("disturbance_method", "")
                cycle = int(row["cycle"])
                first_cycle = cycle if first_cycle is None else min(first_cycle, cycle)
                last_cycle = cycle if last_cycle is None else max(last_cycle, cycle)
                if len(dts) < 2000:
                    dts.append(float(row["dt"]))
    except Exception as exc:
        print(f"Warning: could not inspect {path}: {exc}")
        return False

    if rows == 0 or first_cycle is None or last_cycle is None:
        return False
    if repeat_seen != expected_repeat or condition_seen != spec.condition:
        return False
    if target_seen != "none" or method_seen != "none":
        return False
    median_dt = statistics.median(dts) if dts else 0.0
    duration = max(0, last_cycle - first_cycle) * median_dt
    return duration >= 0.90 * expected_duration


def recent_completed_log(
    spec: MethodSpec,
    repeat: int,
    started_at: dt.datetime,
    expected_duration: float,
) -> Path | None:
    log_name = config_log_name(spec)
    candidates = sorted(
        (REPO_ROOT / "data").glob(f"*/{log_name}"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    min_mtime = started_at.timestamp() - 2.0

    for path in candidates:
        if path.stat().st_mtime < min_mtime:
            break
        if inspect_log_for_completion(path, spec, repeat, expected_duration):
            return path
    return None


def confirm_or_exit(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> None:
    print(
        f"P3 PD MPC batch plan: {len(plan)} runs, duration={args.duration:g}s, "
        f"pause={args.pause:g}s, deadline={args.deadline_ms:g}ms"
    )
    for i, (repeat, spec) in enumerate(plan, start=1):
        print(
            f"{i:02d}. {spec.label} r{repeat:02d}: "
            f"{spec.config.relative_to(REPO_ROOT)}"
        )

    if not args.execute:
        print("\nDry-run only. Add --execute to run the real robot batch.")
        return

    if args.yes:
        return

    print("\nThis will command the real robot with PD hip-knee tracking and no external disturbance.")
    print("Ensure suspension/limits, E-stop, and a watcher are ready.")
    answer = input("Type RUN_P3_MPC to start the batch: ").strip()
    if answer != "RUN_P3_MPC":
        raise SystemExit("confirmation failed; batch not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = args.out_dir / f"p3_mpc_pd_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    fieldnames = [
        "batch_time",
        "method",
        "repeat",
        "condition",
        "config",
        "duration_s",
        "pause_s",
        "deadline_ms",
        "command",
        "start_time",
        "end_time",
        "returncode",
        "status",
        "log_path",
        "mpc_log_schema",
        "mpc_solve_count",
        "mpc_failure_count",
        "mpc_solve_mean_ms",
        "mpc_solve_p95_ms",
        "mpc_solve_p99_ms",
        "mpc_solve_max_ms",
        "mpc_over_deadline_count",
    ]
    writer = csv.DictWriter(fh, fieldnames=fieldnames)
    writer.writeheader()
    return path, writer, fh


def run_batch(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> Path | None:
    if not args.execute:
        for repeat, spec in plan:
            print(shell_join(command_for(args, repeat, spec)))
        return None

    ensure_sudo(args)
    manifest_path, writer, fh = open_manifest(args)
    batch_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        for run_no, (repeat, spec) in enumerate(plan, start=1):
            cmd = command_for(args, repeat, spec)
            print(f"\n[{run_no}/{len(plan)}] {spec.label} r{repeat:02d} {spec.condition}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            proc = subprocess.run(
                cmd,
                input="\n",
                text=True,
                cwd=REPO_ROOT,
            )
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = recent_completed_log(spec, repeat, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print(
                    "Warning: h1_direct returned -6 after a complete log was written; "
                    f"continuing. log={completed_log}"
                )

            mpc_summary: dict[str, object] = {
                "mpc_log_schema": "",
                "mpc_solve_count": "",
                "mpc_failure_count": "",
                "mpc_solve_mean_ms": "",
                "mpc_solve_p95_ms": "",
                "mpc_solve_p99_ms": "",
                "mpc_solve_max_ms": "",
                "mpc_over_deadline_count": "",
            }
            if completed_log is not None:
                mpc_summary = summarize_mpc_solve(completed_log, args.deadline_ms)
                print(
                    "MPC solve: "
                    f"count={mpc_summary['mpc_solve_count']} "
                    f"p99={mpc_summary['mpc_solve_p99_ms']} ms "
                    f"max={mpc_summary['mpc_solve_max_ms']} ms "
                    f">deadline={mpc_summary['mpc_over_deadline_count']}"
                )

            writer.writerow(
                {
                    "batch_time": batch_time,
                    "method": spec.label,
                    "repeat": f"r{repeat:02d}",
                    "condition": spec.condition,
                    "config": str(spec.config.relative_to(REPO_ROOT)),
                    "duration_s": args.duration,
                    "pause_s": args.pause,
                    "deadline_ms": args.deadline_ms,
                    "command": shell_join(cmd),
                    "start_time": start.isoformat(timespec="seconds"),
                    "end_time": end.isoformat(timespec="seconds"),
                    "returncode": proc.returncode,
                    "status": status,
                    "log_path": str(completed_log.relative_to(REPO_ROOT)) if completed_log else "",
                    **mpc_summary,
                }
            )
            fh.flush()
            if proc.returncode != 0 and status != "accepted_after_complete_abort":
                raise SystemExit(f"run failed with return code {proc.returncode}; manifest={manifest_path}")
            if run_no < len(plan) and args.pause > 0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    validate_args(args)
    plan = build_plan(args)
    confirm_or_exit(args, plan)
    manifest_path = run_batch(args, plan)
    if manifest_path:
        print(f"\nBatch complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted.", file=sys.stderr)
        raise SystemExit(130)
