#!/usr/bin/env python3
"""Run the P6.1 same-batch real H1 EID retest matrix.

Default plan runs all six candidates:
  A, B, E, C, D, knee_dq_down.

Each candidate first receives no-disturbance gate runs, followed by software
torque disturbance runs. The disturbance runs are interleaved by repeat index to
reduce batch drift.
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
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p61_batch"
DEFAULT_DISTURBANCE_TARGET = "hip_knee"
DEFAULT_DISTURBANCE_METHOD = "software_load_torque_pulse"


@dataclass(frozen=True)
class CandidateSpec:
    token: str
    label: str
    role: str
    config: Path
    hip_ko_q: float
    hip_ko_dq: float
    knee_ko_q: float
    knee_ko_dq: float
    ku_q: float = 6.0
    ku_dq: float = 0.5


CANDIDATES = [
    CandidateSpec(
        "a",
        "A_O4_U1",
        "conservative_baseline",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_a_o4_u1_hip_knee_eid.yaml",
        0.8,
        0.2,
        0.8,
        0.2,
    ),
    CandidateSpec(
        "b",
        "B_hipO5_kneeO4_U1",
        "hip_ko_increase",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_b_hip_o5_knee_o4_u1_hip_knee_eid.yaml",
        1.0,
        0.25,
        0.8,
        0.2,
    ),
    CandidateSpec(
        "e",
        "E_O5_U1",
        "shared_high_ko",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_e_o5_u1_hip_knee_eid.yaml",
        1.0,
        0.25,
        1.0,
        0.25,
    ),
    CandidateSpec(
        "c",
        "C_hipO4_kneeO3_U1",
        "knee_ko_decrease",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_c_hip_o4_knee_o3_u1_hip_knee_eid.yaml",
        0.8,
        0.2,
        0.6,
        0.15,
    ),
    CandidateSpec(
        "d",
        "D_hipO5_kneeO3_U1",
        "hip_up_knee_down",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_d_hip_o5_knee_o3_u1_hip_knee_eid.yaml",
        1.0,
        0.25,
        0.6,
        0.15,
    ),
    CandidateSpec(
        "knee_dq_down",
        "kneeDqDown_U1",
        "knee_velocity_channel_decrease",
        REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p61_knee_dq_down_u1_hip_knee_eid.yaml",
        0.8,
        0.2,
        0.8,
        0.15,
    ),
]

SPEC_BY_TOKEN = {
    token: spec
    for spec in CANDIDATES
    for token in (
        spec.token,
        spec.label.lower(),
    )
}


@dataclass(frozen=True)
class RunSpec:
    stage: str
    repeat: int
    candidate: CandidateSpec
    disturbed: bool


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run P6.1 same-batch real H1 EID retest matrix.")
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_P61 confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument(
        "--plan",
        choices=["full", "priority", "asymmetric"],
        default="full",
        help="full=A/B/E/C/D/knee_dq_down; priority=A/B/E; asymmetric=A/B/C/D.",
    )
    parser.add_argument(
        "--candidates",
        default="",
        help="Comma-separated override, e.g. a,b,e,c,d,knee_dq_down.",
    )
    parser.add_argument("--gate-repeats", type=int, default=2, help="No-disturbance gate repeats per candidate.")
    parser.add_argument(
        "--disturbance-repeats",
        type=int,
        default=5,
        help="Software-disturbance repeats per candidate. Use 0 to run gate-only checks.",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument(
        "--order",
        choices=["interleaved", "grouped"],
        default="interleaved",
        help="Disturbance-stage order. Default interleaves candidates by repeat index.",
    )
    parser.add_argument(
        "--disturbance-method",
        default=DEFAULT_DISTURBANCE_METHOD,
        help="Metadata recorded in disturbance_method for disturbed runs.",
    )
    parser.add_argument("--disturbance-joints", default="1,2", help="Comma-separated disturbed joint ids.")
    parser.add_argument("--disturbance-torques", default="6,-4", help="Comma-separated disturbance torques [N*m].")
    parser.add_argument("--disturbance-start", type=float, default=4.0, help="Pulse start time [s].")
    parser.add_argument("--disturbance-plateau-start", type=float, default=4.2, help="Pulse plateau start [s].")
    parser.add_argument("--disturbance-plateau-end", type=float, default=5.2, help="Pulse plateau end [s].")
    parser.add_argument("--disturbance-end", type=float, default=5.4, help="Pulse end time [s].")
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def selected_candidates(args: argparse.Namespace) -> list[CandidateSpec]:
    if args.candidates.strip():
        selected: list[CandidateSpec] = []
        seen: set[str] = set()
        for raw in args.candidates.split(","):
            token = raw.strip().lower()
            if not token:
                continue
            if token not in SPEC_BY_TOKEN:
                choices = ", ".join(spec.token for spec in CANDIDATES)
                raise SystemExit(f"unknown candidate '{raw}'. Choices: {choices}")
            spec = SPEC_BY_TOKEN[token]
            if spec.token not in seen:
                selected.append(spec)
                seen.add(spec.token)
        if not selected:
            raise SystemExit("--candidates did not contain any valid candidates")
        return selected

    if args.plan == "priority":
        tokens = {"a", "b", "e"}
    elif args.plan == "asymmetric":
        tokens = {"a", "b", "c", "d"}
    else:
        tokens = {spec.token for spec in CANDIDATES}
    return [spec for spec in CANDIDATES if spec.token in tokens]


def validate_args(args: argparse.Namespace, candidates: list[CandidateSpec]) -> None:
    if args.gate_repeats < 0:
        raise SystemExit("--gate-repeats must be non-negative")
    if args.disturbance_repeats < 0:
        raise SystemExit("--disturbance-repeats must be non-negative")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if args.disturbance_method.strip() in {"", "none"}:
        raise SystemExit("--disturbance-method must be non-empty")
    if not (0.0 <= args.disturbance_start <= args.disturbance_plateau_start <= args.disturbance_plateau_end < args.disturbance_end):
        raise SystemExit("disturbance timing must satisfy 0 <= start <= plateau-start <= plateau-end < end")
    if args.disturbance_end > args.duration:
        raise SystemExit("--disturbance-end must not exceed --duration")
    joints = [part for part in args.disturbance_joints.split(",") if part.strip()]
    torques = [part for part in args.disturbance_torques.split(",") if part.strip()]
    if not joints or len(joints) != len(torques):
        raise SystemExit("--disturbance-joints and --disturbance-torques must be non-empty lists of the same length")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for spec in candidates:
        if not spec.config.exists():
            raise SystemExit(f"config not found: {spec.config}")


def build_plan(args: argparse.Namespace, candidates: list[CandidateSpec]) -> list[RunSpec]:
    plan: list[RunSpec] = []
    for repeat in range(args.start_index, args.start_index + args.gate_repeats):
        for spec in candidates:
            plan.append(RunSpec("gate", repeat, spec, False))

    disturbed_indices = range(args.start_index, args.start_index + args.disturbance_repeats)
    if args.order == "grouped":
        for spec in candidates:
            for repeat in disturbed_indices:
                plan.append(RunSpec("disturbance", repeat, spec, True))
    else:
        for repeat in disturbed_indices:
            for spec in candidates:
                plan.append(RunSpec("disturbance", repeat, spec, True))
    return plan


def condition_for(run: RunSpec) -> str:
    suffix = "software_disturbance" if run.disturbed else "nodist"
    return f"P61_{run.candidate.label}_{suffix}"


def software_disturbance_args(args: argparse.Namespace) -> list[str]:
    return [
        "--software-disturbance-joints",
        args.disturbance_joints,
        "--software-disturbance-torques",
        args.disturbance_torques,
        "--software-disturbance-start",
        f"{args.disturbance_start:g}",
        "--software-disturbance-plateau-start",
        f"{args.disturbance_plateau_start:g}",
        "--software-disturbance-plateau-end",
        f"{args.disturbance_plateau_end:g}",
        "--software-disturbance-end",
        f"{args.disturbance_end:g}",
    ]


def command_for(args: argparse.Namespace, run: RunSpec) -> list[str]:
    disturbance_target = DEFAULT_DISTURBANCE_TARGET if run.disturbed else "none"
    disturbance_method = args.disturbance_method if run.disturbed else "none"
    cmd = [
        str(args.direct),
        str(run.candidate.config.relative_to(REPO_ROOT)),
        "--duration",
        f"{args.duration:g}",
        "--experiment",
        "P6.1",
        "--condition",
        condition_for(run),
        "--repeat",
        f"r{run.repeat:02d}",
        "--disturbance-target",
        disturbance_target,
        "--disturbance-method",
        disturbance_method,
    ]
    if run.disturbed:
        cmd.extend(software_disturbance_args(args))
    if not args.no_sudo:
        cmd = ["sudo", "-n", *cmd]
    return cmd


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def config_log_name(spec: CandidateSpec) -> str:
    for line in spec.config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            value = line.split(":", 1)[1].strip()
            return Path(value).name
    raise RuntimeError(f"{spec.config} missing log_path")


def recent_completed_log(
    args: argparse.Namespace,
    run: RunSpec,
    started_at: dt.datetime,
    expected_duration: float,
) -> Path | None:
    log_name = config_log_name(run.candidate)
    candidates = sorted(
        (REPO_ROOT / "data").glob(f"*/{log_name}"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    min_mtime = started_at.timestamp() - 2.0
    expected_repeat = f"r{run.repeat:02d}"
    expected_condition = condition_for(run)
    expected_target = DEFAULT_DISTURBANCE_TARGET if run.disturbed else "none"
    expected_method = args.disturbance_method if run.disturbed else "none"

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
            continue

        if rows == 0 or first_cycle is None or last_cycle is None:
            continue
        if repeat_seen != expected_repeat or condition_seen != expected_condition:
            continue
        if target_seen != expected_target or method_seen != expected_method:
            continue
        median_dt = statistics.median(dts) if dts else 0.0
        duration = max(0, last_cycle - first_cycle) * median_dt
        if duration >= 0.90 * expected_duration:
            return path
    return None


def confirm_or_exit(args: argparse.Namespace, plan: list[RunSpec]) -> None:
    gate_runs = sum(1 for run in plan if not run.disturbed)
    disturbance_runs = sum(1 for run in plan if run.disturbed)
    labels = ", ".join(dict.fromkeys(run.candidate.label for run in plan if run.stage == "gate")) or "none"
    print(
        f"P6.1 batch: {len(plan)} runs "
        f"({gate_runs} gate, {disturbance_runs} disturbed), duration={args.duration:g}s"
    )
    print(f"Plan={args.plan}, order={args.order}, candidates in gate order: {labels}")
    for i, run in enumerate(plan, start=1):
        print(
            f"{i:02d}. {run.stage} {run.candidate.label} r{run.repeat:02d}: "
            f"hipKo={run.candidate.hip_ko_q:g}/{run.candidate.hip_ko_dq:g}, "
            f"kneeKo={run.candidate.knee_ko_q:g}/{run.candidate.knee_ko_dq:g}, "
            f"disturbed={run.disturbed}"
        )

    if not args.execute:
        print("\nDry-run only. Add --execute to run the real robot batch.")
        return

    if args.yes:
        return

    print("\nThis will command the real robot.")
    print("Gate runs are no-disturbance; disturbed runs add the configured software torque pulse.")
    print("Ensure suspension/limits, E-stop, and a watcher are ready.")
    answer = input("Type RUN_P61 to start the batch: ").strip()
    if answer != "RUN_P61":
        raise SystemExit("confirmation failed; batch not started")


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = args.out_dir / f"p61_{args.plan}_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "stage",
            "candidate",
            "role",
            "hip_ko_q",
            "hip_ko_dq",
            "knee_ko_q",
            "knee_ko_dq",
            "ku_q",
            "ku_dq",
            "disturbed",
            "disturbance_joints",
            "disturbance_torques",
            "repeat",
            "condition",
            "config",
            "duration_s",
            "pause_s",
            "command",
            "start_time",
            "end_time",
            "returncode",
            "status",
            "log_path",
        ],
    )
    writer.writeheader()
    return path, writer, fh


def run_batch(args: argparse.Namespace, plan: list[RunSpec]) -> Path | None:
    if not args.execute:
        for run in plan:
            print(shell_join(command_for(args, run)))
        return None

    ensure_sudo(args)
    manifest_path, writer, fh = open_manifest(args)
    batch_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        for run_no, run in enumerate(plan, start=1):
            cmd = command_for(args, run)
            condition = condition_for(run)
            print(f"\n[{run_no}/{len(plan)}] {run.stage} {run.candidate.label} r{run.repeat:02d} {condition}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            # h1_direct waits for Enter after printing the safety warning.
            proc = subprocess.run(
                cmd,
                input="\n",
                text=True,
                cwd=REPO_ROOT,
            )
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = recent_completed_log(args, run, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print(
                    "Warning: h1_direct returned -6 after a complete log was written; "
                    f"continuing. log={completed_log}"
                )
            writer.writerow(
                {
                    "batch_time": batch_time,
                    "stage": run.stage,
                    "candidate": run.candidate.label,
                    "role": run.candidate.role,
                    "hip_ko_q": run.candidate.hip_ko_q,
                    "hip_ko_dq": run.candidate.hip_ko_dq,
                    "knee_ko_q": run.candidate.knee_ko_q,
                    "knee_ko_dq": run.candidate.knee_ko_dq,
                    "ku_q": run.candidate.ku_q,
                    "ku_dq": run.candidate.ku_dq,
                    "disturbed": int(run.disturbed),
                    "disturbance_joints": args.disturbance_joints if run.disturbed else "",
                    "disturbance_torques": args.disturbance_torques if run.disturbed else "",
                    "repeat": f"r{run.repeat:02d}",
                    "condition": condition,
                    "config": str(run.candidate.config.relative_to(REPO_ROOT)),
                    "duration_s": args.duration,
                    "pause_s": args.pause,
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
            if run_no < len(plan) and args.pause > 0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    candidates = selected_candidates(args)
    validate_args(args, candidates)
    plan = build_plan(args, candidates)
    confirm_or_exit(args, plan)
    manifest_path = run_batch(args, plan)
    if manifest_path:
        print(f"\nBatch complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
