#!/usr/bin/env python3
"""Run the real P1 hip-knee experiment batch.

The script is intentionally conservative:
  - dry-run by default;
  - explicit --execute required for real robot motion;
  - confirmation required unless --yes is passed;
  - sudo credentials are checked once before the batch.
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
import sys
from dataclasses import dataclass
from pathlib import Path

from experiments.hip_knee import batch_common as common
from experiments.hip_knee.batch_common import config_log_name, ensure_sudo, selected_methods, shell_join


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DIRECT = REPO_ROOT / "build-h1" / "h1_direct"
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_p1_batch"


@dataclass(frozen=True)
class MethodSpec:
    method: str
    config: Path
    condition: str


METHODS = {
    "pd": MethodSpec(
        method="pd",
        config=REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p1_lite_hip_knee_pd.yaml",
        condition="P1_PD_same_phase_no_disturbance",
    ),
    "eid": MethodSpec(
        method="eid",
        config=REPO_ROOT / "experiments" / "hip_knee" / "configs" / "h1_real_p1_lite_hip_knee_eid.yaml",
        condition="P1_EID_same_phase_no_disturbance",
    ),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run P1 real H1 PD/EID experiments, N repeats per method."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run h1_direct. Default is dry-run.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_P1 confirmation prompt.")
    parser.add_argument("--no-sudo", action="store_true", help="Run h1_direct directly instead of through sudo.")
    parser.add_argument("--direct", type=Path, default=DEFAULT_DIRECT, help="Path to h1_direct executable.")
    parser.add_argument("--repeats", type=int, default=5, help="Repeats per method.")
    parser.add_argument("--duration", type=float, default=8.0, help="Duration per run in seconds.")
    parser.add_argument("--pause", type=float, default=5.0, help="Pause between runs in seconds.")
    parser.add_argument("--start-index", type=int, default=1, help="Repeat index for the first run.")
    parser.add_argument(
        "--order",
        choices=["alternating", "pd-then-eid", "eid-then-pd"],
        default="alternating",
        help="Batch order. Alternating runs PD then EID for each repeat.",
    )
    parser.add_argument(
        "--methods",
        choices=["both", "pd", "eid"],
        default="both",
        help="Subset of methods to run.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Manifest output directory.")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    common.validate_args(args, METHODS)


def build_plan(args: argparse.Namespace) -> list[tuple[int, MethodSpec]]:
    return common.build_plan(args, METHODS)


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


def recent_completed_log(spec: MethodSpec,
                         repeat: int,
                         started_at: dt.datetime,
                         expected_duration: float) -> Path | None:
    return common.recent_completed_log(
        REPO_ROOT, spec, repeat, started_at, expected_duration,
        {"condition_id": spec.condition},
    )


def confirm_or_exit(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> None:
    print(f"P1 batch plan: {len(plan)} runs, duration={args.duration:g}s, pause={args.pause:g}s")
    for i, (repeat, spec) in enumerate(plan, start=1):
        print(f"{i:02d}. {spec.method.upper()} r{repeat:02d}: {spec.config.relative_to(REPO_ROOT)}")

    if not args.execute:
        print("\nDry-run only. Add --execute to run the real robot batch.")
        return

    if args.yes:
        return

    print("\nThis will command the real robot. Ensure suspension/limits, E-stop, and a watcher are ready.")
    answer = input("Type RUN_P1 to start the batch: ").strip()
    if answer != "RUN_P1":
        raise SystemExit("confirmation failed; batch not started")


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = args.out_dir / f"p1_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "method",
            "repeat",
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


def run_batch(args: argparse.Namespace, plan: list[tuple[int, MethodSpec]]) -> Path | None:
    return common.run_batch(
        args, plan, repo_root=REPO_ROOT, command_for=command_for,
        completed_log_for=lambda args, spec, repeat, start, duration: recent_completed_log(
            spec, repeat, start, duration),
        open_manifest=open_manifest, metadata_for=lambda args, spec: {},
        display_suffix=lambda args, spec: "",
    )


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
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
