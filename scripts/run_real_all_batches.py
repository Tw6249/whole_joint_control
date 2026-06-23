#!/usr/bin/env python3
"""Run the real P1-P4 experiment batches in sequence.

This is an orchestration wrapper around the individual batch scripts:
  - dry-run by default;
  - explicit --execute required for real robot motion;
  - one RUN_ALL confirmation before launching child scripts;
  - each child script still writes its own detailed manifest.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT_DIR = REPO_ROOT / "analysis_artifacts" / "real_all_batch"


@dataclass(frozen=True)
class BatchSpec:
    name: str
    script: Path
    args: list[str]
    description: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run P1, P2, P3, and P4 real H1 batch experiments in sequence."
    )
    parser.add_argument("--execute", action="store_true", help="Actually run child batch scripts.")
    parser.add_argument("--yes", action="store_true", help="Skip the RUN_ALL confirmation prompt.")
    parser.add_argument(
        "--only",
        default="",
        help="Comma-separated subset to run, e.g. p1,p2 or p4. Default: p1,p2,p3,p4.",
    )
    parser.add_argument(
        "--skip",
        default="",
        help="Comma-separated batches to skip, e.g. p3,p4.",
    )
    parser.add_argument(
        "--pause-between-batches",
        type=float,
        default=10.0,
        help="Pause between child batch scripts in seconds.",
    )
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR, help="Top-level manifest directory.")
    parser.add_argument(
        "--p2-target",
        choices=["hip", "knee"],
        default="hip",
        help="Target passed to run_real_p2_batch.py.",
    )
    parser.add_argument(
        "--p2-disturbance-method",
        default="manual_push",
        help="Disturbance method metadata passed to run_real_p2_batch.py.",
    )
    parser.add_argument(
        "--p4-scan",
        choices=["ko", "ku", "both"],
        default="ko",
        help="Scan passed to run_real_p4_batch.py.",
    )
    parser.add_argument(
        "--p4-groups",
        default="",
        help="Optional P4 group subset, e.g. o5,u4. Overrides --p4-scan inside P4 script.",
    )
    parser.add_argument(
        "--p4-target",
        choices=["none", "hip", "knee"],
        default="none",
        help="Target passed to run_real_p4_batch.py.",
    )
    parser.add_argument(
        "--p4-disturbance-method",
        default="manual_push",
        help="Disturbance method metadata passed to run_real_p4_batch.py.",
    )
    return parser.parse_args()


def parse_batch_list(value: str) -> set[str]:
    if not value.strip():
        return set()
    valid = {"p1", "p2", "p3", "p4"}
    result = {part.strip().lower() for part in value.split(",") if part.strip()}
    unknown = result - valid
    if unknown:
        raise SystemExit(f"unknown batch name(s): {', '.join(sorted(unknown))}")
    return result


def build_batches(args: argparse.Namespace) -> list[BatchSpec]:
    p4_args = [
        "--scan",
        args.p4_scan,
        "--target",
        args.p4_target,
        "--disturbance-method",
        args.p4_disturbance_method,
    ]
    if args.p4_groups.strip():
        p4_args.extend(["--groups", args.p4_groups])

    batches = [
        BatchSpec(
            name="p1",
            script=REPO_ROOT / "scripts" / "run_real_p1_batch.py",
            args=[],
            description="P1 PD/EID no-disturbance baseline",
        ),
        BatchSpec(
            name="p2",
            script=REPO_ROOT / "scripts" / "run_real_p2_batch.py",
            args=[
                "--target",
                args.p2_target,
                "--disturbance-method",
                args.p2_disturbance_method,
            ],
            description="P2 PD/EID disturbance comparison",
        ),
        BatchSpec(
            name="p3",
            script=REPO_ROOT / "scripts" / "run_real_p3_batch.py",
            args=[],
            description="P3 quintic vs Preview-MPC reference comparison",
        ),
        BatchSpec(
            name="p4",
            script=REPO_ROOT / "scripts" / "run_real_p4_batch.py",
            args=p4_args,
            description="P4 EID Ko/Ku parameter scan",
        ),
    ]

    only = parse_batch_list(args.only)
    skip = parse_batch_list(args.skip)
    if only:
        batches = [batch for batch in batches if batch.name in only]
    if skip:
        batches = [batch for batch in batches if batch.name not in skip]
    if not batches:
        raise SystemExit("no batches selected")
    for batch in batches:
        if not batch.script.exists():
            raise SystemExit(f"batch script not found: {batch.script}")
    return batches


def command_for(args: argparse.Namespace, batch: BatchSpec) -> list[str]:
    cmd = ["python3", str(batch.script.relative_to(REPO_ROOT)), *batch.args]
    if args.execute:
        cmd.extend(["--execute", "--yes"])
    return cmd


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def print_plan(args: argparse.Namespace, batches: list[BatchSpec]) -> None:
    mode = "EXECUTE" if args.execute else "DRY-RUN"
    print(f"All-batch plan ({mode}): {len(batches)} child batch script(s)")
    for index, batch in enumerate(batches, start=1):
        print(f"{index:02d}. {batch.name.upper()}: {batch.description}")
        print(f"    {shell_join(command_for(args, batch))}")
    if not args.execute:
        print("\nDry-run only. Add --execute to run child batch scripts.")


def confirm_or_exit(args: argparse.Namespace) -> None:
    if not args.execute or args.yes:
        return
    print("\nThis will command the real robot across multiple experiment batches.")
    print("Ensure suspension/limits, E-stop, a watcher, and cooling/rest windows are ready.")
    answer = input("Type RUN_ALL to start the full batch sequence: ").strip()
    if answer != "RUN_ALL":
        raise SystemExit("confirmation failed; full batch sequence not started")


def open_manifest(args: argparse.Namespace) -> tuple[Path, csv.DictWriter, object]:
    args.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = args.out_dir / f"all_batch_manifest_{stamp}.csv"
    fh = path.open("w", newline="", encoding="utf-8")
    writer = csv.DictWriter(
        fh,
        fieldnames=[
            "batch_time",
            "batch",
            "description",
            "command",
            "start_time",
            "end_time",
            "returncode",
            "status",
        ],
    )
    writer.writeheader()
    return path, writer, fh


def run_batches(args: argparse.Namespace, batches: list[BatchSpec]) -> Path | None:
    if not args.execute:
        return None

    manifest_path, writer, fh = open_manifest(args)
    batch_time = dt.datetime.now().isoformat(timespec="seconds")
    try:
        for index, batch in enumerate(batches, start=1):
            cmd = command_for(args, batch)
            print(f"\n[{index}/{len(batches)}] Running {batch.name.upper()}: {batch.description}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            proc = subprocess.run(cmd, cwd=REPO_ROOT)
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            writer.writerow(
                {
                    "batch_time": batch_time,
                    "batch": batch.name,
                    "description": batch.description,
                    "command": shell_join(cmd),
                    "start_time": start.isoformat(timespec="seconds"),
                    "end_time": end.isoformat(timespec="seconds"),
                    "returncode": proc.returncode,
                    "status": status,
                }
            )
            fh.flush()
            if proc.returncode != 0:
                raise SystemExit(f"{batch.name.upper()} failed with return code {proc.returncode}; manifest={manifest_path}")
            if index < len(batches) and args.pause_between_batches > 0:
                print(f"Pausing {args.pause_between_batches:g}s before next batch...")
                time.sleep(args.pause_between_batches)
    finally:
        fh.close()
    return manifest_path


def main() -> None:
    args = parse_args()
    if args.pause_between_batches < 0:
        raise SystemExit("--pause-between-batches must be non-negative")
    batches = build_batches(args)
    print_plan(args, batches)
    confirm_or_exit(args)
    manifest_path = run_batches(args, batches)
    if manifest_path:
        print(f"\nAll selected batches complete. Manifest: {manifest_path}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user.", file=sys.stderr)
        raise SystemExit(130)
