"""Shared P1/P2 batch mechanics; experiment metadata stays in each entry point."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import statistics
import subprocess
import time
from pathlib import Path
from typing import Any, Callable


def selected_methods(args: argparse.Namespace) -> list[str]:
    return [args.methods] if args.methods in ("pd", "eid") else ["pd", "eid"]


def validate_args(args: argparse.Namespace, methods: dict[str, Any]) -> None:
    if args.repeats <= 0:
        raise SystemExit("--repeats must be positive")
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.pause < 0:
        raise SystemExit("--pause must be non-negative")
    if args.start_index <= 0:
        raise SystemExit("--start-index must be positive")
    if not args.direct.exists():
        raise SystemExit(f"h1_direct not found: {args.direct}")
    for spec in methods.values():
        if not spec.config.exists():
            raise SystemExit(f"config not found: {spec.config}")


def build_plan(args: argparse.Namespace, methods: dict[str, Any]) -> list[tuple[int, Any]]:
    indices = range(args.start_index, args.start_index + args.repeats)
    selected = selected_methods(args)
    order = ["eid", "pd"] if args.order == "eid-then-pd" else ["pd", "eid"]
    order = [method for method in order if method in selected]
    if args.order == "alternating":
        return [(repeat, methods[method]) for repeat in indices for method in order]
    return [(repeat, methods[method]) for method in order for repeat in indices]


def shell_join(cmd: list[str]) -> str:
    return " ".join(f"'{part}'" if any(c.isspace() for c in part) else part for part in cmd)


def config_log_name(spec: Any) -> str:
    for line in spec.config.read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("log_path:"):
            return Path(line.split(":", 1)[1].strip()).name
    raise RuntimeError(f"{spec.config} missing log_path")


def recent_completed_log(
    repo_root: Path,
    spec: Any,
    repeat: int,
    started_at: dt.datetime,
    expected_duration: float,
    expected_metadata: dict[str, str],
) -> Path | None:
    candidates = sorted(
        (repo_root / "data").glob(f"*/{config_log_name(spec)}"),
        key=lambda p: p.stat().st_mtime,
        reverse=True,
    )
    expected = {**expected_metadata, "repeat_id": f"r{repeat:02d}"}
    for path in candidates:
        if path.stat().st_mtime < started_at.timestamp() - 2.0:
            break
        try:
            with path.open(newline="", encoding="utf-8") as fh:
                first_cycle = last_cycle = None
                dts: list[float] = []
                metadata: dict[str, str] = {}
                for rows, row in enumerate(csv.DictReader(fh), start=1):
                    if rows == 1:
                        metadata = {key: row.get(key, "") for key in expected}
                    cycle = int(row["cycle"])
                    first_cycle = cycle if first_cycle is None else min(first_cycle, cycle)
                    last_cycle = cycle if last_cycle is None else max(last_cycle, cycle)
                    if len(dts) < 2000:
                        dts.append(float(row["dt"]))
        except Exception as exc:
            print(f"Warning: could not inspect {path}: {exc}")
            continue
        if first_cycle is None or last_cycle is None or metadata != expected:
            continue
        duration = max(0, last_cycle - first_cycle) * (statistics.median(dts) if dts else 0.0)
        if duration >= 0.90 * expected_duration:
            return path
    return None


def ensure_sudo(args: argparse.Namespace) -> None:
    if args.no_sudo or not args.execute:
        return
    print("Checking sudo credentials with `sudo -v`...")
    subprocess.run(["sudo", "-v"], check=True)


def run_batch(
    args: argparse.Namespace,
    plan: list[tuple[int, Any]],
    *,
    repo_root: Path,
    command_for: Callable,
    completed_log_for: Callable,
    open_manifest: Callable,
    metadata_for: Callable,
    display_suffix: Callable,
) -> Path | None:
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
            print(f"\n[{run_no}/{len(plan)}] {spec.method.upper()} r{repeat:02d}{display_suffix(args, spec)}")
            print(shell_join(cmd))
            start = dt.datetime.now()
            # h1_direct waits for Enter after printing its safety warning.
            proc = subprocess.run(cmd, input="\n", text=True, cwd=repo_root)
            end = dt.datetime.now()
            status = "ok" if proc.returncode == 0 else "failed"
            completed_log = completed_log_for(args, spec, repeat, start, args.duration)
            if proc.returncode == -6 and completed_log is not None:
                status = "accepted_after_complete_abort"
                print("Warning: h1_direct returned -6 after a complete log was written; "
                      f"continuing. log={completed_log}")
            writer.writerow({
                "batch_time": batch_time,
                **metadata_for(args, spec),
                "method": spec.method,
                "repeat": f"r{repeat:02d}",
                "config": str(spec.config.relative_to(repo_root)),
                "duration_s": args.duration,
                "pause_s": args.pause,
                "command": shell_join(cmd),
                "start_time": start.isoformat(timespec="seconds"),
                "end_time": end.isoformat(timespec="seconds"),
                "returncode": proc.returncode,
                "status": status,
                "log_path": str(completed_log.relative_to(repo_root)) if completed_log else "",
            })
            fh.flush()
            if proc.returncode != 0 and status != "accepted_after_complete_abort":
                raise SystemExit(f"run failed with return code {proc.returncode}; manifest={manifest_path}")
            if run_no < len(plan) and args.pause > 0:
                print(f"Pausing {args.pause:g}s before next run...")
                time.sleep(args.pause)
    finally:
        fh.close()
    return manifest_path
