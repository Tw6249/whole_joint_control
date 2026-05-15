#!/usr/bin/env python3
"""Run the full-body MuJoCo EID test and plotting workflow.

This script never rewrites the controller YAML. Edit per-joint references in
the YAML, pass it with --config, and the same file is used for render + plots.
"""

from __future__ import annotations

import argparse
from datetime import datetime
import shutil
import subprocess
import sys
from pathlib import Path


def run_command(cmd: list[str]) -> subprocess.CompletedProcess:
    print("+ " + " ".join(cmd), flush=True)
    completed = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(completed.stdout, end="")
    return completed


def render(config: Path, out_dir: Path, duration: float, dt: float, fps: int, keep_frames: bool) -> subprocess.CompletedProcess:
    cmd = [
        sys.executable,
        "scripts/render_mujoco_eid_closed_loop.py",
        "--config",
        str(config),
        "--out-dir",
        str(out_dir),
        "--duration",
        str(duration),
        "--dt",
        str(dt),
        "--fps",
        str(fps),
    ]
    if keep_frames:
        cmd.append("--keep-frames")
    return run_command(cmd)


def plot(config: Path, out_dir: Path) -> None:
    completed = run_command(
        [
            sys.executable,
            "scripts/plot_mujoco_lower_body_results.py",
            "--config",
            str(config),
            "--log",
            str(out_dir / "eid_mujoco_closed_loop_log.csv"),
            "--out-dir",
            str(out_dir),
        ]
    )
    if completed.returncode != 0:
        raise RuntimeError(f"plotting failed with exit code {completed.returncode}")


def safe_stem(text: str) -> str:
    return "".join(ch if ch.isalnum() or ch in {"-", "_"} else "_" for ch in text).strip("_")


def make_run_dir(runs_root: Path, config: Path, run_name: str | None) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = safe_stem(run_name) if run_name else f"{stamp}_{safe_stem(config.stem)}"
    candidate = runs_root / name
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = runs_root / f"{name}_{suffix:02d}"
    return candidate


def write_text(path: Path, lines: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_run_manifest(path: Path, args: argparse.Namespace, config_copy: Path) -> None:
    write_text(
        path,
        [
            "H1 full-body MuJoCo EID run",
            f"created_at={datetime.now().isoformat(timespec='seconds')}",
            f"config={args.config}",
            f"config_copy={config_copy}",
            f"out_dir={args.out_dir}",
            f"duration={args.duration}",
            f"dt={args.dt}",
            f"fps={args.fps}",
            f"keep_frames={args.keep_frames}",
            f"command={' '.join(sys.argv)}",
            "outputs=eid_mujoco_closed_loop.gif/eid_mujoco_closed_loop.mp4,eid_mujoco_closed_loop_log.csv,h1_eid_summary.csv,h1_eid_tracking_grid.png,h1_eid_error_grid.png,h1_eid_torque_grid.png",
        ],
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--out-dir", type=Path, default=None, help="Result directory. Defaults to a timestamped directory under --runs-root.")
    parser.add_argument("--runs-root", type=Path, default=Path("data/mujoco_fit/runs"))
    parser.add_argument("--run-name", default=None, help="Optional directory name under --runs-root when --out-dir is not set.")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--keep-frames", action="store_true", help="Keep intermediate rendered PNG frames. By default they are deleted after video creation.")
    args = parser.parse_args()

    if not args.config.exists():
        raise FileNotFoundError(f"config not found: {args.config}")
    if args.out_dir is None:
        args.out_dir = make_run_dir(args.runs_root, args.config, args.run_name)
    args.out_dir.mkdir(parents=True, exist_ok=True)

    config_copy = args.out_dir / "input_config.yaml"
    shutil.copy2(args.config, config_copy)
    write_run_manifest(args.out_dir / "run_manifest.txt", args, config_copy)

    rendered = render(args.config, args.out_dir, args.duration, args.dt, args.fps, args.keep_frames)
    if rendered.returncode != 0:
        return rendered.returncode

    plot(args.config, args.out_dir)
    latest_path = Path("data/mujoco_fit/LATEST.txt")
    write_text(latest_path, [str(args.out_dir)])
    print(f"config={args.config}")
    print(f"result_dir={args.out_dir}")
    print(f"latest={latest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
