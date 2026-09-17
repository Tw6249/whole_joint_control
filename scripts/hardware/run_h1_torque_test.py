#!/usr/bin/env python3
"""Interactive read-only H1 hip/knee torque logging workflow.

The script starts h1_torque_logger, records operator-entered phase markers, and
merges those markers into the final CSV. It never publishes robot commands.
"""

from __future__ import annotations

# Support direct execution as well as python -m from the project root.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional


DEFAULT_CONFIG = Path("experiments/hip_knee/configs/h1_real_p3_selected_mpc_hip_knee_pd.yaml")
DEFAULT_LOGGER = Path("build-h1/h1_torque_logger")
DEFAULT_JOINTS = "1,2,4,5"
STOP_WORDS = {"stop", "quit", "q", "停止", "停止采集", "结束", "结束采集"}


@dataclass
class Marker:
    wall_unix_ns: int
    wall_time_local: str
    text: str


def local_time_from_ns(unix_ns: int) -> str:
    seconds = unix_ns // 1_000_000_000
    nanos = unix_ns % 1_000_000_000
    return datetime.fromtimestamp(seconds).strftime("%Y-%m-%d %H:%M:%S") + f".{nanos:09d}"


def timestamp_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--logger", type=Path, default=DEFAULT_LOGGER)
    parser.add_argument("--out", type=Path, help="Final merged CSV path")
    parser.add_argument("--duration", type=float, default=180.0, help="Maximum recording time in seconds")
    parser.add_argument("--sample-period", type=float, default=0.01, help="Sampling period in seconds")
    parser.add_argument("--joints", default=DEFAULT_JOINTS, help="Comma-separated joint ids")
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--keep-raw", action="store_true", help="Keep the temporary raw CSV")
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.sample_period <= 0:
        raise ValueError("--sample-period must be positive")
    if not args.config.exists():
        raise FileNotFoundError(f"config not found: {args.config}")
    if not args.logger.exists():
        raise FileNotFoundError(
            f"logger not found: {args.logger}; build it with "
            "cmake --build build-h1 --target h1_torque_logger"
        )


def is_stop_command(text: str) -> bool:
    lowered = text.lower()
    return lowered in STOP_WORDS or "停止采集" in text or "结束采集" in text


def reader_thread(proc: subprocess.Popen[str]) -> None:
    assert proc.stdout is not None
    for line in proc.stdout:
        print("[logger]", line.rstrip())


def stop_logger(proc: subprocess.Popen[str]) -> int:
    if proc.poll() is not None:
        return proc.returncode
    proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            return proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            return proc.wait()


def merge_markers(raw_csv: Path, final_csv: Path, markers: List[Marker]) -> int:
    final_csv.parent.mkdir(parents=True, exist_ok=True)
    markers = sorted(markers, key=lambda m: m.wall_unix_ns)
    marker_index = -1
    next_event = 0
    rows = 0

    with raw_csv.open("r", newline="") as src, final_csv.open("w", newline="") as dst:
        reader = csv.DictReader(src)
        if not reader.fieldnames:
            raise RuntimeError(f"raw CSV is empty: {raw_csv}")

        fieldnames = list(reader.fieldnames)
        extra = ["operator_state", "operator_event", "operator_marker_wall_time"]
        for name in extra:
            if name not in fieldnames:
                fieldnames.append(name)

        writer = csv.DictWriter(dst, fieldnames=fieldnames)
        writer.writeheader()

        for row in reader:
            wall_ns = int(row["wall_unix_ns"])
            while marker_index + 1 < len(markers) and markers[marker_index + 1].wall_unix_ns <= wall_ns:
                marker_index += 1

            event = ""
            event_time = ""
            while next_event < len(markers) and markers[next_event].wall_unix_ns <= wall_ns:
                event = markers[next_event].text if not event else event + " | " + markers[next_event].text
                event_time = markers[next_event].wall_time_local
                next_event += 1

            if marker_index >= 0:
                row["operator_state"] = markers[marker_index].text
                row["operator_marker_wall_time"] = markers[marker_index].wall_time_local
            else:
                row["operator_state"] = ""
                row["operator_marker_wall_time"] = ""
            row["operator_event"] = event
            if event_time:
                row["operator_marker_wall_time"] = event_time
            writer.writerow(row)
            rows += 1

    return rows


def main() -> int:
    args = parse_args()
    validate_args(args)

    out = args.out or Path("data") / f"h1_native_hip_knee_torque_{timestamp_name()}.csv"
    raw = out.with_name(out.stem + "_raw.csv")

    cmd = [
        str(args.logger),
        str(args.config),
        "--duration",
        f"{args.duration:.6f}",
        "--sample-period",
        f"{args.sample_period:.6f}",
        "--joints",
        args.joints,
        "--iface",
        args.iface,
        "--domain",
        str(args.domain),
        "--out",
        str(raw),
    ]

    print("只读采集即将开始，不会给机器人下发运动命令。")
    print(f"采样周期: {args.sample_period:.3f} s")
    print(f"采集关节: {args.joints}  (默认 1=右髋, 2=右膝, 4=左髋, 5=左膝)")
    print(f"最终 CSV: {out}")
    print("启动后请输入状态，例如：吊挂、进入阻尼状态、进入锁定站立、开始落地、双脚完全承重、开始正常行走。")
    print("输入 stop 或 停止采集 结束。")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        cwd=Path.cwd(),
        preexec_fn=os.setsid,
    )
    thread = threading.Thread(target=reader_thread, args=(proc,), daemon=True)
    thread.start()

    markers: List[Marker] = []
    exit_code: Optional[int] = None
    try:
        while proc.poll() is None:
            try:
                text = input("状态> ").strip()
            except EOFError:
                text = "停止采集"
            now_ns = time.time_ns()
            if not text:
                continue
            if is_stop_command(text):
                markers.append(Marker(now_ns, local_time_from_ns(now_ns), text))
                exit_code = stop_logger(proc)
                break
            markers.append(Marker(now_ns, local_time_from_ns(now_ns), text))
            print(f"已记录: {text} @ {markers[-1].wall_time_local}")
        if exit_code is None:
            exit_code = proc.wait()
    except KeyboardInterrupt:
        now_ns = time.time_ns()
        markers.append(Marker(now_ns, local_time_from_ns(now_ns), "KeyboardInterrupt/停止采集"))
        exit_code = stop_logger(proc)

    thread.join(timeout=1.0)
    if not raw.exists():
        print(f"没有生成原始 CSV: {raw}", file=sys.stderr)
        return exit_code if exit_code else 2

    rows = merge_markers(raw, out, markers)
    if not args.keep_raw:
        raw.unlink(missing_ok=True)

    print(f"已合并 {rows} 行采样数据和 {len(markers)} 个状态标记。")
    print(f"最终 CSV: {out}")
    return 0 if exit_code in (0, 3, -signal.SIGINT) else int(exit_code or 0)


if __name__ == "__main__":
    raise SystemExit(main())
