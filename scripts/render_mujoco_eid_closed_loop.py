#!/usr/bin/env python3
"""Run the deployable C++ EID controller against MuJoCo and render video.

This script keeps the real Unitree SDK path untouched. It talks to the small
`h1_eid_stepper` executable over stdin/stdout; that executable uses the same
header-only `EidMultiJointController` as `h1_direct`.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from fit_mujoco_eid_params import DISPLAY_NAMES, gather_joint_info
from render_mujoco_joint_video import configure_model_for_visibility, make_camera, write_gif


JOINT_HEADER_RE = re.compile(r"^  ([0-9]+):\s*(?:#.*)?$")
KEY_VALUE_RE = re.compile(r"^\s+([A-Za-z0-9_]+):\s*([^#]*?)(?:\s*#.*)?$")
SAFETY_LOWSTATE_TIMEOUT = 1 << 0
SAFETY_NONFINITE_COMMAND = 1 << 1
SAFETY_COMMAND_SATURATED = 1 << 2
SAFETY_INVALID_STATE = 1 << 3
FATAL_SAFETY_FLAGS = SAFETY_LOWSTATE_TIMEOUT | SAFETY_NONFINITE_COMMAND | SAFETY_INVALID_STATE


@dataclass(frozen=True)
class JointReference:
    signal: str
    center: float
    amplitude: float
    frequency: float
    phase: float
    step_time: float


def initial_reference_value(ref: JointReference) -> float:
    if ref.signal == "step":
        return ref.center if ref.step_time > 0.0 else ref.center + ref.amplitude
    return ref.center + ref.amplitude * math.sin(ref.phase)


def default_stepper_path() -> Path:
    candidates = [
        Path("build-h1/Debug/h1_eid_stepper.exe"),
        Path("build/Debug/h1_eid_stepper.exe"),
        Path("build-h1/h1_eid_stepper"),
        Path("build/h1_eid_stepper"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return candidates[0]


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def load_controller_references(config: Path) -> dict[int, JointReference]:
    section: str | None = None
    current_joint: int | None = None
    defaults: dict[str, str] = {}
    values: dict[int, dict[str, str]] = {}

    for raw_line in config.read_text(encoding="utf-8").splitlines():
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if not raw_line.startswith(" "):
            section = stripped.rstrip(":") if stripped.endswith(":") else None
            current_joint = None
            continue

        if section == "eid_defaults":
            match = KEY_VALUE_RE.match(raw_line)
            if match and match.group(1) in {
                "reference_signal",
                "ref_center",
                "ref_amplitude",
                "ref_frequency",
                "ref_phase",
                "ref_step_time",
            }:
                defaults[match.group(1)] = match.group(2).strip()
            continue

        if section != "eid_controllers":
            continue
        header = JOINT_HEADER_RE.match(raw_line)
        if header:
            current_joint = int(header.group(1))
            values.setdefault(current_joint, {})
            continue
        if current_joint is None:
            continue
        match = KEY_VALUE_RE.match(raw_line)
        if match and match.group(1) in {
            "reference_signal",
            "ref_center",
            "ref_amplitude",
            "ref_frequency",
            "ref_phase",
            "ref_step_time",
        }:
            values[current_joint][match.group(1)] = match.group(2).strip()

    refs: dict[int, JointReference] = {}
    for joint_id in sorted(values):
        item = {**defaults, **values[joint_id]}
        required = {"ref_center", "ref_amplitude", "ref_frequency", "ref_phase"}
        missing = sorted(required - set(item))
        if missing:
            raise RuntimeError(f"{config}: eid_controllers.{joint_id} missing {missing}")
        signal = item.get("reference_signal", "sine").strip().lower()
        if signal not in {"sine", "step"}:
            raise RuntimeError(f"{config}: eid_controllers.{joint_id}.reference_signal must be sine or step")
        refs[joint_id] = JointReference(
            signal=signal,
            center=float(item["ref_center"]),
            amplitude=float(item["ref_amplitude"]),
            frequency=float(item["ref_frequency"]),
            phase=float(item["ref_phase"]),
            step_time=float(item.get("ref_step_time", "1.0")),
        )
    if not refs:
        raise RuntimeError(f"{config}: no eid_controllers found")
    return refs


def initial_qpos(model: mujoco.MjModel, info: dict, refs: dict[int, JointReference], height: float) -> np.ndarray:
    if model.nkey > 0:
        qpos = np.array(model.key_qpos[0], dtype=float)
    else:
        qpos = np.array(model.qpos0, dtype=float)
    qpos[0:3] = [0.0, 0.0, height]
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for joint_id, ref in refs.items():
        joint_info = info[joint_id]
        qpos[joint_info.qposadr] = clamp(
            initial_reference_value(ref),
            joint_info.q_min,
            joint_info.q_max,
        )
    return qpos


def fix_suspended_base(data: mujoco.MjData, height: float) -> None:
    data.qpos[0:3] = [0.0, 0.0, height]
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[0:6] = 0.0


def start_stepper(stepper: Path, config: Path) -> subprocess.Popen:
    if not stepper.exists():
        raise FileNotFoundError(
            f"{stepper} not found. Build it first, e.g. "
            "`cmake --build build-h1 --config Debug --target h1_eid_stepper`."
        )
    proc = subprocess.Popen(
        [str(stepper), str(config)],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )
    assert proc.stdout is not None
    ready = proc.stdout.readline().strip()
    if not ready.startswith("ready"):
        stderr = proc.stderr.read() if proc.stderr is not None else ""
        proc.kill()
        raise RuntimeError(f"stepper did not become ready: {ready}\n{stderr}")
    return proc


def send_state(proc: subprocess.Popen, cycle: int, t: float, dt: float, data: mujoco.MjData, info: dict) -> dict:
    assert proc.stdin is not None
    assert proc.stdout is not None
    fields = ["state", str(cycle), f"{t:.12g}", f"{dt:.12g}", "0.0"]
    for motor_id in range(20):
        if motor_id in info:
            joint_info = info[motor_id]
            q = float(data.qpos[joint_info.qposadr])
            dq = float(data.qvel[joint_info.dofadr])
            tau_est = float(data.actuator_force[motor_id]) if motor_id < len(data.actuator_force) else 0.0
        else:
            q = dq = tau_est = 0.0
        fields.extend([f"{q:.17g}", f"{dq:.17g}", f"{tau_est:.17g}"])
    proc.stdin.write(" ".join(fields) + "\n")
    proc.stdin.flush()

    line = proc.stdout.readline().strip()
    if not line.startswith("cmd "):
        raise RuntimeError(f"unexpected stepper response: {line}")
    parts = line.split()
    values = [float(x) for x in parts[1:]]
    if len(values) != 1 + 20 * 6:
        raise RuntimeError(f"unexpected command field count: {len(values)}")
    offset = 1
    return {
        "flags": int(values[0]),
        "tau": values[offset : offset + 20],
        "kp": values[offset + 20 : offset + 40],
        "kd": values[offset + 40 : offset + 60],
        "q_ref": values[offset + 60 : offset + 80],
        "tau_debug": values[offset + 80 : offset + 100],
        "joint_flags": [int(v) for v in values[offset + 100 : offset + 120]],
    }


def annotate_frame(
    image: np.ndarray,
    t: float,
    q_rmse: float,
    flags: int,
    active_joints: list[int],
    config_name: str,
) -> Image.Image:
    frame = Image.fromarray(image)
    draw = ImageDraw.Draw(frame, "RGBA")
    w, h = frame.size
    for x in (int(0.47 * w), int(0.53 * w)):
        draw.line([(x, 0), (x, int(0.28 * h))], fill=(255, 220, 80, 210), width=3)
    draw.rectangle([(12, 12), (560, 86)], fill=(0, 0, 0, 130))
    active_label = ",".join(str(j) for j in active_joints)
    if len(active_label) > 38:
        active_label = f"{len(active_joints)} joints ({active_joints[0]}..{active_joints[-1]}, no 9)"
    draw.text((22, 22), f"C++ EidMultiJointController -> MuJoCo  t={t:4.2f}s", fill=(255, 255, 255, 255))
    draw.text((22, 42), f"actual q tracking, q RMSE={q_rmse:.4f}, flags={flags}", fill=(230, 240, 255, 255))
    draw.text((22, 62), f"{config_name}  active={active_label}", fill=(230, 240, 255, 255))
    return frame


def try_write_mp4(frames_dir: Path, out_path: Path, fps: int) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return False
    cmd = [
        ffmpeg,
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(frames_dir / "frame_%04d.png"),
        "-pix_fmt",
        "yuv420p",
        "-vcodec",
        "libx264",
        str(out_path),
    ]
    completed = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return completed.returncode == 0 and out_path.exists() and out_path.stat().st_size > 0


def render_closed_loop(args: argparse.Namespace) -> tuple[Path, str, Path]:
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    configure_model_for_visibility(model)
    info = gather_joint_info(model)
    refs = load_controller_references(args.config)
    active_joints = sorted(refs)
    data = mujoco.MjData(model)
    data.qpos[:] = initial_qpos(model, info, refs, args.height_m)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    camera = make_camera()
    frames_dir = args.out_dir / "eid_closed_loop_frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    for frame in frames_dir.glob("frame_*.png"):
        frame.unlink()

    csv_path = args.out_dir / "eid_mujoco_closed_loop_log.csv"
    proc = start_stepper(args.stepper, args.config)
    frame_paths: list[Path] = []
    q_sse = 0.0
    q_count = 0
    max_abs_tau = 0.0
    combined_flags = 0
    render_every = max(1, int(round(1.0 / (args.fps * args.dt))))
    steps = int(round(args.duration / args.dt))

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "cycle",
                    "t",
                    "joint_id",
                    "q",
                    "dq",
                    "q_ref",
                    "q_error",
                    "tau_cmd",
                    "flags",
                ],
            )
            writer.writeheader()
            for step in range(steps):
                t = step * args.dt
                fix_suspended_base(data, args.height_m)
                command = send_state(proc, step, t, args.dt, data, info)
                combined_flags |= command["flags"]
                data.ctrl[:] = 0.0
                for joint_id in active_joints:
                    tau = float(command["tau"][joint_id])
                    data.ctrl[joint_id] = tau
                    max_abs_tau = max(max_abs_tau, abs(tau))

                    joint_info = info[joint_id]
                    q = float(data.qpos[joint_info.qposadr])
                    dq = float(data.qvel[joint_info.dofadr])
                    q_ref = float(command["q_ref"][joint_id])
                    q_error = q_ref - q
                    q_sse += q_error * q_error
                    q_count += 1
                    combined_flags |= command["joint_flags"][joint_id]
                    if step % max(1, int(round(0.02 / args.dt))) == 0:
                        writer.writerow(
                            {
                                "cycle": step,
                                "t": t,
                                "joint_id": joint_id,
                                "q": q,
                                "dq": dq,
                                "q_ref": q_ref,
                                "q_error": q_error,
                                "tau_cmd": tau,
                                "flags": command["flags"] | command["joint_flags"][joint_id],
                            }
                        )

                mujoco.mj_step(model, data)
                fix_suspended_base(data, args.height_m)
                mujoco.mj_forward(model, data)

                if step % render_every == 0:
                    renderer.update_scene(data, camera=camera)
                    image = renderer.render()
                    q_rmse = math.sqrt(q_sse / q_count) if q_count else 0.0
                    frame = annotate_frame(image, t, q_rmse, combined_flags, active_joints, args.config.name)
                    frame_path = frames_dir / f"frame_{len(frame_paths):04d}.png"
                    frame.save(frame_path)
                    frame_paths.append(frame_path)
    finally:
        renderer.close()
        if proc.stdin is not None:
            try:
                proc.stdin.write("quit\n")
                proc.stdin.flush()
            except BrokenPipeError:
                pass
        proc.wait(timeout=5)

    mp4_path = args.out_dir / "eid_mujoco_closed_loop.mp4"
    gif_path = args.out_dir / "eid_mujoco_closed_loop.gif"
    if try_write_mp4(frames_dir, mp4_path, args.fps):
        video_path = mp4_path
        video_kind = "mp4"
    else:
        write_gif(frame_paths, gif_path, args.fps)
        video_path = gif_path
        video_kind = "gif"

    q_rmse = math.sqrt(q_sse / q_count) if q_count else 0.0
    fatal_flags = combined_flags & FATAL_SAFETY_FLAGS
    nonfatal_flags = combined_flags & ~FATAL_SAFETY_FLAGS
    manifest_path = args.out_dir / "eid_mujoco_closed_loop_manifest.txt"
    frames_saved = bool(args.keep_frames)
    manifest_path.write_text(
        "\n".join(
            [
                "C++ EidMultiJointController closed-loop MuJoCo visualization",
                f"scene={args.scene}",
                f"config={args.config}",
                f"stepper={args.stepper}",
                f"video={video_path}",
                f"video_kind={video_kind}",
                f"csv={csv_path}",
                f"duration={args.duration}",
                f"dt={args.dt}",
                f"fps={args.fps}",
                f"frames={len(frame_paths)}",
                f"frames_dir={frames_dir if frames_saved else ''}",
                f"frames_saved={str(frames_saved).lower()}",
                f"active_joints={','.join(str(j) for j in active_joints)}",
                f"q_rmse={q_rmse}",
                f"max_abs_tau={max_abs_tau}",
                f"combined_flags={combined_flags}",
                f"fatal_flags={fatal_flags}",
                f"nonfatal_flags={nonfatal_flags}",
                "note=Floating base is pinned in the air; joints are stepped by MuJoCo dynamics using controller torque commands.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    if not args.keep_frames:
        shutil.rmtree(frames_dir, ignore_errors=True)
    if fatal_flags != 0:
        raise RuntimeError(f"fatal controller/safety flags were set: {fatal_flags} (combined={combined_flags})")
    return video_path, video_kind, manifest_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--config", type=Path, default=Path("config/h1_full_body_mujoco_fit.yaml"))
    parser.add_argument("--stepper", type=Path, default=default_stepper_path())
    parser.add_argument("--out-dir", type=Path, default=Path("data/mujoco_fit/latest"))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--height-m", type=float, default=1.35)
    parser.add_argument("--keep-frames", action="store_true", help="Keep intermediate rendered PNG frames.")
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    video_path, video_kind, manifest_path = render_closed_loop(args)
    print(f"video={video_path}")
    print(f"video_kind={video_kind}")
    print(f"manifest={manifest_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
