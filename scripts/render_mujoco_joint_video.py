#!/usr/bin/env python3
"""Render a suspended H1 full-body multi-joint control visualization.

The script kinematically applies the same style of joint references used by the
EID controller and renders the official MuJoCo model. It does not run contact
physics or replace the controller; it is a visual sanity check for joint mapping,
signs, ranges, and simultaneous motion.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from fit_mujoco_eid_params import (
    ACTIVE_JOINTS,
    DISPLAY_NAMES,
    REFERENCE_DEFAULTS,
    base_qpos,
    gather_joint_info,
)


VIDEO_AMPLITUDE_SCALE = {
    0: 3.0,
    1: 3.0,
    2: 2.5,
    3: 3.0,
    4: 3.0,
    5: 2.5,
    6: 3.0,
    7: 3.0,
    8: 3.0,
    10: 3.0,
    11: 3.0,
    12: 2.0,
    13: 2.0,
    14: 2.0,
    15: 2.0,
    16: 2.0,
    17: 2.0,
    18: 2.0,
    19: 2.0,
}


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def configure_model_for_visibility(model: mujoco.MjModel) -> None:
    black_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "black")
    if black_id >= 0:
        model.mat_rgba[black_id] = [0.62, 0.64, 0.67, 1.0]
    white_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MATERIAL, "white")
    if white_id >= 0:
        model.mat_rgba[white_id] = [0.95, 0.95, 0.9, 1.0]


def suspended_qpos(model: mujoco.MjModel, height: float) -> np.ndarray:
    qpos = base_qpos(model)
    qpos[0] = 0.0
    qpos[1] = 0.0
    qpos[2] = height
    qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    return qpos


def apply_joint_references(model: mujoco.MjModel, qpos: np.ndarray, info: dict, t: float) -> None:
    for joint_id in ACTIVE_JOINTS:
        center, amplitude, frequency, phase = REFERENCE_DEFAULTS[joint_id]
        amplitude *= VIDEO_AMPLITUDE_SCALE.get(joint_id, 1.0)
        # Slight phase offsets keep simultaneous motion visually readable.
        phase += 0.35 * joint_id
        q = center + amplitude * math.sin(2.0 * math.pi * frequency * t + phase)
        joint_info = info[joint_id]
        margin = 0.03 * (joint_info.q_max - joint_info.q_min)
        qpos[joint_info.qposadr] = clamp(q, joint_info.q_min + margin, joint_info.q_max - margin)


def make_camera() -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [0.0, 0.0, 0.85]
    camera.distance = 3.0
    camera.azimuth = 135.0
    camera.elevation = -12.0
    return camera


def annotate_frame(image: np.ndarray, t: float) -> Image.Image:
    frame = Image.fromarray(image)
    draw = ImageDraw.Draw(frame, "RGBA")
    w, h = frame.size
    # Simple visual harness overlay. It is intentionally an annotation, not a
    # simulated constraint.
    for x in (int(0.47 * w), int(0.53 * w)):
        draw.line([(x, 0), (x, int(0.28 * h))], fill=(255, 220, 80, 210), width=3)
    draw.rectangle([(12, 12), (330, 58)], fill=(0, 0, 0, 120))
    draw.text((22, 22), f"Suspended H1 full-body EID refs  t={t:4.2f}s", fill=(255, 255, 255, 255))
    return frame


def render_frames(args: argparse.Namespace) -> list[Path]:
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    configure_model_for_visibility(model)
    data = mujoco.MjData(model)
    info = gather_joint_info(model)
    camera = make_camera()
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    frames_dir = args.out_dir / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)
    for old_frame in frames_dir.glob("frame_*.png"):
        old_frame.unlink()

    frame_paths: list[Path] = []
    n_frames = int(round(args.duration * args.fps))
    for frame_idx in range(n_frames):
        t = frame_idx / args.fps
        qpos = suspended_qpos(model, args.height_m)
        apply_joint_references(model, qpos, info, t)
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        image = renderer.render()
        frame = annotate_frame(image, t)
        frame_path = frames_dir / f"frame_{frame_idx:04d}.png"
        frame.save(frame_path)
        frame_paths.append(frame_path)

    renderer.close()
    return frame_paths


def write_gif(frame_paths: list[Path], out_path: Path, fps: int) -> None:
    frames = [Image.open(path).convert("P", palette=Image.Palette.ADAPTIVE) for path in frame_paths]
    duration_ms = int(round(1000 / fps))
    frames[0].save(
        out_path,
        save_all=True,
        append_images=frames[1:],
        duration=duration_ms,
        loop=0,
        optimize=False,
    )
    for frame in frames:
        frame.close()


def try_write_mp4(frame_paths: list[Path], out_path: Path, fps: int) -> bool:
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        return False
    frames_dir = frame_paths[0].parent
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


def write_manifest(args: argparse.Namespace, frame_paths: list[Path], video_path: Path, video_kind: str) -> None:
    manifest = args.out_dir / "suspended_h1_full_body_manifest.txt"
    joints = ", ".join(f"{j}:{DISPLAY_NAMES[j]}" for j in ACTIVE_JOINTS)
    manifest.write_text(
        "\n".join(
            [
                "Suspended H1 full-body multi-joint visualization",
                f"scene={args.scene}",
                f"video={video_path}",
                f"video_kind={video_kind}",
                f"frames={len(frame_paths)}",
                f"fps={args.fps}",
                f"duration={args.duration}",
                f"height_m={args.height_m}",
                f"active_joints={joints}",
                "note=Kinematic visualization of reference motion; not a physics/contact simulation.",
            ]
        )
        + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=Path("h1_official_mujoco/scene.xml"))
    parser.add_argument("--out-dir", type=Path, default=Path("data/mujoco_fit/latest"))
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--height-m", type=float, default=1.35)
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    frame_paths = render_frames(args)
    mp4_path = args.out_dir / "suspended_h1_full_body_control.mp4"
    gif_path = args.out_dir / "suspended_h1_full_body_control.gif"

    if try_write_mp4(frame_paths, mp4_path, args.fps):
        video_path = mp4_path
        video_kind = "mp4"
    else:
        write_gif(frame_paths, gif_path, args.fps)
        video_path = gif_path
        video_kind = "gif"

    write_manifest(args, frame_paths, video_path, video_kind)
    print(f"video={video_path}")
    print(f"video_kind={video_kind}")
    print(f"frames={len(frame_paths)}")
    print(f"frames_dir={frame_paths[0].parent}")
    print(f"manifest={args.out_dir / 'suspended_h1_full_body_manifest.txt'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
