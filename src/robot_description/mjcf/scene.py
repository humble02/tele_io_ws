#!/usr/bin/env python3
"""Render the MJCF scene camera with OpenCV at a fixed rate."""

from __future__ import annotations

import argparse
import importlib
import time
from pathlib import Path

import cv2
import mujoco


SCRIPT_DIR = Path(__file__).resolve().parent
SCENE_PATH = SCRIPT_DIR / "scene.xml"
DEFAULT_CAMERAS = ["head_camera", "left_wrist_camera", "right_wrist_camera"]
LEFT_ARM_JOINTS = [f"Joint{i}_L" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"Joint{i}_R" for i in range(1, 8)]
LEFT_ARM_QPOS = [-1.5708, 1.5708, 1.5708, -1.5708, 0.0, 0.0, 0.0]
RIGHT_ARM_QPOS = [1.5708, 1.5708, -1.5708, -1.5708, 0.0, 0.0, 0.0]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--camera",
        action="append",
        dest="cameras",
        help="MJCF camera name to render. Repeat to render a subset; defaults to all robot cameras.",
    )
    parser.add_argument("--width", type=int, default=1920, help="Rendered image width.")
    parser.add_argument("--height", type=int, default=1080, help="Rendered image height.")
    parser.add_argument("--display-scale", type=float, default=0.5, help="OpenCV display scale.")
    parser.add_argument("--hz", type=float, default=30.0, help="Target visualization rate.")
    parser.add_argument(
        "--frames",
        type=int,
        default=0,
        help="Stop after this many frames; 0 runs until the window is closed.",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Render without opening an OpenCV window.",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Do not open the MuJoCo viewer GUI.",
    )
    return parser.parse_args()


def set_joint_positions(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joint_names: list[str],
    joint_positions: list[float],
) -> None:
    for joint_name, joint_position in zip(joint_names, joint_positions, strict=True):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id < 0:
            raise ValueError(f"Joint not found: {joint_name}")
        data.qpos[model.jnt_qposadr[joint_id]] = joint_position


def validate_cameras(model: mujoco.MjModel, camera_names: list[str]) -> None:
    missing = [
        camera_name
        for camera_name in camera_names
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name) < 0
    ]
    if missing:
        available = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_id)
            for camera_id in range(model.ncam)
        ]
        raise ValueError(f"Camera not found: {missing}. Available cameras: {available}")


def main() -> None:
    args = parse_args()
    if args.hz <= 0:
        raise ValueError("--hz must be positive")
    if args.display_scale <= 0:
        raise ValueError("--display-scale must be positive")
    camera_names = args.cameras if args.cameras else DEFAULT_CAMERAS

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    data = mujoco.MjData(model)
    set_joint_positions(model, data, LEFT_ARM_JOINTS, LEFT_ARM_QPOS)
    set_joint_positions(model, data, RIGHT_ARM_JOINTS, RIGHT_ARM_QPOS)
    mujoco.mj_forward(model, data)
    validate_cameras(model, camera_names)

    frame_period = 1.0 / args.hz
    rendered = 0
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)
    viewer = None

    try:
        if not args.no_viewer:
            mujoco_viewer = importlib.import_module("mujoco.viewer")
            viewer = mujoco_viewer.launch_passive(model, data)

        while args.frames <= 0 or rendered < args.frames:
            frame_start = time.perf_counter()

            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()

            for camera_name in camera_names:
                renderer.update_scene(data, camera=camera_name)
                rgb = renderer.render()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

                if args.no_display:
                    continue
                display_image = cv2.resize(
                    bgr,
                    None,
                    fx=args.display_scale,
                    fy=args.display_scale,
                    interpolation=cv2.INTER_AREA,
                )
                cv2.imshow(camera_name, display_image)

            if not args.no_display:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break

            rendered += 1
            elapsed = time.perf_counter() - frame_start
            if elapsed < frame_period:
                time.sleep(frame_period - elapsed)
    finally:
        if viewer is not None:
            viewer.close()
        renderer.close()
        if not args.no_display:
            cv2.destroyAllWindows()

    print(f"Rendered {rendered} frame set(s) from {camera_names} at target {args.hz:g} Hz")


if __name__ == "__main__":
    main()
