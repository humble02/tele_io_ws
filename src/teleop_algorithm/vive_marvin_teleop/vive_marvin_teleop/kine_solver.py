"""Marvin IK wrapper used by the Vive teleoperation node."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from ._internal.fx_kine import Marvin_Kine


@dataclass
class IkResult:
    """IK output for one arm."""

    success: bool
    joints_deg: Optional[list[float]]
    reason: str = ""


class MarvinIkSolver:
    """Dual-arm Marvin IK adapter.

    The vendored SDK expects target translations in millimeters and Euler
    angles in degrees. Public ROS interfaces stay in meters/radians.
    """

    def __init__(self, config_path: str | Path):
        cfg_path = Path(config_path).expanduser().resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"Marvin IK config not found: {cfg_path}")

        config = Marvin_Kine().load_config(config_path=str(cfg_path))
        if config is None:
            raise RuntimeError(f"Failed to load Marvin IK config: {cfg_path}")

        self._left = Marvin_Kine()
        self._left.initial_kine(
            robot_serial=0,
            robot_type=config["TYPE"][0],
            dh=config["DH"][0],
            pnva=config["PNVA"][0],
            j67=config["BD"][0],
        )
        self._set_identity_tool(self._left, robot_serial=0)
        self._right = Marvin_Kine()
        self._right.initial_kine(
            robot_serial=1,
            robot_type=config["TYPE"][1],
            dh=config["DH"][1],
            pnva=config["PNVA"][1],
            j67=config["BD"][1],
        )
        self._set_identity_tool(self._right, robot_serial=1)

    @staticmethod
    def _set_identity_tool(kine: Marvin_Kine, robot_serial: int) -> None:
        identity = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        if kine.set_tool_kine(robot_serial, identity) is False:
            raise RuntimeError(f"Failed to set identity tool kinematics for robot_serial={robot_serial}")

    def solve(
        self,
        side: str,
        target_matrix_m: np.ndarray,
        reference_joints_rad: list[float],
        zsp_type: int,
        zsp_para: list[float],
        zsp_angle_deg: float,
        singular_tolerance_deg: list[float],
    ) -> IkResult:
        if len(reference_joints_rad) != 7:
            return IkResult(False, None, "reference_joints_rad must have 7 values")

        kine = self._left if side == "left" else self._right
        robot_serial = 0 if side == "left" else 1
        ref_deg = [float(np.rad2deg(value)) for value in reference_joints_rad]
        target_pose = self._matrix_to_xyzabc(target_matrix_m)
        target_pose_mm = list(target_pose)
        for index in range(3):
            target_pose_mm[index] *= 1000.0

        try:
            target_mat_mm = kine.xyzabc_to_mat4x4(target_pose_mm)
            if target_mat_mm is False:
                return IkResult(False, None, "xyzabc_to_mat4x4 failed")
            ik = kine.ik(
                robot_serial=robot_serial,
                pose_mat=target_mat_mm,
                ref_joints=ref_deg,
                zsp_type=int(zsp_type),
                zsp_para=list(zsp_para),
                zsp_angle=float(zsp_angle_deg),
                dgr=list(singular_tolerance_deg),
            )
        except Exception as exc:
            return IkResult(False, None, f"IK exception: {exc}")

        if ik is False:
            return IkResult(False, None, "IK returned false")
        if bool(ik.m_Output_IsOutRange):
            return IkResult(False, None, "IK target out of range")
        if bool(ik.m_Output_IsJntExd):
            return IkResult(False, None, "IK joint limit exceeded")
        raw_joints_deg = ik.m_Output_RetJoint.to_list()
        if not all(np.isfinite(raw_joints_deg)):
            return IkResult(False, None, "IK returned non-finite joints")

        # The SDK can return equivalent multi-turn angles (for example -835 deg).
        # Marvin joint limits are contained within one turn, so canonicalize before
        # applying the ROS-side command limiter.
        joints_deg = [
            (raw + 180.0) % 360.0 - 180.0
            for raw in raw_joints_deg
        ]
        return IkResult(True, joints_deg)

    @staticmethod
    def _matrix_to_xyzabc(matrix: np.ndarray) -> np.ndarray:
        xyz = matrix[:3, 3]
        # Match the official tianji_arm_node convention:
        # SciPy as_euler('ZYX') returns yaw,pitch,roll; the official code
        # reverses that to RX,RY,RZ.
        rotation = matrix[:3, :3]
        pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
        cos_pitch = np.cos(pitch)
        if abs(cos_pitch) > 1e-9:
            roll = np.arctan2(rotation[2, 1], rotation[2, 2])
            yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
        else:
            roll = 0.0
            yaw = np.arctan2(-rotation[0, 1], rotation[1, 1])
        rpy = np.rad2deg([roll, pitch, yaw])
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]], dtype=float)
