"""IO end-effector pose teleoperation for Marvin arms."""

from __future__ import annotations

import argparse
import sys
from contextlib import suppress
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

from .kine_solver import MarvinIkSolver


@dataclass
class IoTeleopConfig:
    control_rate_hz: float = 60.0
    auto_enable: bool = False
    feedback_timeout_sec: float = 0.5
    target_timeout_sec: float = 0.5
    target_pose_topic: str = "/io_teleop/target_ee_poses"
    pose_order: str = "right_left"
    transform_from_common_base: bool = True
    position_scale: float = 0.8
    left_arm_base_xyz: list[float] = None
    left_arm_base_rpy: list[float] = None
    right_arm_base_xyz: list[float] = None
    right_arm_base_rpy: list[float] = None
    left_feedback_topic: str = "/marvin/left/joint_states"
    right_feedback_topic: str = "/marvin/right/joint_states"
    left_command_topic: str = "/marvin/left/joint_commands"
    right_command_topic: str = "/marvin/right/joint_commands"
    left_joint_names: list[str] = None
    right_joint_names: list[str] = None
    left_ik_reference_deg: list[float] = None
    right_ik_reference_deg: list[float] = None
    zsp_type: int = 1
    left_zsp_para: list[float] = None
    right_zsp_para: list[float] = None
    zsp_angle_deg: float = 0.0
    singular_tolerance_deg: list[float] = None
    max_joint_step_rad: float = 0.02
    command_publish_on_change_only: bool = False

    @classmethod
    def from_file(cls, path: Path) -> "IoTeleopConfig":
        with path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
        raw = raw.get("io_marvin_teleop", raw)
        valid = {field.name for field in fields(cls)}
        cfg = cls(**{key: value for key, value in raw.items() if key in valid})
        cfg.left_joint_names = cfg.left_joint_names or [f"Joint{i}_L" for i in range(1, 8)]
        cfg.right_joint_names = cfg.right_joint_names or [f"Joint{i}_R" for i in range(1, 8)]
        cfg.left_ik_reference_deg = cfg.left_ik_reference_deg or [
            -90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0
        ]
        cfg.right_ik_reference_deg = cfg.right_ik_reference_deg or [
            90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0
        ]
        cfg.left_zsp_para = cfg.left_zsp_para or [0.0, -1.0, -0.5, 0.0, 0.0, 0.0]
        cfg.right_zsp_para = cfg.right_zsp_para or [0.0, 1.0, -0.5, 0.0, 0.0, 0.0]
        cfg.singular_tolerance_deg = cfg.singular_tolerance_deg or [5.0, 5.0, 5.0]
        cfg.left_arm_base_xyz = cfg.left_arm_base_xyz or [0.02201, 0.03725, 0.065]
        cfg.left_arm_base_rpy = cfg.left_arm_base_rpy or [-1.5708, 0.0, 0.0]
        cfg.right_arm_base_xyz = cfg.right_arm_base_xyz or [0.02201, -0.03725, 0.065]
        cfg.right_arm_base_rpy = cfg.right_arm_base_rpy or [1.5708, 0.0, 0.0]
        return cfg


class ArmState:
    def __init__(self, side: str, joint_names: list[str]) -> None:
        self.side = side
        self.joint_names = list(joint_names)
        self.feedback_rad: Optional[list[float]] = None
        self.feedback_stamp: Optional[Time] = None
        self.last_command_rad: Optional[list[float]] = None
        self.last_warn: dict[str, Time] = {}


class IoMarvinTeleopNode(Node):
    def __init__(self, config_path: Path, kine_config_path: Path) -> None:
        super().__init__("io_marvin_teleop")
        self._cfg = IoTeleopConfig.from_file(config_path)
        self._validate_config()
        self._ik = MarvinIkSolver(kine_config_path)
        self._enabled = bool(self._cfg.auto_enable)

        self._left = ArmState("left", self._cfg.left_joint_names)
        self._right = ArmState("right", self._cfg.right_joint_names)
        self._ik_reference_rad = {
            "left": [float(np.deg2rad(value)) for value in self._cfg.left_ik_reference_deg],
            "right": [float(np.deg2rad(value)) for value in self._cfg.right_ik_reference_deg],
        }
        self._targets: dict[str, Optional[np.ndarray]] = {"left": None, "right": None}
        self._target_stamp: Optional[Time] = None
        self._arm_base_inverse = {
            "left": np.linalg.inv(
                self._xyz_rpy_to_matrix(
                    self._cfg.left_arm_base_xyz, self._cfg.left_arm_base_rpy
                )
            ),
            "right": np.linalg.inv(
                self._xyz_rpy_to_matrix(
                    self._cfg.right_arm_base_xyz, self._cfg.right_arm_base_rpy
                )
            ),
        }

        self._left_pub = self.create_publisher(
            JointState, self._cfg.left_command_topic, qos_profile_sensor_data
        )
        self._right_pub = self.create_publisher(
            JointState, self._cfg.right_command_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            self._cfg.left_feedback_topic,
            lambda msg: self._on_feedback(self._left, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._cfg.right_feedback_topic,
            lambda msg: self._on_feedback(self._right, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseArray,
            self._cfg.target_pose_topic,
            self._on_target_poses,
            qos_profile_sensor_data,
        )
        self.create_service(SetBool, "~/set_enabled", self._set_enabled)
        self.create_timer(1.0 / self._cfg.control_rate_hz, self._control)

        self.get_logger().info(
            "io_marvin_teleop ready: "
            f"enabled={self._enabled}, target={self._cfg.target_pose_topic}, "
            f"pose_order={self._cfg.pose_order}, position_scale={self._cfg.position_scale:.3f}, "
            f"rate={self._cfg.control_rate_hz:.1f} Hz"
        )

    def _validate_config(self) -> None:
        if self._cfg.control_rate_hz <= 0.0:
            raise ValueError("control_rate_hz must be > 0")
        if self._cfg.feedback_timeout_sec <= 0.0 or self._cfg.target_timeout_sec <= 0.0:
            raise ValueError("feedback and target timeouts must be > 0")
        if self._cfg.pose_order not in ("left_right", "right_left"):
            raise ValueError("pose_order must be left_right or right_left")
        if self._cfg.position_scale <= 0.0:
            raise ValueError("position_scale must be > 0")
        if len(self._cfg.left_joint_names) != 7 or len(self._cfg.right_joint_names) != 7:
            raise ValueError("each arm must have exactly 7 joint names")
        if (
            len(self._cfg.left_ik_reference_deg) != 7
            or len(self._cfg.right_ik_reference_deg) != 7
        ):
            raise ValueError("each IK reference must contain exactly 7 joint angles")
        if not all(
            np.isfinite(value)
            for value in self._cfg.left_ik_reference_deg + self._cfg.right_ik_reference_deg
        ):
            raise ValueError("IK reference angles must be finite")
        transform_values = (
            self._cfg.left_arm_base_xyz,
            self._cfg.left_arm_base_rpy,
            self._cfg.right_arm_base_xyz,
            self._cfg.right_arm_base_rpy,
        )
        if any(len(values) != 3 for values in transform_values):
            raise ValueError("arm base xyz/rpy parameters must each contain 3 values")

    def _on_feedback(self, arm: ArmState, msg: JointState) -> None:
        positions = self._extract_positions(msg, arm.joint_names)
        if positions is None or not all(np.isfinite(positions)):
            self._warn_throttled(arm, "feedback", "Ignoring invalid arm feedback")
            return
        arm.feedback_rad = positions
        arm.feedback_stamp = self.get_clock().now()
        if arm.last_command_rad is None:
            arm.last_command_rad = list(positions)

    def _on_target_poses(self, msg: PoseArray) -> None:
        if len(msg.poses) < 2:
            self.get_logger().warn(
                f"Ignoring {self._cfg.target_pose_topic}: expected 2 poses, got {len(msg.poses)}",
                throttle_duration_sec=1.0,
            )
            return

        first_side, second_side = self._cfg.pose_order.split("_")
        first = self._pose_to_matrix(msg.poses[0])
        second = self._pose_to_matrix(msg.poses[1])
        if first is None or second is None:
            self.get_logger().warn(
                f"Ignoring {self._cfg.target_pose_topic}: invalid position or quaternion",
                throttle_duration_sec=1.0,
            )
            return
        self._targets[first_side] = self._prepare_target(first_side, first)
        self._targets[second_side] = self._prepare_target(second_side, second)
        self._target_stamp = self.get_clock().now()

    def _prepare_target(self, side: str, target: np.ndarray) -> np.ndarray:
        prepared = np.array(target, dtype=float, copy=True)
        if self._cfg.transform_from_common_base:
            prepared = self._arm_base_inverse[side] @ prepared
        prepared[:3, 3] *= float(self._cfg.position_scale)
        return prepared

    def _set_enabled(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            missing = [
                arm.side for arm in (self._left, self._right) if not self._feedback_fresh(arm)
            ]
            if missing:
                response.success = False
                response.message = f"fresh feedback missing for: {', '.join(missing)}"
                return response
            if not self._target_fresh():
                response.success = False
                response.message = "fresh IO target poses missing"
                return response
            self._left.last_command_rad = list(self._left.feedback_rad)
            self._right.last_command_rad = list(self._right.feedback_rad)
            self._enabled = True
            response.success = True
            response.message = "IO Marvin teleop enabled"
        else:
            self._enabled = False
            response.success = True
            response.message = "IO Marvin teleop disabled"
        self.get_logger().info(response.message)
        return response

    def _control(self) -> None:
        if not self._enabled:
            return
        if not self._target_fresh():
            self.get_logger().warn(
                "IO target poses stale; skipping commands", throttle_duration_sec=1.0
            )
            return
        self._control_arm(self._left, self._cfg.left_zsp_para, self._left_pub)
        self._control_arm(self._right, self._cfg.right_zsp_para, self._right_pub)

    def _control_arm(self, arm: ArmState, zsp_para: list[float], publisher) -> None:
        if not self._feedback_fresh(arm):
            self._warn_throttled(arm, "feedback_stale", f"{arm.side} feedback stale; skipping")
            return
        target = self._targets[arm.side]
        if target is None:
            return
        result = self._ik.solve(
            side=arm.side,
            target_matrix_m=target,
            reference_joints_rad=self._ik_reference_rad[arm.side],
            zsp_type=self._cfg.zsp_type,
            zsp_para=zsp_para,
            zsp_angle_deg=self._cfg.zsp_angle_deg,
            singular_tolerance_deg=self._cfg.singular_tolerance_deg,
        )
        if not result.success:
            self._warn_throttled(arm, "ik", f"{arm.side} IK failed: {result.reason}")
            return

        target_rad = [float(np.deg2rad(value)) for value in result.joints_deg]
        command_rad = self._limit_step(arm, target_rad)
        if self._cfg.command_publish_on_change_only and arm.last_command_rad == command_rad:
            return
        arm.last_command_rad = list(command_rad)
        publisher.publish(self._make_joint_state(arm.joint_names, command_rad))

    def _feedback_fresh(self, arm: ArmState) -> bool:
        if arm.feedback_rad is None or arm.feedback_stamp is None:
            return False
        age = (self.get_clock().now() - arm.feedback_stamp).nanoseconds * 1e-9
        return age <= self._cfg.feedback_timeout_sec

    def _target_fresh(self) -> bool:
        if self._target_stamp is None or any(value is None for value in self._targets.values()):
            return False
        age = (self.get_clock().now() - self._target_stamp).nanoseconds * 1e-9
        return age <= self._cfg.target_timeout_sec

    def _limit_step(self, arm: ArmState, target_rad: list[float]) -> list[float]:
        if arm.last_command_rad is None or self._cfg.max_joint_step_rad <= 0.0:
            return list(target_rad)
        current = np.asarray(arm.last_command_rad, dtype=float)
        target = np.asarray(target_rad, dtype=float)
        limit = float(self._cfg.max_joint_step_rad)
        return list(current + np.clip(target - current, -limit, limit))

    def _make_joint_state(self, names: list[str], positions: list[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(names)
        msg.position = [float(value) for value in positions]
        return msg

    def _warn_throttled(self, arm: ArmState, key: str, message: str) -> None:
        now = self.get_clock().now()
        last = arm.last_warn.get(key)
        if last is None or (now - last).nanoseconds > 2_000_000_000:
            arm.last_warn[key] = now
            self.get_logger().warn(message)

    @staticmethod
    def _extract_positions(msg: JointState, names: list[str]) -> Optional[list[float]]:
        if msg.name:
            by_name = {name: value for name, value in zip(msg.name, msg.position)}
            if any(name not in by_name for name in names):
                return None
            return [float(by_name[name]) for name in names]
        if len(msg.position) < len(names):
            return None
        return [float(value) for value in msg.position[: len(names)]]

    @staticmethod
    def _pose_to_matrix(pose: Pose) -> Optional[np.ndarray]:
        values = np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
        if not np.all(np.isfinite(values)):
            return None
        x, y, z, w = values[3:]
        norm = float(np.linalg.norm([x, y, z, w]))
        if norm <= 1e-9:
            return None
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
        matrix[:3, 3] = values[:3]
        return matrix

    @staticmethod
    def _xyz_rpy_to_matrix(xyz: list[float], rpy: list[float]) -> np.ndarray:
        roll, pitch, yaw = [float(value) for value in rpy]
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        rotation_x = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
        rotation_y = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
        rotation_z = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = rotation_z @ rotation_y @ rotation_x
        matrix[:3, 3] = [float(value) for value in xyz]
        return matrix


def _package_file(*parts: str) -> Path:
    return Path(get_package_share_directory("vive_marvin_teleop")).joinpath(*parts)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="IO PoseArray teleoperation for Marvin")
    parser.add_argument("--config", default=str(_package_file("config", "io_marvin_teleop.yaml")))
    parser.add_argument("--kine-config", default=str(_package_file("config", "ccs_m6.MvKDCfg")))
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    program_name = sys.argv[0] if sys.argv else "io_marvin_teleop_node"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])
    rclpy.init(args=raw_argv)
    node = None
    try:
        node = IoMarvinTeleopNode(Path(args.config), Path(args.kine_config))
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            with suppress(KeyboardInterrupt):
                node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt):
                rclpy.shutdown()
    return 0
