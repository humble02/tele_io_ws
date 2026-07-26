#!/usr/bin/env python3

from __future__ import annotations

import math
from contextlib import suppress
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


LEFT_JOINT_NAMES = [f"Joint{index}_L" for index in range(1, 8)]
RIGHT_JOINT_NAMES = [f"Joint{index}_R" for index in range(1, 8)]
LEFT_ELBOW_TARGET_DEG = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
RIGHT_ELBOW_TARGET_DEG = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]


@dataclass
class ArmRuntime:
    side: str
    joint_names: list[str]
    state_topic: str
    command_topic: str
    target_rad: list[float]
    current: Optional[list[float]] = None
    start: Optional[list[float]] = None
    publisher: Optional[rclpy.publisher.Publisher] = None


def parse_arms(value: str) -> list[str]:
    normalized = value.strip().lower()
    if normalized in ("both", "all", ""):
        return ["left", "right"]
    if normalized in ("left", "right"):
        return [normalized]

    arms = [part.strip() for part in normalized.split(",") if part.strip()]
    invalid = [arm for arm in arms if arm not in ("left", "right")]
    if invalid:
        raise ValueError(f"Invalid arms value '{value}', expected left/right/both")
    return arms


class MarvinElbowPose(Node):
    def __init__(self) -> None:
        super().__init__("marvin_elbow_pose")

        self.declare_parameter("arms", "both")
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("hold_before_move_sec", 0.5)
        self.declare_parameter("move_duration_sec", 5.0)
        self.declare_parameter("timeout_sec", 30.0)
        self.declare_parameter("tolerance_rad", 0.02)
        self.declare_parameter("exit_on_success", True)
        self.declare_parameter("left_state_topic", "/marvin/left/joint_states")
        self.declare_parameter("right_state_topic", "/marvin/right/joint_states")
        self.declare_parameter("left_command_topic", "/marvin/left/joint_commands")
        self.declare_parameter("right_command_topic", "/marvin/right/joint_commands")
        self.declare_parameter("left_target_deg", LEFT_ELBOW_TARGET_DEG)
        self.declare_parameter("right_target_deg", RIGHT_ELBOW_TARGET_DEG)

        self._enabled_arms = parse_arms(str(self.get_parameter("arms").value))
        self._command_rate_hz = float(self.get_parameter("command_rate_hz").value)
        self._hold_before_move_sec = float(self.get_parameter("hold_before_move_sec").value)
        self._move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        self._timeout_sec = float(self.get_parameter("timeout_sec").value)
        self._tolerance_rad = float(self.get_parameter("tolerance_rad").value)
        self._exit_on_success = bool(self.get_parameter("exit_on_success").value)

        if self._command_rate_hz <= 0.0:
            raise ValueError(f"command_rate_hz must be > 0, got {self._command_rate_hz}")
        if self._hold_before_move_sec < 0.0:
            raise ValueError(
                f"hold_before_move_sec must be >= 0, got {self._hold_before_move_sec}"
            )
        if self._move_duration_sec <= 0.0:
            raise ValueError(f"move_duration_sec must be > 0, got {self._move_duration_sec}")
        if self._timeout_sec <= 0.0:
            raise ValueError(f"timeout_sec must be > 0, got {self._timeout_sec}")
        if self._tolerance_rad < 0.0:
            raise ValueError(f"tolerance_rad must be >= 0, got {self._tolerance_rad}")

        left_target = self._target_param_to_rad("left_target_deg")
        right_target = self._target_param_to_rad("right_target_deg")

        self._arms: Dict[str, ArmRuntime] = {}
        if "left" in self._enabled_arms:
            self._add_arm(
                "left",
                LEFT_JOINT_NAMES,
                str(self.get_parameter("left_state_topic").value),
                str(self.get_parameter("left_command_topic").value),
                left_target,
            )
        if "right" in self._enabled_arms:
            self._add_arm(
                "right",
                RIGHT_JOINT_NAMES,
                str(self.get_parameter("right_state_topic").value),
                str(self.get_parameter("right_command_topic").value),
                right_target,
            )

        self._start_time: Optional[rclpy.time.Time] = None
        self._wait_start_time = self.get_clock().now()
        self._motion_started = False
        self._shutdown_requested = False
        self._timer = self.create_timer(1.0 / self._command_rate_hz, self._on_timer)

        targets = ", ".join(
            f"{arm.side}={['%.1f' % math.degrees(v) for v in arm.target_rad]}"
            for arm in self._arms.values()
        )
        self.get_logger().info(
            "marvin_elbow_pose waiting for feedback: "
            f"arms={self._enabled_arms}, targets_deg={targets}, "
            f"move_duration={self._move_duration_sec:.2f}s"
        )

    def _target_param_to_rad(self, parameter_name: str) -> list[float]:
        values = [
            float(value)
            for value in self.get_parameter(parameter_name).get_parameter_value().double_array_value
        ]
        if len(values) != 7:
            raise ValueError(f"{parameter_name} must contain 7 values, got {len(values)}")
        return [math.radians(value) for value in values]

    def _add_arm(
        self,
        side: str,
        joint_names: list[str],
        state_topic: str,
        command_topic: str,
        target_rad: list[float],
    ) -> None:
        arm = ArmRuntime(
            side=side,
            joint_names=joint_names,
            state_topic=state_topic,
            command_topic=command_topic,
            target_rad=target_rad,
        )
        arm.publisher = self.create_publisher(JointState, command_topic, qos_profile_sensor_data)
        self.create_subscription(
            JointState,
            state_topic,
            lambda msg, arm_side=side: self._on_state(arm_side, msg),
            qos_profile_sensor_data,
        )
        self._arms[side] = arm

    def _on_state(self, side: str, msg: JointState) -> None:
        arm = self._arms[side]
        positions = self._extract_positions(msg, arm)
        if positions is None:
            return
        arm.current = positions

    def _on_timer(self) -> None:
        if self._shutdown_requested:
            return

        now = self.get_clock().now()
        if self._start_time is None:
            if self._ready_for_motion():
                self._capture_start(now)
            elif self._timed_out_waiting(now):
                missing = [side for side, arm in self._arms.items() if arm.current is None]
                self.get_logger().error(f"Timed out waiting for feedback from arms: {missing}")
                self._request_shutdown()
            return

        elapsed = (now - self._start_time).nanoseconds * 1e-9
        if elapsed > self._hold_before_move_sec + self._move_duration_sec + self._timeout_sec:
            self.get_logger().error("Timed out waiting for Marvin elbow pose target feedback")
            self._request_shutdown()
            return

        move_elapsed = max(0.0, elapsed - self._hold_before_move_sec)
        raw_alpha = min(1.0, move_elapsed / self._move_duration_sec)
        alpha = raw_alpha * raw_alpha * (3.0 - 2.0 * raw_alpha)

        for arm in self._arms.values():
            if arm.start is None:
                continue
            command = [
                start + (target - start) * alpha
                for start, target in zip(arm.start, arm.target_rad)
            ]
            self._publish_command(arm, command)

        if raw_alpha >= 1.0 and self._targets_reached():
            self.get_logger().info("Marvin elbow pose reached")
            if self._exit_on_success:
                self._request_shutdown()
            else:
                for arm in self._arms.values():
                    self._publish_command(arm, arm.target_rad)

    def _ready_for_motion(self) -> bool:
        return all(arm.current is not None for arm in self._arms.values())

    def _capture_start(self, now: rclpy.time.Time) -> None:
        for arm in self._arms.values():
            if arm.current is None:
                raise RuntimeError(f"Internal error: {arm.side} arm has no feedback")
            arm.start = list(arm.current)
        self._start_time = now
        self._motion_started = True
        self.get_logger().info("Feedback received, moving from current joint state")

    def _timed_out_waiting(self, now: rclpy.time.Time) -> bool:
        return (now - self._wait_start_time).nanoseconds * 1e-9 > self._timeout_sec

    def _targets_reached(self) -> bool:
        for arm in self._arms.values():
            if arm.current is None:
                return False
            for current, target in zip(arm.current, arm.target_rad):
                if abs(current - target) > self._tolerance_rad:
                    return False
        return True

    def _publish_command(self, arm: ArmRuntime, positions: list[float]) -> None:
        if arm.publisher is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(arm.joint_names)
        msg.position = [float(value) for value in positions]
        try:
            arm.publisher.publish(msg)
        except RCLError:
            if rclpy.ok():
                raise

    def _extract_positions(self, msg: JointState, arm: ArmRuntime) -> Optional[list[float]]:
        if not msg.position:
            self.get_logger().warn(
                f"Ignoring empty JointState from {arm.state_topic}",
                throttle_duration_sec=1.0,
            )
            return None

        if not msg.name:
            if len(msg.position) < len(arm.joint_names):
                self.get_logger().warn(
                    f"Ignoring unnamed JointState from {arm.state_topic}: expected "
                    f"{len(arm.joint_names)} positions, got {len(msg.position)}",
                    throttle_duration_sec=1.0,
                )
                return None
            return [float(value) for value in msg.position[: len(arm.joint_names)]]

        index_by_name = {name: index for index, name in enumerate(msg.name)}
        positions = []
        for name in arm.joint_names:
            index = index_by_name.get(name)
            if index is None or index >= len(msg.position):
                self.get_logger().warn(
                    f"Ignoring JointState from {arm.state_topic}: missing joint '{name}'",
                    throttle_duration_sec=1.0,
                )
                return None
            value = float(msg.position[index])
            if not math.isfinite(value):
                self.get_logger().warn(
                    f"Ignoring JointState from {arm.state_topic}: non-finite joint '{name}'",
                    throttle_duration_sec=1.0,
                )
                return None
            positions.append(value)
        return positions

    def _request_shutdown(self) -> None:
        self._shutdown_requested = True
        self._timer.cancel()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MarvinElbowPose()
    try:
        while rclpy.ok() and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with suppress(KeyboardInterrupt, RCLError):
            node.destroy_node()
        if rclpy.ok():
            with suppress(KeyboardInterrupt, RCLError):
                rclpy.shutdown()


if __name__ == "__main__":
    main()
