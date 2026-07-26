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


LEFT_ELBOW_POSE_DEG = [-90.0, 90.0, 90.0, -90.0, 0.0, 0.0, 0.0]
RIGHT_ELBOW_POSE_DEG = [90.0, 90.0, -90.0, -90.0, 0.0, 0.0, 0.0]


def arm_joint_names(suffix: str) -> list[str]:
    return [f"Joint{index}_{suffix}" for index in range(1, 8)]


def hand_joint_names(prefix: str) -> list[str]:
    return [
        f"{prefix}_finger{finger}_joint{joint}"
        for finger in range(1, 6)
        for joint in range(1, 5)
    ]


@dataclass
class DummyDevice:
    name: str
    command_topic: str
    state_topic: str
    joint_names: list[str]
    positions: list[float]
    publisher: Optional[rclpy.publisher.Publisher] = None


def enabled_sides(value: str) -> set[str]:
    normalized = value.strip().lower()
    if normalized in ("both", "all", "true", "yes", ""):
        return {"left", "right"}
    if normalized in ("none", "false", "no", "off"):
        return set()
    sides = {part.strip() for part in normalized.split(",") if part.strip()}
    return {side for side in sides if side in ("left", "right")}


def initial_arm_positions(side: str, pose: str) -> list[float]:
    normalized = pose.strip().lower()
    if normalized == "zero":
        return [0.0] * 7
    if normalized == "elbow":
        target_deg = LEFT_ELBOW_POSE_DEG if side == "left" else RIGHT_ELBOW_POSE_DEG
        return [math.radians(value) for value in target_deg]
    raise ValueError(f"initial_arm_pose must be 'zero' or 'elbow', got '{pose}'")


class DummyDriver(Node):
    def __init__(self) -> None:
        super().__init__("dummy_driver")

        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("arms", "both")
        self.declare_parameter("hands", "both")
        self.declare_parameter("initial_arm_pose", "zero")

        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0, got {publish_rate_hz}")

        arm_sides = enabled_sides(str(self.get_parameter("arms").value))
        hand_sides = enabled_sides(str(self.get_parameter("hands").value))
        initial_arm_pose = str(self.get_parameter("initial_arm_pose").value)

        self._devices: Dict[str, DummyDevice] = {}
        if "left" in arm_sides:
            self._add_device(
                "marvin_left",
                "/marvin/left/joint_commands",
                "/marvin/left/joint_states",
                arm_joint_names("L"),
                initial_arm_positions("left", initial_arm_pose),
            )
        if "right" in arm_sides:
            self._add_device(
                "marvin_right",
                "/marvin/right/joint_commands",
                "/marvin/right/joint_states",
                arm_joint_names("R"),
                initial_arm_positions("right", initial_arm_pose),
            )
        if "left" in hand_sides:
            self._add_device(
                "hand_left",
                "/hand_left/joint_commands",
                "/hand_left/joint_states",
                hand_joint_names("left"),
            )
        if "right" in hand_sides:
            self._add_device(
                "hand_right",
                "/hand_right/joint_commands",
                "/hand_right/joint_states",
                hand_joint_names("right"),
            )

        if not self._devices:
            raise RuntimeError("dummy_driver has no enabled devices")

        self.create_timer(1.0 / publish_rate_hz, self._publish_all)
        self.get_logger().info(
            "dummy_driver ready: "
            + ", ".join(
                f"{device.command_topic}->{device.state_topic}"
                for device in self._devices.values()
            )
        )

    def _add_device(
        self,
        name: str,
        command_topic: str,
        state_topic: str,
        joint_names: list[str],
        initial_positions: Optional[list[float]] = None,
    ) -> None:
        device = DummyDevice(
            name=name,
            command_topic=command_topic,
            state_topic=state_topic,
            joint_names=joint_names,
            positions=list(initial_positions or [0.0] * len(joint_names)),
        )
        device.publisher = self.create_publisher(
            JointState, state_topic, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            command_topic,
            lambda msg, device_name=name: self._on_command(device_name, msg),
            qos_profile_sensor_data,
        )
        self._devices[name] = device

    def _on_command(self, device_name: str, msg: JointState) -> None:
        device = self._devices[device_name]
        if not msg.position:
            self.get_logger().warn(
                f"Ignoring empty command on {device.command_topic}",
                throttle_duration_sec=1.0,
            )
            return

        if msg.name:
            updated = self._apply_named_command(device, msg)
            if updated == 0:
                self.get_logger().warn(
                    f"Ignoring command on {device.command_topic}: no known joint names",
                    throttle_duration_sec=1.0,
                )
                return
        else:
            if len(msg.position) < len(device.joint_names):
                self.get_logger().warn(
                    f"Ignoring command on {device.command_topic}: expected "
                    f"{len(device.joint_names)} positions, got {len(msg.position)}",
                    throttle_duration_sec=1.0,
                )
                return
            device.positions = [float(value) for value in msg.position[: len(device.joint_names)]]

        self._publish_device(device)

    def _apply_named_command(self, device: DummyDevice, msg: JointState) -> int:
        index_by_name = {name: index for index, name in enumerate(device.joint_names)}
        updated = 0
        for name, position in zip(msg.name, msg.position):
            index = index_by_name.get(name)
            if index is None:
                continue
            device.positions[index] = float(position)
            updated += 1
        return updated

    def _publish_all(self) -> None:
        for device in self._devices.values():
            self._publish_device(device)

    def _publish_device(self, device: DummyDevice) -> None:
        if device.publisher is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(device.joint_names)
        msg.position = list(device.positions)
        device.publisher.publish(msg)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DummyDriver()
    try:
        rclpy.spin(node)
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
