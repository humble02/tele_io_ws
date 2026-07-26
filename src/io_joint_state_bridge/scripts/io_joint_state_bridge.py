#!/usr/bin/env python3

from __future__ import annotations

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
LEFT_HAND_JOINT_NAMES = [
    f"left_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)
]
RIGHT_HAND_JOINT_NAMES = [
    f"right_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)
]
ARM_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
HAND_JOINT_NAMES = LEFT_HAND_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES
STATE_JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES
DEFAULT_IO_COMMAND_JOINT_NAMES = RIGHT_JOINT_NAMES + LEFT_JOINT_NAMES
DEFAULT_IO_STATE_JOINT_NAMES = (
    RIGHT_JOINT_NAMES + LEFT_JOINT_NAMES + LEFT_HAND_JOINT_NAMES + RIGHT_HAND_JOINT_NAMES
)


@dataclass
class JointCache:
    names: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]
    has_velocity: bool = False
    has_effort: bool = False
    received: bool = False


class IoJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("io_joint_state_bridge")

        self.declare_parameter("publish_rate_hz", 1000.0)
        self.declare_parameter("require_both_feedback", True)
        self.declare_parameter("require_hand_feedback", False)
        self.declare_parameter("forward_arm_commands", True)
        self.declare_parameter("io_joint_names", DEFAULT_IO_COMMAND_JOINT_NAMES)
        self.declare_parameter("io_command_joint_names", DEFAULT_IO_COMMAND_JOINT_NAMES)
        self.declare_parameter("io_state_joint_names", DEFAULT_IO_STATE_JOINT_NAMES)
        self.declare_parameter("io_state_topic", "/io_teleop/joint_states")
        self.declare_parameter("io_command_topic", "/io_teleop/joint_cmd")
        self.declare_parameter(
            "io_left_hand_command_topic", "/io_teleop/joint_cmd_finger_left"
        )
        self.declare_parameter(
            "io_right_hand_command_topic", "/io_teleop/joint_cmd_finger_right"
        )
        self.declare_parameter("left_state_topic", "/marvin/left/joint_states")
        self.declare_parameter("right_state_topic", "/marvin/right/joint_states")
        self.declare_parameter("left_hand_state_topic", "/hand_left/joint_states")
        self.declare_parameter("right_hand_state_topic", "/hand_right/joint_states")
        self.declare_parameter("left_command_topic", "/marvin/left/joint_commands")
        self.declare_parameter("right_command_topic", "/marvin/right/joint_commands")
        self.declare_parameter("left_hand_command_topic", "/hand_left/joint_commands")
        self.declare_parameter("right_hand_command_topic", "/hand_right/joint_commands")

        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self._publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0, got {self._publish_rate_hz}")

        self._require_both_feedback = bool(self.get_parameter("require_both_feedback").value)
        self._require_hand_feedback = bool(self.get_parameter("require_hand_feedback").value)
        self._forward_arm_commands = bool(
            self.get_parameter("forward_arm_commands").value
        )
        self._io_command_joint_names = list(
            self.get_parameter("io_command_joint_names")
            .get_parameter_value()
            .string_array_value
        )
        if not self._io_command_joint_names:
            self._io_command_joint_names = list(
                self.get_parameter("io_joint_names").get_parameter_value().string_array_value
            )
        self._io_state_joint_names = list(
            self.get_parameter("io_state_joint_names").get_parameter_value().string_array_value
        )
        self._validate_io_command_joint_names(self._io_command_joint_names)
        self._validate_io_state_joint_names(self._io_state_joint_names)
        self._io_state_topic = str(self.get_parameter("io_state_topic").value)
        self._io_command_topic = str(self.get_parameter("io_command_topic").value)
        self._io_left_hand_command_topic = str(
            self.get_parameter("io_left_hand_command_topic").value
        )
        self._io_right_hand_command_topic = str(
            self.get_parameter("io_right_hand_command_topic").value
        )
        self._left_state_topic = str(self.get_parameter("left_state_topic").value)
        self._right_state_topic = str(self.get_parameter("right_state_topic").value)
        self._left_hand_state_topic = str(self.get_parameter("left_hand_state_topic").value)
        self._right_hand_state_topic = str(self.get_parameter("right_hand_state_topic").value)
        self._left_command_topic = str(self.get_parameter("left_command_topic").value)
        self._right_command_topic = str(self.get_parameter("right_command_topic").value)
        self._left_hand_command_topic = str(
            self.get_parameter("left_hand_command_topic").value
        )
        self._right_hand_command_topic = str(
            self.get_parameter("right_hand_command_topic").value
        )

        self._left = JointCache(
            names=LEFT_JOINT_NAMES,
            position=[0.0] * len(LEFT_JOINT_NAMES),
            velocity=[0.0] * len(LEFT_JOINT_NAMES),
            effort=[0.0] * len(LEFT_JOINT_NAMES),
        )
        self._right = JointCache(
            names=RIGHT_JOINT_NAMES,
            position=[0.0] * len(RIGHT_JOINT_NAMES),
            velocity=[0.0] * len(RIGHT_JOINT_NAMES),
            effort=[0.0] * len(RIGHT_JOINT_NAMES),
        )
        self._left_hand = JointCache(
            names=LEFT_HAND_JOINT_NAMES,
            position=[0.0] * len(LEFT_HAND_JOINT_NAMES),
            velocity=[0.0] * len(LEFT_HAND_JOINT_NAMES),
            effort=[0.0] * len(LEFT_HAND_JOINT_NAMES),
        )
        self._right_hand = JointCache(
            names=RIGHT_HAND_JOINT_NAMES,
            position=[0.0] * len(RIGHT_HAND_JOINT_NAMES),
            velocity=[0.0] * len(RIGHT_HAND_JOINT_NAMES),
            effort=[0.0] * len(RIGHT_HAND_JOINT_NAMES),
        )

        self._io_state_pub = self.create_publisher(
            JointState, self._io_state_topic, qos_profile_sensor_data
        )
        self._left_command_pub = None
        self._right_command_pub = None
        if self._forward_arm_commands:
            self._left_command_pub = self.create_publisher(
                JointState, self._left_command_topic, qos_profile_sensor_data
            )
            self._right_command_pub = self.create_publisher(
                JointState, self._right_command_topic, qos_profile_sensor_data
            )
        self._left_hand_command_pub = self.create_publisher(
            JointState, self._left_hand_command_topic, qos_profile_sensor_data
        )
        self._right_hand_command_pub = self.create_publisher(
            JointState, self._right_hand_command_topic, qos_profile_sensor_data
        )

        self.create_subscription(
            JointState,
            self._left_state_topic,
            lambda msg: self._on_arm_state(self._left, msg, self._left_state_topic),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._right_state_topic,
            lambda msg: self._on_arm_state(self._right, msg, self._right_state_topic),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._left_hand_state_topic,
            lambda msg: self._on_arm_state(self._left_hand, msg, self._left_hand_state_topic),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._right_hand_state_topic,
            lambda msg: self._on_arm_state(self._right_hand, msg, self._right_hand_state_topic),
            qos_profile_sensor_data,
        )
        if self._forward_arm_commands:
            self.create_subscription(
                JointState,
                self._io_command_topic,
                self._on_io_command,
                qos_profile_sensor_data,
            )
        self.create_subscription(
            JointState,
            self._io_left_hand_command_topic,
            lambda msg: self._on_io_hand_command(
                msg,
                LEFT_HAND_JOINT_NAMES,
                self._left_hand_command_pub,
                self._io_left_hand_command_topic,
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._io_right_hand_command_topic,
            lambda msg: self._on_io_hand_command(
                msg,
                RIGHT_HAND_JOINT_NAMES,
                self._right_hand_command_pub,
                self._io_right_hand_command_topic,
            ),
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self._publish_rate_hz, self._publish_io_state)
        self.get_logger().info(
            "io_joint_state_bridge ready: "
            f"{self._left_state_topic}+{self._right_state_topic}+"
            f"{self._left_hand_state_topic}+{self._right_hand_state_topic} "
            f"-> {self._io_state_topic}, "
            f"arm_command_forwarding={self._forward_arm_commands}, "
            f"{self._io_left_hand_command_topic} -> {self._left_hand_command_topic}, "
            f"{self._io_right_hand_command_topic} -> {self._right_hand_command_topic}, "
            f"rate={self._publish_rate_hz:.1f} Hz"
        )

    def _on_arm_state(self, cache: JointCache, msg: JointState, source: str) -> None:
        positions = self._extract_by_names(msg, cache.names, source)
        if positions is None:
            return

        cache.position = positions
        cache.velocity = self._extract_optional_by_names(msg.name, msg.velocity, cache.names)
        cache.effort = self._extract_optional_by_names(msg.name, msg.effort, cache.names)
        cache.has_velocity = bool(msg.velocity) and len(cache.velocity) == len(cache.names)
        cache.has_effort = bool(msg.effort) and len(cache.effort) == len(cache.names)
        cache.received = True

    def _publish_io_state(self) -> None:
        if self._require_both_feedback and (not self._left.received or not self._right.received):
            return
        if self._require_hand_feedback and (
            not self._left_hand.received or not self._right_hand.received
        ):
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        position_by_name = self._cache_values_by_name("position")
        velocity_by_name = self._cache_values_by_name("velocity")
        effort_by_name = self._cache_values_by_name("effort")

        msg.name = list(self._io_state_joint_names)
        msg.position = [position_by_name[name] for name in self._io_state_joint_names]
        if all(cache.has_velocity for cache in self._state_caches()):
            msg.velocity = [velocity_by_name[name] for name in self._io_state_joint_names]
        if all(cache.has_effort for cache in self._state_caches()):
            msg.effort = [effort_by_name[name] for name in self._io_state_joint_names]

        try:
            self._io_state_pub.publish(msg)
        except RCLError:
            if rclpy.ok():
                raise

    def _on_io_command(self, msg: JointState) -> None:
        named_msg = self._normalize_io_command(msg)
        if named_msg is None:
            return

        left_command = self._extract_command_side(named_msg, LEFT_JOINT_NAMES, "left")
        right_command = self._extract_command_side(named_msg, RIGHT_JOINT_NAMES, "right")
        if left_command is None or right_command is None:
            return

        if self._left_command_pub is None or self._right_command_pub is None:
            return
        self._left_command_pub.publish(left_command)
        self._right_command_pub.publish(right_command)

    def _on_io_hand_command(
        self,
        msg: JointState,
        joint_names: list[str],
        publisher: rclpy.publisher.Publisher,
        source: str,
    ) -> None:
        positions = self._extract_by_names(msg, joint_names, source)
        if positions is None:
            return

        out = JointState()
        out.header.stamp = msg.header.stamp
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(joint_names)
        out.position = positions
        out.velocity = self._extract_optional_by_names(msg.name, msg.velocity, joint_names)
        out.effort = self._extract_optional_by_names(msg.name, msg.effort, joint_names)
        publisher.publish(out)

    def _extract_command_side(
        self,
        msg: JointState,
        side_names: list[str],
        side: str,
    ) -> Optional[JointState]:
        out = JointState()
        out.header.stamp = msg.header.stamp
        if out.header.stamp.sec == 0 and out.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(side_names)

        out.position = self._extract_by_names(msg, side_names, self._io_command_topic) or []
        if not out.position:
            self.get_logger().warn(
                f"Ignoring {self._io_command_topic}: missing {side} arm joint positions",
                throttle_duration_sec=1.0,
            )
            return None
        out.velocity = self._extract_optional_by_names(msg.name, msg.velocity, side_names)
        out.effort = self._extract_optional_by_names(msg.name, msg.effort, side_names)
        return out

    def _normalize_io_command(self, msg: JointState) -> Optional[JointState]:
        if msg.name:
            return msg

        if len(msg.position) < len(self._io_command_joint_names):
            self.get_logger().warn(
                f"Ignoring unnamed {self._io_command_topic}: expected "
                f"{len(self._io_command_joint_names)} arm positions, got {len(msg.position)}",
                throttle_duration_sec=1.0,
            )
            return None

        named = JointState()
        named.header = msg.header
        named.name = list(self._io_command_joint_names)
        named.position = [
            float(value) for value in msg.position[: len(self._io_command_joint_names)]
        ]
        if len(msg.velocity) >= len(self._io_command_joint_names):
            named.velocity = [
                float(value) for value in msg.velocity[: len(self._io_command_joint_names)]
            ]
        if len(msg.effort) >= len(self._io_command_joint_names):
            named.effort = [
                float(value) for value in msg.effort[: len(self._io_command_joint_names)]
            ]
        return named

    def _cache_values_by_name(self, field_name: str) -> dict[str, float]:
        values = {}
        for cache in self._state_caches():
            values.update(dict(zip(cache.names, getattr(cache, field_name))))
        return values

    @staticmethod
    def _validate_io_command_joint_names(joint_names: list[str]) -> None:
        if len(joint_names) != len(ARM_JOINT_NAMES):
            raise ValueError(
                f"io_command_joint_names must contain 14 arm joints, got {len(joint_names)}"
            )
        if set(joint_names) != set(ARM_JOINT_NAMES):
            raise ValueError(
                "io_command_joint_names must contain exactly "
                "Joint1_L..Joint7_L and Joint1_R..Joint7_R"
            )

    @staticmethod
    def _validate_io_state_joint_names(joint_names: list[str]) -> None:
        if len(joint_names) != len(STATE_JOINT_NAMES):
            raise ValueError(
                f"io_state_joint_names must contain 54 arm+hand joints, got {len(joint_names)}"
            )
        if set(joint_names) != set(STATE_JOINT_NAMES):
            raise ValueError(
                "io_state_joint_names must contain Marvin arm joints and Wuji hand joints"
            )

    def _state_caches(self) -> list[JointCache]:
        return [self._left, self._right, self._left_hand, self._right_hand]

    def _extract_by_names(
        self,
        msg: JointState,
        expected_names: list[str],
        source: str,
    ) -> Optional[list[float]]:
        if not msg.name:
            if len(msg.position) < len(expected_names):
                self.get_logger().warn(
                    f"Ignoring unnamed JointState from {source}: expected "
                    f"{len(expected_names)} positions, got {len(msg.position)}",
                    throttle_duration_sec=1.0,
                )
                return None
            return [float(value) for value in msg.position[: len(expected_names)]]

        index_by_name: Dict[str, int] = {name: index for index, name in enumerate(msg.name)}
        values: list[float] = []
        for name in expected_names:
            index = index_by_name.get(name)
            if index is None or index >= len(msg.position):
                self.get_logger().warn(
                    f"Ignoring JointState from {source}: missing joint '{name}'",
                    throttle_duration_sec=1.0,
                )
                return None
            values.append(float(msg.position[index]))
        return values

    @staticmethod
    def _extract_optional_by_names(
        names: list[str],
        values: List[float],
        expected_names: list[str],
    ) -> list[float]:
        if not values:
            return [0.0] * len(expected_names)
        if not names:
            if len(values) >= len(expected_names):
                return [float(value) for value in values[: len(expected_names)]]
            return [0.0] * len(expected_names)

        index_by_name = {name: index for index, name in enumerate(names)}
        result: list[float] = []
        for name in expected_names:
            index = index_by_name.get(name)
            if index is None or index >= len(values):
                return [0.0] * len(expected_names)
            result.append(float(values[index]))
        return result


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = IoJointStateBridge()
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
