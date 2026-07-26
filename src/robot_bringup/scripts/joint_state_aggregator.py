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


DEFAULT_JOINT_NAMES = [
    "Joint1_L",
    "Joint2_L",
    "Joint3_L",
    "Joint4_L",
    "Joint5_L",
    "Joint6_L",
    "Joint7_L",
    "Joint1_R",
    "Joint2_R",
    "Joint3_R",
    "Joint4_R",
    "Joint5_R",
    "Joint6_R",
    "Joint7_R",
    "left_finger1_joint1",
    "left_finger1_joint2",
    "left_finger1_joint3",
    "left_finger1_joint4",
    "left_finger2_joint1",
    "left_finger2_joint2",
    "left_finger2_joint3",
    "left_finger2_joint4",
    "left_finger3_joint1",
    "left_finger3_joint2",
    "left_finger3_joint3",
    "left_finger3_joint4",
    "left_finger4_joint1",
    "left_finger4_joint2",
    "left_finger4_joint3",
    "left_finger4_joint4",
    "left_finger5_joint1",
    "left_finger5_joint2",
    "left_finger5_joint3",
    "left_finger5_joint4",
    "right_finger1_joint1",
    "right_finger1_joint2",
    "right_finger1_joint3",
    "right_finger1_joint4",
    "right_finger2_joint1",
    "right_finger2_joint2",
    "right_finger2_joint3",
    "right_finger2_joint4",
    "right_finger3_joint1",
    "right_finger3_joint2",
    "right_finger3_joint3",
    "right_finger3_joint4",
    "right_finger4_joint1",
    "right_finger4_joint2",
    "right_finger4_joint3",
    "right_finger4_joint4",
    "right_finger5_joint1",
    "right_finger5_joint2",
    "right_finger5_joint3",
    "right_finger5_joint4",
]

DEFAULT_INPUT_TOPICS = [
    "/marvin/left/joint_states",
    "/marvin/right/joint_states",
    "/hand_left/joint_states",
    "/hand_right/joint_states",
]


@dataclass
class JointValue:
    position: float = 0.0
    velocity: float = 0.0
    effort: float = 0.0
    has_velocity: bool = False
    has_effort: bool = False
    last_update_sec: Optional[float] = None


class JointStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_aggregator")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("stale_timeout_sec", 0.0)
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("input_topics", DEFAULT_INPUT_TOPICS)

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._input_topics: List[str] = list(
            self.get_parameter("input_topics").get_parameter_value().string_array_value
        )
        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)

        if not self._joint_names:
            raise RuntimeError("joint_names parameter must not be empty")

        self._joint_values: Dict[str, JointValue] = {
            name: JointValue() for name in self._joint_names
        }
        self._joint_set = set(self._joint_names)

        self._pub = self.create_publisher(JointState, "/joint_states", qos_profile_sensor_data)
        self._subs = [
            self.create_subscription(
                JointState,
                topic,
                lambda msg, source=topic: self._on_joint_state(msg, source),
                qos_profile_sensor_data,
            )
            for topic in self._input_topics
        ]

        period = 1.0 / max(self._publish_rate_hz, 1.0)
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f"Aggregating {len(self._joint_names)} joints from {len(self._input_topics)} topics."
        )

    def _on_joint_state(self, msg: JointState, source: str) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if not msg.name:
            self.get_logger().warn(
                f"Ignoring unnamed JointState from {source}; names are required for aggregation.",
                throttle_duration_sec=2.0,
            )
            return

        for index, name in enumerate(msg.name):
            if name not in self._joint_set:
                continue
            if index >= len(msg.position):
                continue

            value = self._joint_values[name]
            value.position = float(msg.position[index])
            value.has_velocity = index < len(msg.velocity)
            value.has_effort = index < len(msg.effort)
            value.velocity = float(msg.velocity[index]) if value.has_velocity else 0.0
            value.effort = float(msg.effort[index]) if value.has_effort else 0.0
            value.last_update_sec = now_sec

    def _publish(self) -> None:
        if not rclpy.ok():
            return

        now = self.get_clock().now()
        now_sec = now.nanoseconds * 1e-9

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self._joint_names)
        msg.position = []
        msg.velocity = []
        msg.effort = []

        include_velocity = True
        include_effort = True
        for name in self._joint_names:
            value = self._joint_values[name]
            if self._is_stale(value, now_sec):
                msg.position.append(0.0)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
            else:
                msg.position.append(value.position)
                msg.velocity.append(value.velocity)
                msg.effort.append(value.effort)
                include_velocity = include_velocity and value.has_velocity
                include_effort = include_effort and value.has_effort

        if not include_velocity:
            msg.velocity = []
        if not include_effort:
            msg.effort = []

        try:
            self._pub.publish(msg)
        except RCLError:
            if rclpy.ok():
                raise

    def _is_stale(self, value: JointValue, now_sec: float) -> bool:
        if value.last_update_sec is None:
            return False
        if self._stale_timeout_sec <= 0.0:
            return False
        return now_sec - value.last_update_sec > self._stale_timeout_sec


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JointStateAggregator()
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
