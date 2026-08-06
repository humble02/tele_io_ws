#!/usr/bin/env python3

from __future__ import annotations

from contextlib import suppress
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rcl_interfaces.srv import SetParametersAtomically
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8


LEFT_JOINT_NAMES = [f"Joint{index}_L" for index in range(1, 8)]
RIGHT_JOINT_NAMES = [f"Joint{index}_R" for index in range(1, 8)]
LEFT_HAND_JOINT_NAMES = [
    f"left_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)
]
RIGHT_HAND_JOINT_NAMES = [
    f"right_finger{finger}_joint{joint}" for finger in range(1, 6) for joint in range(1, 5)
]
ARM_JOINT_NAMES = LEFT_JOINT_NAMES + RIGHT_JOINT_NAMES
DEFAULT_IO_COMMAND_JOINT_NAMES = RIGHT_JOINT_NAMES + LEFT_JOINT_NAMES
DEFAULT_IO_STATE_JOINT_NAMES = RIGHT_JOINT_NAMES + LEFT_JOINT_NAMES
LEFT_FROZEN = 0x01
RIGHT_FROZEN = 0x02
MARVIN_LIMIT_RETRY_PERIOD_SEC = 0.5


@dataclass
class JointCache:
    names: list[str]
    position: list[float]
    velocity: list[float]
    effort: list[float]
    has_velocity: bool = False
    has_effort: bool = False
    received: bool = False
    input_names: tuple[str, ...] = ()
    input_indices: tuple[int, ...] = ()
    last_update_ns: int = 0


@dataclass
class SideGate:
    side: str
    frozen: bool = False
    arm_hold: Optional[JointState] = None
    hand_hold: Optional[JointState] = None
    arm_output: Optional[JointState] = None
    hand_output: Optional[JointState] = None
    latest_arm_target: Optional[JointState] = None
    latest_hand_target: Optional[JointState] = None
    arm_resuming: bool = False
    hand_resuming: bool = False
    arm_last_publish_ns: int = 0
    hand_last_publish_ns: int = 0


class IoJointStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("io_joint_state_bridge")

        self.declare_parameter("publish_rate_hz", 1000.0)
        self.declare_parameter("require_both_feedback", True)
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
        self.declare_parameter("left_command_topic", "/marvin/left/joint_commands")
        self.declare_parameter("right_command_topic", "/marvin/right/joint_commands")
        self.declare_parameter("freeze_mask_topic", "/io_teleop/freeze_mask")
        self.declare_parameter("hold_publish_rate_hz", 50.0)
        self.declare_parameter("hold_feedback_timeout_sec", 0.25)
        self.declare_parameter("resume_arm_max_velocity_rad_s", 0.5)
        self.declare_parameter("resume_hand_max_velocity_rad_s", 1.0)
        self.declare_parameter("enable_marvin_limit_promotion", True)
        self.declare_parameter("marvin_driver_node", "/marvin/marvin_driver")
        self.declare_parameter("marvin_limit_promotion_delay_sec", 10.0)
        self.declare_parameter("footswitch_resume_limit_delay_sec", 3.0)
        self.declare_parameter("startup_velocity_ratio", 10)
        self.declare_parameter("startup_acceleration_ratio", 10)
        self.declare_parameter("promoted_velocity_ratio", 100)
        self.declare_parameter("promoted_acceleration_ratio", 100)
        self.declare_parameter("left_hand_state_topic", "/hand_left/joint_states")
        self.declare_parameter("right_hand_state_topic", "/hand_right/joint_states")
        self.declare_parameter("left_hand_command_topic", "/hand_left/joint_commands")
        self.declare_parameter("right_hand_command_topic", "/hand_right/joint_commands")

        self._publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if self._publish_rate_hz <= 0.0:
            raise ValueError(f"publish_rate_hz must be > 0, got {self._publish_rate_hz}")

        self._require_both_feedback = bool(self.get_parameter("require_both_feedback").value)
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
        self._left_command_topic = str(self.get_parameter("left_command_topic").value)
        self._right_command_topic = str(self.get_parameter("right_command_topic").value)
        self._freeze_mask_topic = str(self.get_parameter("freeze_mask_topic").value)
        self._hold_publish_rate_hz = float(
            self.get_parameter("hold_publish_rate_hz").value
        )
        self._hold_feedback_timeout_sec = float(
            self.get_parameter("hold_feedback_timeout_sec").value
        )
        self._resume_arm_max_velocity_rad_s = float(
            self.get_parameter("resume_arm_max_velocity_rad_s").value
        )
        self._resume_hand_max_velocity_rad_s = float(
            self.get_parameter("resume_hand_max_velocity_rad_s").value
        )
        if self._hold_publish_rate_hz <= 0.0:
            raise ValueError("hold_publish_rate_hz must be > 0")
        if self._hold_feedback_timeout_sec <= 0.0:
            raise ValueError("hold_feedback_timeout_sec must be > 0")
        if self._resume_arm_max_velocity_rad_s < 0.0:
            raise ValueError("resume_arm_max_velocity_rad_s must be >= 0")
        if self._resume_hand_max_velocity_rad_s < 0.0:
            raise ValueError("resume_hand_max_velocity_rad_s must be >= 0")
        self._enable_marvin_limit_promotion = bool(
            self.get_parameter("enable_marvin_limit_promotion").value
        )
        self._marvin_driver_node = str(
            self.get_parameter("marvin_driver_node").value
        ).rstrip("/")
        self._marvin_limit_promotion_delay_sec = float(
            self.get_parameter("marvin_limit_promotion_delay_sec").value
        )
        self._footswitch_resume_limit_delay_sec = float(
            self.get_parameter("footswitch_resume_limit_delay_sec").value
        )
        self._startup_velocity_ratio = int(
            self.get_parameter("startup_velocity_ratio").value
        )
        self._startup_acceleration_ratio = int(
            self.get_parameter("startup_acceleration_ratio").value
        )
        self._promoted_velocity_ratio = int(
            self.get_parameter("promoted_velocity_ratio").value
        )
        self._promoted_acceleration_ratio = int(
            self.get_parameter("promoted_acceleration_ratio").value
        )
        if self._enable_marvin_limit_promotion:
            if not self._marvin_driver_node:
                raise ValueError("marvin_driver_node must not be empty")
            if self._marvin_limit_promotion_delay_sec <= 0.0:
                raise ValueError("marvin_limit_promotion_delay_sec must be > 0")
            if self._footswitch_resume_limit_delay_sec <= 0.0:
                raise ValueError("footswitch_resume_limit_delay_sec must be > 0")
            for name, value in (
                ("startup_velocity_ratio", self._startup_velocity_ratio),
                ("startup_acceleration_ratio", self._startup_acceleration_ratio),
                ("promoted_velocity_ratio", self._promoted_velocity_ratio),
                ("promoted_acceleration_ratio", self._promoted_acceleration_ratio),
            ):
                if not 1 <= value <= 100:
                    raise ValueError(f"{name} must be in [1, 100]")
        self._left_hand_state_topic = str(
            self.get_parameter("left_hand_state_topic").value
        )
        self._right_hand_state_topic = str(
            self.get_parameter("right_hand_state_topic").value
        )
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
        self._left_gate = SideGate("left")
        self._right_gate = SideGate("right")
        self._state_caches = (self._left, self._right)
        cache_by_name = {
            name: (cache, index)
            for cache in self._state_caches
            for index, name in enumerate(cache.names)
        }
        self._io_state_layout = [cache_by_name[name] for name in self._io_state_joint_names]
        self._input_layout_cache: dict[
            tuple[tuple[str, ...], tuple[str, ...]], tuple[int, ...]
        ] = {}

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
            lambda msg: self._on_hand_state(
                self._left_hand, self._left_gate, msg, self._left_hand_state_topic
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            self._right_hand_state_topic,
            lambda msg: self._on_hand_state(
                self._right_hand, self._right_gate, msg, self._right_hand_state_topic
            ),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt8,
            self._freeze_mask_topic,
            self._on_freeze_mask,
            10,
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
                self._left_gate,
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
                self._right_gate,
                self._right_hand_command_pub,
                self._io_right_hand_command_topic,
            ),
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / self._publish_rate_hz, self._publish_io_state)
        self.create_timer(1.0 / self._hold_publish_rate_hz, self._publish_holds)
        self._marvin_limit_client = None
        self._marvin_limit_timer = None
        self._marvin_limit_future = None
        self._marvin_limit_future_phase = None
        self._marvin_limit_future_generation = None
        self._marvin_limit_generation = 0
        self._marvin_limit_sequence = None
        self._marvin_limit_delay_sec = 0.0
        self._marvin_limit_phase = "disabled"
        if self._enable_marvin_limit_promotion:
            service_name = f"{self._marvin_driver_node}/set_parameters_atomically"
            self._marvin_limit_client = self.create_client(
                SetParametersAtomically, service_name
            )
            self._start_marvin_limit_sequence(
                "startup", self._marvin_limit_promotion_delay_sec
            )
        self.get_logger().info(
            "io_joint_state_bridge ready: "
            f"{self._left_state_topic}+{self._right_state_topic} "
            f"-> {self._io_state_topic}, "
            f"arm_command_forwarding={self._forward_arm_commands}, "
            f"{self._io_left_hand_command_topic} -> {self._left_hand_command_topic}, "
            f"{self._io_right_hand_command_topic} -> {self._right_hand_command_topic}, "
            f"freeze_mask={self._freeze_mask_topic}, "
            f"rate={self._publish_rate_hz:.1f} Hz"
        )
        if self._enable_marvin_limit_promotion:
            self.get_logger().info(
                "Marvin arm limits will first be reset to "
                f"velocity_ratio={self._startup_velocity_ratio} "
                f"acceleration_ratio={self._startup_acceleration_ratio}; "
                f"the {self._marvin_limit_promotion_delay_sec:.1f} s promotion delay "
                "starts only after that update succeeds"
            )

    def _start_marvin_limit_sequence(self, sequence: str, delay_sec: float) -> None:
        if not self._enable_marvin_limit_promotion:
            return
        if self._marvin_limit_timer is not None:
            self._marvin_limit_timer.cancel()
        self._marvin_limit_generation += 1
        self._marvin_limit_sequence = sequence
        self._marvin_limit_delay_sec = delay_sec
        self._marvin_limit_phase = f"{sequence}_reset"
        self._marvin_limit_timer = self.create_timer(
            MARVIN_LIMIT_RETRY_PERIOD_SEC,
            self._request_marvin_limit_update,
        )
        self._request_marvin_limit_update()

    def _request_marvin_limit_update(self) -> None:
        if self._marvin_limit_future is not None:
            return
        if self._marvin_limit_client is None or not self._marvin_limit_client.service_is_ready():
            self.get_logger().warn(
                f"Waiting for parameter service on {self._marvin_driver_node}; "
                "Marvin arm limits remain unchanged",
                throttle_duration_sec=10.0,
            )
            return

        if self._marvin_limit_phase.endswith("_reset"):
            velocity_ratio = self._startup_velocity_ratio
            acceleration_ratio = self._startup_acceleration_ratio
        elif self._marvin_limit_phase.endswith("_promotion"):
            velocity_ratio = self._promoted_velocity_ratio
            acceleration_ratio = self._promoted_acceleration_ratio
        else:
            return

        request = SetParametersAtomically.Request()
        request.parameters = [
            Parameter("velocity_ratio", value=velocity_ratio).to_parameter_msg(),
            Parameter(
                "acceleration_ratio", value=acceleration_ratio
            ).to_parameter_msg(),
        ]
        self._marvin_limit_future_phase = self._marvin_limit_phase
        self._marvin_limit_future_generation = self._marvin_limit_generation
        self._marvin_limit_future = self._marvin_limit_client.call_async(request)
        self._marvin_limit_future.add_done_callback(
            self._on_marvin_limit_update_response
        )

    def _on_marvin_limit_update_response(self, future) -> None:
        phase = self._marvin_limit_future_phase
        generation = self._marvin_limit_future_generation
        self._marvin_limit_future = None
        self._marvin_limit_future_phase = None
        self._marvin_limit_future_generation = None
        if generation != self._marvin_limit_generation:
            self._request_marvin_limit_update()
            return

        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS futures may raise middleware errors
            self.get_logger().error(
                f"Failed to update Marvin arm limits: {exc}; will retry"
            )
            return

        if response is None or not response.result.successful:
            reason = "no response" if response is None else response.result.reason
            self.get_logger().error(
                f"Marvin driver rejected arm limit update: {reason}; will retry"
            )
            return

        if phase is not None and phase.endswith("_reset"):
            if self._marvin_limit_timer is not None:
                self._marvin_limit_timer.cancel()
            self._marvin_limit_phase = f"{self._marvin_limit_sequence}_promotion"
            self._marvin_limit_timer = self.create_timer(
                self._marvin_limit_delay_sec,
                self._request_marvin_limit_update,
            )
            self.get_logger().info(
                "Marvin arm limits reset successfully: "
                f"velocity_ratio={self._startup_velocity_ratio} "
                f"acceleration_ratio={self._startup_acceleration_ratio}; "
                f"starting {self._marvin_limit_delay_sec:.1f} s "
                f"{self._marvin_limit_sequence} buffer"
            )
            return

        if phase is not None and phase.endswith("_promotion"):
            if self._marvin_limit_timer is not None:
                self._marvin_limit_timer.cancel()
            self._marvin_limit_phase = "done"
            self.get_logger().info(
                "Marvin arm limits promoted successfully: "
                f"velocity_ratio={self._promoted_velocity_ratio} "
                f"acceleration_ratio={self._promoted_acceleration_ratio}"
            )

    def _on_arm_state(self, cache: JointCache, msg: JointState, source: str) -> None:
        indices = self._input_indices(cache, msg, source)
        if indices is None:
            return

        positions = self._extract_values(msg.position, indices)
        if positions is None:
            self.get_logger().warn(
                f"Ignoring JointState from {source}: position array is too short",
                throttle_duration_sec=1.0,
            )
            return

        cache.position = positions
        velocity = self._extract_values(msg.velocity, indices) if msg.velocity else None
        effort = self._extract_values(msg.effort, indices) if msg.effort else None
        cache.has_velocity = velocity is not None
        cache.has_effort = effort is not None
        if velocity is not None:
            cache.velocity = velocity
        if effort is not None:
            cache.effort = effort
        cache.received = True
        cache.last_update_ns = self.get_clock().now().nanoseconds

        gate = self._left_gate if cache is self._left else self._right_gate
        publisher = (
            self._left_command_pub if cache is self._left else self._right_command_pub
        )
        if gate.frozen and gate.arm_hold is None and publisher is not None:
            gate.arm_hold = self._command_from_cache(cache)
            self._publish_gate_output(gate, "arm", gate.arm_hold, publisher)

    def _on_hand_state(
        self,
        cache: JointCache,
        gate: SideGate,
        msg: JointState,
        source: str,
    ) -> None:
        indices = self._input_indices(cache, msg, source)
        if indices is None:
            return
        positions = self._extract_values(msg.position, indices)
        if positions is None:
            self.get_logger().warn(
                f"Ignoring JointState from {source}: position array is too short",
                throttle_duration_sec=1.0,
            )
            return

        cache.position = positions
        velocity = self._extract_values(msg.velocity, indices) if msg.velocity else None
        effort = self._extract_values(msg.effort, indices) if msg.effort else None
        cache.has_velocity = velocity is not None
        cache.has_effort = effort is not None
        if velocity is not None:
            cache.velocity = velocity
        if effort is not None:
            cache.effort = effort
        cache.received = True
        cache.last_update_ns = self.get_clock().now().nanoseconds

        publisher = (
            self._left_hand_command_pub
            if gate is self._left_gate
            else self._right_hand_command_pub
        )
        if gate.frozen and gate.hand_hold is None:
            gate.hand_hold = self._command_from_cache(cache)
            self._publish_gate_output(gate, "hand", gate.hand_hold, publisher)

    def _publish_io_state(self) -> None:
        if self._require_both_feedback and (not self._left.received or not self._right.received):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._io_state_joint_names)
        msg.position = [cache.position[index] for cache, index in self._io_state_layout]
        if all(cache.has_velocity for cache in self._state_caches):
            msg.velocity = [cache.velocity[index] for cache, index in self._io_state_layout]
        if all(cache.has_effort for cache in self._state_caches):
            msg.effort = [cache.effort[index] for cache, index in self._io_state_layout]

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
        self._forward_gated_command(
            self._left_gate,
            "arm",
            left_command,
            self._left_command_pub,
            self._resume_arm_max_velocity_rad_s,
        )
        self._forward_gated_command(
            self._right_gate,
            "arm",
            right_command,
            self._right_command_pub,
            self._resume_arm_max_velocity_rad_s,
        )

    def _on_io_hand_command(
        self,
        msg: JointState,
        joint_names: list[str],
        gate: SideGate,
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
        self._forward_gated_command(
            gate,
            "hand",
            out,
            publisher,
            self._resume_hand_max_velocity_rad_s,
        )

    def _on_freeze_mask(self, msg: UInt8) -> None:
        was_frozen = self._left_gate.frozen or self._right_gate.frozen
        mask = int(msg.data) & (LEFT_FROZEN | RIGHT_FROZEN)
        if int(msg.data) != mask:
            self.get_logger().warn(
                f"Ignoring unsupported freeze-mask bits in 0x{int(msg.data):02x}",
                throttle_duration_sec=1.0,
            )
        if was_frozen and mask == 0:
            self._start_marvin_limit_sequence(
                "footswitch_resume", self._footswitch_resume_limit_delay_sec
            )
        self._set_side_frozen(
            self._left_gate,
            bool(mask & LEFT_FROZEN),
            self._left,
            self._left_hand,
            self._left_command_pub,
            self._left_hand_command_pub,
        )
        self._set_side_frozen(
            self._right_gate,
            bool(mask & RIGHT_FROZEN),
            self._right,
            self._right_hand,
            self._right_command_pub,
            self._right_hand_command_pub,
        )

    def _set_side_frozen(
        self,
        gate: SideGate,
        frozen: bool,
        arm_cache: JointCache,
        hand_cache: JointCache,
        arm_publisher: Optional[rclpy.publisher.Publisher],
        hand_publisher: rclpy.publisher.Publisher,
    ) -> None:
        if frozen == gate.frozen:
            return

        gate.frozen = frozen
        if frozen:
            gate.arm_resuming = False
            gate.hand_resuming = False
            gate.arm_hold = self._snapshot_hold(
                arm_cache, gate.arm_output, f"{gate.side} arm"
            )
            gate.hand_hold = self._snapshot_hold(
                hand_cache, gate.hand_output, f"{gate.side} hand"
            )
            if gate.arm_hold is not None and arm_publisher is not None:
                self._publish_gate_output(gate, "arm", gate.arm_hold, arm_publisher)
            if gate.hand_hold is not None:
                self._publish_gate_output(gate, "hand", gate.hand_hold, hand_publisher)
            self.get_logger().warning(f"{gate.side} arm and hand frozen")
            return

        gate.arm_resuming = gate.arm_output is not None
        gate.hand_resuming = gate.hand_output is not None
        self.get_logger().info(
            f"{gate.side} arm and hand released; resuming IO commands with rate limiting"
        )

    def _snapshot_hold(
        self,
        cache: JointCache,
        last_output: Optional[JointState],
        label: str,
    ) -> Optional[JointState]:
        # Keep the controller's last position setpoint unchanged. Replacing it with
        # measured feedback would remove the position error that may be balancing
        # gravity/load and can therefore cause a small motion at freeze time.
        if last_output is not None:
            return self._copy_command(last_output, zero_dynamics=True)

        now_ns = self.get_clock().now().nanoseconds
        feedback_age_sec = (
            (now_ns - cache.last_update_ns) / 1e9 if cache.last_update_ns else float("inf")
        )
        if cache.received and feedback_age_sec <= self._hold_feedback_timeout_sec:
            self.get_logger().warn(
                f"{label} has no previous output target; holding measured feedback as fallback"
            )
            return self._command_from_cache(cache)

        self.get_logger().error(
            f"Cannot create a {label} hold command: no feedback or previous output; "
            "new IO commands will still be blocked"
        )
        return None

    def _forward_gated_command(
        self,
        gate: SideGate,
        kind: str,
        target: JointState,
        publisher: rclpy.publisher.Publisher,
        max_velocity_rad_s: float,
    ) -> None:
        setattr(gate, f"latest_{kind}_target", self._copy_command(target))
        if gate.frozen:
            return

        output = target
        if getattr(gate, f"{kind}_resuming"):
            current = getattr(gate, f"{kind}_output")
            last_publish_ns = getattr(gate, f"{kind}_last_publish_ns")
            output, complete = self._rate_limit_command(
                current,
                target,
                last_publish_ns,
                max_velocity_rad_s,
            )
            setattr(gate, f"{kind}_resuming", not complete)
            if complete:
                setattr(gate, f"{kind}_hold", None)

        self._publish_gate_output(gate, kind, output, publisher)

    def _publish_holds(self) -> None:
        for gate, arm_publisher, hand_publisher in (
            (
                self._left_gate,
                self._left_command_pub,
                self._left_hand_command_pub,
            ),
            (
                self._right_gate,
                self._right_command_pub,
                self._right_hand_command_pub,
            ),
        ):
            if not gate.frozen:
                continue
            if gate.arm_hold is not None and arm_publisher is not None:
                self._publish_gate_output(gate, "arm", gate.arm_hold, arm_publisher)
            if gate.hand_hold is not None:
                self._publish_gate_output(gate, "hand", gate.hand_hold, hand_publisher)

    def _publish_gate_output(
        self,
        gate: SideGate,
        kind: str,
        command: JointState,
        publisher: rclpy.publisher.Publisher,
    ) -> None:
        out = self._copy_command(command)
        out.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(out)
        setattr(gate, f"{kind}_output", self._copy_command(out))
        setattr(
            gate,
            f"{kind}_last_publish_ns",
            self.get_clock().now().nanoseconds,
        )

    def _rate_limit_command(
        self,
        current: Optional[JointState],
        target: JointState,
        last_publish_ns: int,
        max_velocity_rad_s: float,
    ) -> tuple[JointState, bool]:
        if (
            current is None
            or max_velocity_rad_s <= 0.0
            or len(current.position) != len(target.position)
        ):
            return self._copy_command(target), True

        now_ns = self.get_clock().now().nanoseconds
        elapsed_sec = (now_ns - last_publish_ns) / 1e9 if last_publish_ns else 0.0
        elapsed_sec = min(max(elapsed_sec, 1e-4), 0.1)
        max_step = max_velocity_rad_s * elapsed_sec
        complete = True
        positions: list[float] = []
        for current_value, target_value in zip(current.position, target.position):
            delta = float(target_value) - float(current_value)
            if abs(delta) <= max_step:
                positions.append(float(target_value))
            else:
                complete = False
                positions.append(
                    float(current_value) + (max_step if delta > 0.0 else -max_step)
                )

        out = self._copy_command(target)
        out.position = positions
        if not complete:
            out.velocity = [0.0] * len(positions)
            out.effort = [0.0] * len(positions)
        return out, complete

    def _command_from_cache(self, cache: JointCache) -> JointState:
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(cache.names)
        out.position = list(cache.position)
        out.velocity = [0.0] * len(cache.names)
        out.effort = [0.0] * len(cache.names)
        return out

    @staticmethod
    def _copy_command(msg: JointState, zero_dynamics: bool = False) -> JointState:
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.name = list(msg.name)
        out.position = [float(value) for value in msg.position]
        if zero_dynamics:
            out.velocity = [0.0] * len(out.position)
            out.effort = [0.0] * len(out.position)
        else:
            out.velocity = [float(value) for value in msg.velocity]
            out.effort = [float(value) for value in msg.effort]
        return out

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
        if len(joint_names) != len(ARM_JOINT_NAMES):
            raise ValueError(
                f"io_state_joint_names must contain 14 arm joints, got {len(joint_names)}"
            )
        if set(joint_names) != set(ARM_JOINT_NAMES):
            raise ValueError(
                "io_state_joint_names must contain exactly "
                "Joint1_L..Joint7_L and Joint1_R..Joint7_R"
            )

    def _input_indices(
        self, cache: JointCache, msg: JointState, source: str
    ) -> Optional[tuple[int, ...]]:
        if not msg.name:
            if len(msg.position) < len(cache.names):
                self.get_logger().warn(
                    f"Ignoring unnamed JointState from {source}: expected "
                    f"{len(cache.names)} positions, got {len(msg.position)}",
                    throttle_duration_sec=1.0,
                )
                return None
            if len(cache.input_indices) != len(cache.names):
                cache.input_names = ()
                cache.input_indices = tuple(range(len(cache.names)))
            return cache.input_indices

        input_names = tuple(msg.name)
        if input_names == cache.input_names:
            return cache.input_indices

        indices = self._indices_from_names(input_names, cache.names)
        if indices is None:
            missing_name = next(name for name in cache.names if name not in input_names)
            self.get_logger().warn(
                f"Ignoring JointState from {source}: missing joint '{missing_name}'",
                throttle_duration_sec=1.0,
            )
            return None

        cache.input_names = input_names
        cache.input_indices = indices
        return indices

    @staticmethod
    def _extract_values(
        values: List[float], indices: tuple[int, ...]
    ) -> Optional[list[float]]:
        if not values or any(index >= len(values) for index in indices):
            return None
        return [float(values[index]) for index in indices]

    def _indices_from_names(
        self, input_names: tuple[str, ...], expected_names: list[str]
    ) -> Optional[tuple[int, ...]]:
        key = input_names, tuple(expected_names)
        cached = self._input_layout_cache.get(key)
        if cached is not None:
            return cached

        index_by_name = {name: index for index, name in enumerate(input_names)}
        indices = tuple(index_by_name.get(name, -1) for name in expected_names)
        if any(index < 0 for index in indices):
            return None
        if len(self._input_layout_cache) >= 16:
            self._input_layout_cache.clear()
        self._input_layout_cache[key] = indices
        return indices

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

        input_names = tuple(msg.name)
        indices = self._indices_from_names(input_names, expected_names)
        if indices is None:
            missing_name = next(name for name in expected_names if name not in input_names)
            self.get_logger().warn(
                f"Ignoring JointState from {source}: missing joint '{missing_name}'",
                throttle_duration_sec=1.0,
            )
            return None
        values = self._extract_values(msg.position, indices)
        if values is None:
            self.get_logger().warn(
                f"Ignoring JointState from {source}: position array is too short",
                throttle_duration_sec=1.0,
            )
        return values

    def _extract_optional_by_names(
        self,
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

        indices = self._indices_from_names(tuple(names), expected_names)
        if indices is None:
            return [0.0] * len(expected_names)
        return self._extract_values(values, indices) or [0.0] * len(expected_names)


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
