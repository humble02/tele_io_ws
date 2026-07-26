#!/usr/bin/env python3
"""Dual WujiHand wave motion demo.

Both hands move together:
  1. wait for feedback from each hand
  2. smoothly home from measured positions to zero
  3. run the same F2-F5 sinusoidal wave pattern as wave_demo.py
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.task import Future
from sensor_msgs.msg import JointState


NUM_JOINTS = 20
JOINTS_PER_FINGER = 4
DEFAULT_WAVE_FREQUENCY = 0.5
DEFAULT_WAVE_AMPLITUDE = 0.8
DEFAULT_FINGER_PHASE_DELAY = 0.25


@dataclass
class HandRuntime:
    name: str
    start: list[float] | None = None
    command: list[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    publisher: object | None = None


def parse_hand_names(raw: str) -> list[str]:
    names = [name.strip().strip("/") for name in raw.split(",") if name.strip().strip("/")]
    return names or ["hand_left", "hand_right"]


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def smoothstep(t: float) -> float:
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def interpolate(start: list[float], target: list[float], ratio: float) -> list[float]:
    t = smoothstep(ratio)
    return [s + (g - s) * t for s, g in zip(start, target)]


class DualWaveDemo(Node):
    def __init__(self):
        super().__init__("dual_wave_demo")

        self.declare_parameter("hand_names", "hand_left,hand_right")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("timeout", 5.0)
        self.declare_parameter("home_duration", 2.0)
        self.declare_parameter("settle_duration", 0.3)
        self.declare_parameter("wave_frequency", DEFAULT_WAVE_FREQUENCY)
        self.declare_parameter("wave_amplitude", DEFAULT_WAVE_AMPLITUDE)
        self.declare_parameter("finger_phase_delay", DEFAULT_FINGER_PHASE_DELAY)
        self.declare_parameter("wave_duration", 0.0)
        self.declare_parameter("return_home", True)

        hand_names = parse_hand_names(self.get_parameter("hand_names").value)
        self.rate = max(1.0, float(self.get_parameter("rate").value))
        self.timeout = max(0.1, float(self.get_parameter("timeout").value))
        self.home_duration = max(0.0, float(self.get_parameter("home_duration").value))
        self.settle_duration = max(0.0, float(self.get_parameter("settle_duration").value))
        self.wave_frequency = max(0.0, float(self.get_parameter("wave_frequency").value))
        self.wave_amplitude = max(0.0, float(self.get_parameter("wave_amplitude").value))
        self.finger_phase_delay = max(
            0.0, float(self.get_parameter("finger_phase_delay").value)
        )
        self.wave_duration = max(0.0, float(self.get_parameter("wave_duration").value))
        self.return_home = bool(self.get_parameter("return_home").value)

        self.zero = [0.0] * NUM_JOINTS

        self.hands = {name: HandRuntime(name=name) for name in hand_names}
        self._state_subscriptions = []
        for name, hand in self.hands.items():
            hand.publisher = self.create_publisher(
                JointState, f"/{name}/joint_commands", qos_profile_sensor_data
            )
            self._state_subscriptions.append(
                self.create_subscription(
                    JointState,
                    f"/{name}/joint_states",
                    lambda msg, hand_name=name: self._on_state(hand_name, msg),
                    qos_profile_sensor_data,
                )
            )

        self.phase = "wait_feedback"
        self.phase_start = self.get_clock().now()
        self.phase_source = self.zero
        self.done_future = Future()
        self.deadline = self.get_clock().now() + Duration(seconds=self.timeout)
        self.timer = self.create_timer(1.0 / self.rate, self._on_timer)

        self.get_logger().info(
            f"Dual wave demo waiting for feedback from {', '.join(self.hands)}."
        )

    def _on_state(self, hand_name: str, msg: JointState) -> None:
        if len(msg.position) < NUM_JOINTS:
            return
        values = list(msg.position[:NUM_JOINTS])
        if not all(math.isfinite(value) for value in values):
            return

        hand = self.hands[hand_name]
        if hand.start is None:
            hand.start = values
            hand.command = list(values)
            self.get_logger().info(f"Received initial feedback from {hand_name}.")

    def _on_timer(self) -> None:
        if self.done_future.done():
            return

        if self.phase == "wait_feedback":
            if all(hand.start is not None for hand in self.hands.values()):
                self._start_phase("home", source_from_commands=False)
            elif self.get_clock().now() >= self.deadline:
                missing = [name for name, hand in self.hands.items() if hand.start is None]
                self.get_logger().error(f"No joint_states before timeout: {', '.join(missing)}")
                self.done_future.set_result(False)
            return

        if self.phase == "home":
            self._publish_interpolated_to(self.zero, self.home_duration)
            if self._phase_complete(self.home_duration):
                self._start_phase("settle")
            return

        if self.phase == "settle":
            self._publish_target(self.zero)
            if self._phase_complete(self.settle_duration):
                self._start_phase("wave")
            return

        if self.phase == "wave":
            self._publish_target(self._wave_command(self._phase_elapsed()))
            if self.wave_duration > 0.0 and self._phase_complete(self.wave_duration):
                if self.return_home:
                    self._start_phase("return_home")
                else:
                    self.get_logger().info("Dual wave demo complete; leaving final wave command.")
                    self.done_future.set_result(True)
            return

        if self.phase == "return_home":
            self._publish_interpolated_to(self.zero, self.home_duration)
            if self._phase_complete(self.home_duration):
                self._publish_target(self.zero)
                self.get_logger().info("Dual wave demo complete; returned to zero.")
                self.done_future.set_result(True)

    def _start_phase(self, phase: str, source_from_commands: bool = True) -> None:
        self.phase = phase
        self.phase_start = self.get_clock().now()
        if source_from_commands:
            # All hands receive the same trajectory after initial homing.
            self.phase_source = list(next(iter(self.hands.values())).command)
        self.get_logger().info(f"Starting phase: {phase}")

    def _phase_elapsed(self) -> float:
        return (self.get_clock().now() - self.phase_start).nanoseconds / 1e9

    def _phase_complete(self, duration: float) -> bool:
        return self._phase_elapsed() >= duration

    def _publish_interpolated_to(self, target: list[float], duration: float) -> None:
        ratio = 1.0 if duration <= 0.0 else self._phase_elapsed() / duration
        if self.phase == "home":
            for hand in self.hands.values():
                source = hand.start if hand.start is not None else self.zero
                self._publish_hand(hand, interpolate(source, target, ratio))
        else:
            command = interpolate(self.phase_source, target, ratio)
            self._publish_target(command)

    def _wave_command(self, elapsed: float) -> list[float]:
        command = [0.0] * NUM_JOINTS

        # F2-F5 wave motion, skipping F1/thumb and each finger's side/abduction joint.
        # finger_phase_delay is expressed in cycles, so 0.25 means each next finger
        # lags by a quarter of a wave period. Set it to 0.0 for synchronized fingers.
        for finger in range(1, 5):
            finger_offset = finger - 1
            phase = 2.0 * math.pi * (
                self.wave_frequency * elapsed - self.finger_phase_delay * finger_offset
            )
            y = (1.0 - math.cos(phase)) * self.wave_amplitude
            base = finger * JOINTS_PER_FINGER
            command[base + 0] = y
            command[base + 2] = y
            command[base + 3] = y
        return command

    def _publish_target(self, target: list[float]) -> None:
        for hand in self.hands.values():
            self._publish_hand(hand, target)

    def _publish_hand(self, hand: HandRuntime, command: list[float]) -> None:
        if hand.publisher is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = list(command)
        hand.command = list(command)
        hand.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualWaveDemo()
    try:
        rclpy.spin_until_future_complete(node, node.done_future)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
