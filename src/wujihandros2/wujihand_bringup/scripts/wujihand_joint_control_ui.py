#!/usr/bin/env python3
"""Tkinter live joint command UI for WujiHand ROS 2 driver."""

from __future__ import annotations

import math
import signal
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

try:
    import tkinter as tk
    from tkinter import font as tkfont
    from tkinter import ttk
except ImportError as exc:
    print("tkinter is not available. Install python3-tk for this Python.", file=sys.stderr)
    raise SystemExit(1) from exc

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


NUM_FINGERS = 5
JOINTS_PER_FINGER = 4
NUM_JOINTS = NUM_FINGERS * JOINTS_PER_FINGER
SIDES = ("left", "right")

# Fallback copy of the current URDF limits. Runtime loads wuji_description URDFs first.
DEFAULT_LIMITS_RAD = [
    (0.0475, 1.6033),
    (-0.1387, 0.9324),
    (-0.4642, 1.5623),
    (-0.4699, 1.5568),
    (-0.1585, 1.5604),
    (-0.3700, 0.3700),
    (-0.4777, 1.5485),
    (-0.4683, 1.5753),
    (-0.1644, 1.5516),
    (-0.3700, 0.3700),
    (-0.4739, 1.5512),
    (-0.4684, 1.5745),
    (-0.1554, 1.5585),
    (-0.3700, 0.3700),
    (-0.4765, 1.5487),
    (-0.4777, 1.5634),
    (-0.1626, 1.5585),
    (-0.3700, 0.3700),
    (-0.4768, 1.5490),
    (-0.4683, 1.5735),
]


def normalize_hand_names(value: str) -> list[str]:
    names = [name.strip().strip("/") for name in value.split(",") if name.strip().strip("/")]
    return names or ["hand_0"]


def hand_namespace(hand_name: str) -> str:
    text = hand_name.strip().strip("/")
    return f"/{text}" if text else "/hand_0"


def infer_side_from_hand_name(hand_name: str) -> str | None:
    text = hand_name.lower().strip("/")
    if text.endswith("_left") or text in ("left", "hand_left"):
        return "left"
    if text.endswith("_right") or text in ("right", "hand_right"):
        return "right"
    return None


def infer_side_from_joint_names(names: list[str]) -> str | None:
    for name in names:
        if name.startswith("left_finger"):
            return "left"
        if name.startswith("right_finger"):
            return "right"
    return None


def default_joint_names(side: str) -> list[str]:
    return [
        f"{side}_finger{finger}_joint{joint}"
        for finger in range(1, NUM_FINGERS + 1)
        for joint in range(1, JOINTS_PER_FINGER + 1)
    ]


def short_joint_name(index: int) -> str:
    finger = index // JOINTS_PER_FINGER + 1
    joint = index % JOINTS_PER_FINGER + 1
    return f"F{finger} J{joint}"


def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def rad_to_deg(value: float) -> float:
    return value * 180.0 / math.pi


def clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def load_urdf_limits(side: str) -> dict[str, tuple[float, float]]:
    try:
        description_dir = Path(get_package_share_directory("wuji_description"))
        urdf_file = description_dir / "urdf" / f"{side}-ros.urdf"
        root = ET.parse(urdf_file).getroot()
    except (PackageNotFoundError, OSError, ET.ParseError):
        return {
            name: DEFAULT_LIMITS_RAD[i]
            for i, name in enumerate(default_joint_names(side))
        }

    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        limit = joint.find("limit")
        if limit is None or not name.startswith(f"{side}_finger"):
            continue
        try:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])
        except (KeyError, ValueError):
            continue
        if math.isfinite(lower) and math.isfinite(upper) and lower < upper:
            limits[name] = (lower, upper)
    return limits


@dataclass
class HandState:
    hand_name: str
    namespace: str
    preferred_side: str | None
    publisher: object | None = None
    feedback_received: bool = False
    side: str | None = None
    joint_names: list[str] = field(default_factory=list)
    current_rad: list[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    command_rad: list[float] = field(default_factory=lambda: [0.0] * NUM_JOINTS)
    limits_rad: list[tuple[float, float]] = field(default_factory=list)


class WujiHandJointControlNode(Node):
    """ROS side of the live slider UI."""

    def __init__(self) -> None:
        super().__init__("wujihand_joint_control_ui")

        self.declare_parameter("hand_names", "hand_0")
        self.declare_parameter("live_rate_hz", 50.0)
        self.declare_parameter("require_feedback", True)
        self.declare_parameter("ui_scale", 1.4)
        self.declare_parameter("window_width", 1320)
        self.declare_parameter("window_height", 820)

        hand_names = normalize_hand_names(str(self.get_parameter("hand_names").value))
        self.live_rate_hz = max(1.0, float(self.get_parameter("live_rate_hz").value))
        self.require_feedback = bool(self.get_parameter("require_feedback").value)
        self.ui_scale = max(0.75, float(self.get_parameter("ui_scale").value))
        self.window_width = max(900, int(self.get_parameter("window_width").value))
        self.window_height = max(640, int(self.get_parameter("window_height").value))

        self.feedback_callback: Callable[[str, bool], None] | None = None
        self.publish_callback: Callable[[str], None] | None = None
        self.hands: dict[str, HandState] = {}
        self._subscriptions = []

        for hand_name in hand_names:
            namespace = hand_namespace(hand_name)
            state = HandState(
                hand_name=hand_name,
                namespace=namespace,
                preferred_side=infer_side_from_hand_name(hand_name),
            )
            command_topic = f"{namespace}/joint_commands"
            feedback_topic = f"{namespace}/joint_states"
            state.publisher = self.create_publisher(JointState, command_topic, qos_profile_sensor_data)
            self._subscriptions.append(
                self.create_subscription(
                    JointState,
                    feedback_topic,
                    lambda msg, key=hand_name: self._handle_feedback(key, msg),
                    qos_profile_sensor_data,
                )
            )
            self.hands[hand_name] = state
            self.get_logger().info(
                f"UI for {hand_name}: subscribing {feedback_topic}, publishing {command_topic}"
            )

    def _handle_feedback(self, hand_name: str, msg: JointState) -> None:
        state = self.hands[hand_name]
        first_feedback = not state.feedback_received

        side = infer_side_from_joint_names(list(msg.name)) or state.preferred_side
        if side not in SIDES:
            if first_feedback:
                self.get_logger().warn(
                    f"{hand_name}: cannot infer handedness from first joint_states yet."
                )
            return

        joint_names = default_joint_names(side)
        values = self._positions_for_joint_names(joint_names, msg)
        if values is None:
            return

        if first_feedback:
            limits_by_name = load_urdf_limits(side)
            state.side = side
            state.joint_names = joint_names
            state.limits_rad = [
                limits_by_name.get(name, DEFAULT_LIMITS_RAD[i])
                for i, name in enumerate(joint_names)
            ]
            state.command_rad = [
                clamp(values[i], state.limits_rad[i][0], state.limits_rad[i][1])
                for i in range(NUM_JOINTS)
            ]
            state.feedback_received = True
            self.get_logger().info(f"{hand_name}: received initial {side} hand feedback.")

        state.current_rad = values
        if self.feedback_callback is not None:
            self.feedback_callback(hand_name, first_feedback)

    def _positions_for_joint_names(
        self, joint_names: list[str], msg: JointState
    ) -> list[float] | None:
        if msg.name:
            if len(msg.position) < len(msg.name):
                return None
            by_name = {name: msg.position[i] for i, name in enumerate(msg.name)}
            try:
                values = [by_name[name] for name in joint_names]
            except KeyError:
                return None
        else:
            if len(msg.position) < NUM_JOINTS:
                return None
            values = list(msg.position[:NUM_JOINTS])

        return values if all(math.isfinite(value) for value in values) else None

    def publish_hand_command(self, hand_name: str) -> bool:
        state = self.hands[hand_name]
        if state.publisher is None:
            return False
        if self.require_feedback and not state.feedback_received:
            return False
        if len(state.joint_names) != NUM_JOINTS or len(state.command_rad) != NUM_JOINTS:
            return False
        if not all(math.isfinite(value) for value in state.command_rad):
            return False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(state.joint_names)
        msg.position = list(state.command_rad)
        state.publisher.publish(msg)
        if self.publish_callback is not None:
            self.publish_callback(hand_name)
        return True


class HandPanel:
    """Tk widgets for one hand namespace."""

    def __init__(
        self,
        parent: tk.Widget,
        node: WujiHandJointControlNode,
        hand_name: str,
        column: int,
    ) -> None:
        self.node = node
        self.hand_name = hand_name
        self._updating = False

        self.frame = ttk.LabelFrame(parent, text=f"{hand_name} - waiting for feedback", padding=(10, 8))
        self.frame.grid(row=0, column=column, padx=8, pady=8, sticky="nsew")
        self.frame.columnconfigure(1, weight=1)

        ttk.Label(self.frame, text="Joint", width=9).grid(row=0, column=0, sticky="w")
        ttk.Label(self.frame, text="Command", anchor="center").grid(row=0, column=1, sticky="ew")
        ttk.Label(self.frame, text="Cmd deg", width=10).grid(row=0, column=2, sticky="e")
        ttk.Label(self.frame, text="Fb deg", width=10).grid(row=0, column=3, sticky="e")

        self.slider_vars = [tk.DoubleVar(value=0.0) for _ in range(NUM_JOINTS)]
        self.command_vars = [tk.StringVar(value="--") for _ in range(NUM_JOINTS)]
        self.feedback_vars = [tk.StringVar(value="--") for _ in range(NUM_JOINTS)]
        self.limit_vars = [tk.StringVar(value="--") for _ in range(NUM_JOINTS)]
        self.sliders: list[ttk.Scale] = []

        for i in range(NUM_JOINTS):
            row = i + 1
            ttk.Label(self.frame, text=short_joint_name(i)).grid(row=row, column=0, pady=2, sticky="w")
            slider = ttk.Scale(
                self.frame,
                from_=0.0,
                to=1.0,
                variable=self.slider_vars[i],
                command=lambda _value, index=i: self._on_slider_changed(index),
            )
            slider.grid(row=row, column=1, padx=8, pady=2, sticky="ew")
            slider.state(["disabled"])
            self.sliders.append(slider)
            ttk.Label(self.frame, textvariable=self.command_vars[i], width=10).grid(
                row=row, column=2, sticky="e"
            )
            ttk.Label(self.frame, textvariable=self.feedback_vars[i], width=10).grid(
                row=row, column=3, sticky="e"
            )

        ttk.Label(self.frame, text="Limits from URDF, values shown in degrees.").grid(
            row=NUM_JOINTS + 1, column=0, columnspan=4, pady=(8, 0), sticky="w"
        )

    def configure_from_state(self) -> None:
        state = self.node.hands[self.hand_name]
        if not state.feedback_received:
            return

        label = f"{self.hand_name} - {state.side} hand"
        self.frame.configure(text=label)
        self._updating = True
        try:
            for i, slider in enumerate(self.sliders):
                lower_deg = rad_to_deg(state.limits_rad[i][0])
                upper_deg = rad_to_deg(state.limits_rad[i][1])
                slider.configure(from_=lower_deg, to=upper_deg)
                slider.state(["!disabled"])
                self.slider_vars[i].set(rad_to_deg(state.command_rad[i]))
                self.command_vars[i].set(f"{rad_to_deg(state.command_rad[i]):.2f}")
                self.feedback_vars[i].set(f"{rad_to_deg(state.current_rad[i]):.2f}")
        finally:
            self._updating = False

    def update_feedback(self) -> None:
        state = self.node.hands[self.hand_name]
        if not state.feedback_received:
            return
        for i, value in enumerate(state.current_rad):
            self.feedback_vars[i].set(f"{rad_to_deg(value):.2f}")

    def sync_command_from_feedback(self) -> None:
        state = self.node.hands[self.hand_name]
        if not state.feedback_received:
            return
        self._updating = True
        try:
            for i, value in enumerate(state.current_rad):
                lower, upper = state.limits_rad[i]
                command = clamp(value, lower, upper)
                state.command_rad[i] = command
                self.slider_vars[i].set(rad_to_deg(command))
                self.command_vars[i].set(f"{rad_to_deg(command):.2f}")
        finally:
            self._updating = False

    def _on_slider_changed(self, index: int) -> None:
        if self._updating:
            return
        state = self.node.hands[self.hand_name]
        if not state.feedback_received:
            return
        command = deg_to_rad(self.slider_vars[index].get())
        lower, upper = state.limits_rad[index]
        command = clamp(command, lower, upper)
        state.command_rad[index] = command
        self.command_vars[index].set(f"{rad_to_deg(command):.2f}")


class WujiHandJointControlApp:
    """Tkinter app that owns live slider state and publishing cadence."""

    def __init__(self, node: WujiHandJointControlNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self._apply_ui_scale()
        self.root.title("WujiHand Live Joint Control")
        self.root.geometry(f"{self.node.window_width}x{self.node.window_height}")
        self.root.minsize(900, 640)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        self.live_enabled = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="Waiting for joint feedback.")
        self._closed = False
        self._publish_count = 0

        self.node.feedback_callback = self.on_feedback
        self.node.publish_callback = self.on_publish

        self._build_ui()
        self._schedule_ros_spin()
        self._schedule_live_publish()

    def _apply_ui_scale(self) -> None:
        self.root.tk.call("tk", "scaling", self.node.ui_scale)
        for name in ("TkDefaultFont", "TkTextFont", "TkMenuFont", "TkHeadingFont", "TkCaptionFont"):
            try:
                font = tkfont.nametofont(name)
            except tk.TclError:
                continue
            font.configure(size=max(11, int(font.cget("size") * 1.1)))

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=1)

        top = ttk.Frame(self.root, padding=(12, 10, 12, 4))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(3, weight=1)

        self.live_check = ttk.Checkbutton(
            top,
            text=f"Enable live publish ({self.node.live_rate_hz:.0f} Hz)",
            variable=self.live_enabled,
            command=self.on_live_toggle,
        )
        self.live_check.grid(row=0, column=0, padx=(0, 10), sticky="w")
        ttk.Button(top, text="Sync command from feedback", command=self.sync_all_from_feedback).grid(
            row=0, column=1, padx=(0, 10), sticky="w"
        )
        ttk.Label(top, textvariable=self.status).grid(row=0, column=3, sticky="w")

        panel_container = ttk.Frame(self.root, padding=(4, 0, 4, 8))
        panel_container.grid(row=1, column=0, sticky="nsew")
        for col in range(max(1, len(self.node.hands))):
            panel_container.columnconfigure(col, weight=1)

        self.panels: dict[str, HandPanel] = {}
        for col, hand_name in enumerate(self.node.hands):
            self.panels[hand_name] = HandPanel(panel_container, self.node, hand_name, col)

    def on_feedback(self, hand_name: str, first_feedback: bool) -> None:
        panel = self.panels[hand_name]
        if first_feedback:
            panel.configure_from_state()
            self.status.set(f"{hand_name}: initial feedback loaded.")
        else:
            panel.update_feedback()

    def on_publish(self, hand_name: str) -> None:
        self._publish_count += 1
        if self._publish_count % max(1, int(self.node.live_rate_hz)) == 0:
            self.status.set(f"Publishing live commands; last: {hand_name}.")

    def on_live_toggle(self) -> None:
        if self.live_enabled.get() and not self._any_hand_ready():
            self.live_enabled.set(False)
            self.status.set("Live publish needs feedback first.")
            return
        self.status.set(
            f"Live publish enabled at {self.node.live_rate_hz:.0f} Hz."
            if self.live_enabled.get()
            else "Live publish off."
        )

    def sync_all_from_feedback(self) -> None:
        synced = 0
        for hand_name, panel in self.panels.items():
            if self.node.hands[hand_name].feedback_received:
                panel.sync_command_from_feedback()
                synced += 1
        self.status.set(
            f"Synced command from feedback for {synced} hand(s)."
            if synced
            else "No feedback to sync yet."
        )

    def _any_hand_ready(self) -> bool:
        return any(hand.feedback_received for hand in self.node.hands.values())

    def _schedule_ros_spin(self) -> None:
        if self._closed or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            if rclpy.ok():
                raise
            self.close()
            return
        self.root.after(10, self._schedule_ros_spin)

    def _schedule_live_publish(self) -> None:
        if self._closed:
            return
        if self.live_enabled.get():
            published = 0
            for hand_name in self.node.hands:
                if self.node.publish_hand_command(hand_name):
                    published += 1
            if published == 0:
                self.live_enabled.set(False)
                self.status.set("No ready hand to publish; live publish disabled.")
        period_ms = max(5, int(1000.0 / self.node.live_rate_hz))
        self.root.after(period_ms, self._schedule_live_publish)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.live_enabled.set(False)
        self.root.destroy()

    def _handle_signal(self, _signum: int, _frame: object) -> None:
        if not self._closed:
            self.root.after(0, self.close)

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    rclpy.init()
    node = WujiHandJointControlNode()
    try:
        app = WujiHandJointControlApp(node)
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
