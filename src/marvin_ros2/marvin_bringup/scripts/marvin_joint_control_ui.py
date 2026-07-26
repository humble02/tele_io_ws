#!/usr/bin/env python3
"""Tkinter joint command UI for the Marvin ROS 2 driver."""

from __future__ import annotations

import math
import signal
import sys
from dataclasses import dataclass, field
from typing import Callable

try:
    import tkinter as tk
    from tkinter import font as tkfont
    from tkinter import messagebox, ttk
except ImportError as exc:
    print("tkinter is not available. Install python3-tk for this Python.", file=sys.stderr)
    raise SystemExit(1) from exc

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


JOINT_COUNT = 7
ARM_ORDER = ("left", "right")


def default_joint_names(suffix: str) -> list[str]:
    return [f"Joint{i}_{suffix}" for i in range(1, JOINT_COUNT + 1)]


def normalize_arms(value: str) -> set[str]:
    text = value.strip().lower()
    if text in ("both", "all", "ab"):
        return {"left", "right"}
    if text in ("left", "a"):
        return {"left"}
    if text in ("right", "b"):
        return {"right"}
    return {"left", "right"}


def normalize_namespace(value: str) -> str:
    text = value.strip().strip("/")
    return f"/{text}" if text else ""


def deg_to_rad(value: float) -> float:
    return value * math.pi / 180.0


def rad_to_deg(value: float) -> float:
    return value * 180.0 / math.pi


def finite_float(text: str) -> float | None:
    try:
        value = float(text)
    except ValueError:
        return None
    return value if math.isfinite(value) else None


@dataclass
class ArmRosState:
    side: str
    joint_names: list[str]
    enabled: bool = False
    feedback_received: bool = False
    current_rad: list[float] = field(default_factory=lambda: [0.0] * JOINT_COUNT)
    publisher: object | None = None


class MarvinJointControlNode(Node):
    """ROS side of the Tk UI."""

    def __init__(self) -> None:
        super().__init__("marvin_joint_control_ui")

        self.declare_parameter("arms", "both")
        self.declare_parameter("topic_namespace", "marvin")
        self.declare_parameter("live_rate_hz", 20.0)
        self.declare_parameter("require_feedback", True)
        self.declare_parameter("slider_min_deg", -180.0)
        self.declare_parameter("slider_max_deg", 180.0)
        self.declare_parameter("ui_scale", 1.6)
        self.declare_parameter("window_width", 1320)
        self.declare_parameter("window_height", 600)
        self.declare_parameter("left_joint_names", default_joint_names("L"))
        self.declare_parameter("right_joint_names", default_joint_names("R"))

        enabled_arms = normalize_arms(self.get_parameter("arms").value)
        namespace = normalize_namespace(self.get_parameter("topic_namespace").value)
        self.live_rate_hz = max(1.0, float(self.get_parameter("live_rate_hz").value))
        self.require_feedback = bool(self.get_parameter("require_feedback").value)
        self.slider_min_deg = float(self.get_parameter("slider_min_deg").value)
        self.slider_max_deg = float(self.get_parameter("slider_max_deg").value)
        self.ui_scale = max(0.75, float(self.get_parameter("ui_scale").value))
        self.window_width = max(800, int(self.get_parameter("window_width").value))
        self.window_height = max(600, int(self.get_parameter("window_height").value))

        self.arms: dict[str, ArmRosState] = {
            "left": ArmRosState(
                side="left",
                joint_names=list(self.get_parameter("left_joint_names").value),
                enabled="left" in enabled_arms,
            ),
            "right": ArmRosState(
                side="right",
                joint_names=list(self.get_parameter("right_joint_names").value),
                enabled="right" in enabled_arms,
            ),
        }
        if len(self.arms["left"].joint_names) != JOINT_COUNT:
            self.arms["left"].joint_names = default_joint_names("L")
        if len(self.arms["right"].joint_names) != JOINT_COUNT:
            self.arms["right"].joint_names = default_joint_names("R")

        self.feedback_callback: Callable[[str], None] | None = None
        self.publish_callback: Callable[[str], None] | None = None

        self._ui_subscriptions = []
        for side, arm in self.arms.items():
            if not arm.enabled:
                continue
            state_topic = f"{namespace}/{side}/joint_states"
            command_topic = f"{namespace}/{side}/joint_commands"
            arm.publisher = self.create_publisher(JointState, command_topic, qos_profile_sensor_data)
            self._ui_subscriptions.append(
                self.create_subscription(
                    JointState,
                    state_topic,
                    lambda msg, arm_side=side: self._handle_feedback(arm_side, msg),
                    qos_profile_sensor_data,
                )
            )
            self.get_logger().info(
                f"UI for {side} arm: subscribing {state_topic}, publishing {command_topic}"
            )

    def _handle_feedback(self, side: str, msg: JointState) -> None:
        arm = self.arms[side]
        values = self._joint_positions_by_name(arm.joint_names, msg)
        if values is None:
            return
        arm.current_rad = values
        first_feedback = not arm.feedback_received
        arm.feedback_received = True
        if first_feedback:
            self.get_logger().info(f"Received initial {side} arm feedback.")
        if self.feedback_callback is not None:
            self.feedback_callback(side)

    def _joint_positions_by_name(
        self, joint_names: list[str], msg: JointState
    ) -> list[float] | None:
        if not msg.name:
            if len(msg.position) < JOINT_COUNT:
                return None
            values = list(msg.position[:JOINT_COUNT])
        else:
            if len(msg.position) < len(msg.name):
                return None
            by_name = {name: msg.position[i] for i, name in enumerate(msg.name)}
            try:
                values = [by_name[name] for name in joint_names]
            except KeyError:
                return None

        return values if all(math.isfinite(value) for value in values) else None

    def publish_arm_command(self, side: str, command_rad: list[float]) -> bool:
        arm = self.arms[side]
        if not arm.enabled or arm.publisher is None:
            return False
        if self.require_feedback and not arm.feedback_received:
            self.get_logger().warn(
                f"Refusing to publish {side} arm command before feedback is received."
            )
            return False
        if len(command_rad) != JOINT_COUNT or not all(math.isfinite(v) for v in command_rad):
            self.get_logger().warn(f"Refusing to publish invalid {side} arm command.")
            return False

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = arm.joint_names
        msg.position = list(command_rad)
        arm.publisher.publish(msg)
        if self.publish_callback is not None:
            self.publish_callback(side)
        return True


class MarvinJointControlApp:
    """Tkinter UI that owns command entry and publishing state."""

    def __init__(self, node: MarvinJointControlNode) -> None:
        self.node = node
        self.root = tk.Tk()
        self._apply_ui_scale()
        self.root.title("Marvin Joint Control")
        self.root.geometry(f"{self.node.window_width}x{self.node.window_height}")
        self.root.minsize(1040, 560)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        self.mode = tk.StringVar(value="manual")
        self.live_enabled = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="Waiting for joint feedback.")
        self._updating_widgets = False
        self._closed = False
        self._last_live_publish_time = 0.0
        self._feedback_loaded: set[str] = set()

        self.slider_vars: dict[str, list[tk.DoubleVar]] = {
            side: [tk.DoubleVar(value=0.0) for _ in range(JOINT_COUNT)] for side in ARM_ORDER
        }
        self.entry_vars: dict[str, list[tk.StringVar]] = {
            side: [tk.StringVar(value="0.0") for _ in range(JOINT_COUNT)] for side in ARM_ORDER
        }
        self.current_vars: dict[str, list[tk.StringVar]] = {
            side: [tk.StringVar(value="--") for _ in range(JOINT_COUNT)] for side in ARM_ORDER
        }

        self.node.feedback_callback = self.on_feedback
        self.node.publish_callback = self.on_publish

        self._build_ui()
        self._set_mode("manual")
        self._schedule_ros_spin()
        self._schedule_live_publish()

    def _apply_ui_scale(self) -> None:
        self.root.tk.call("tk", "scaling", self.node.ui_scale)
        font_names = (
            "TkDefaultFont",
            "TkTextFont",
            "TkMenuFont",
            "TkHeadingFont",
            "TkCaptionFont",
        )
        for name in font_names:
            try:
                font = tkfont.nametofont(name)
            except tk.TclError:
                continue
            font.configure(size=max(12, int(font.cget("size") * 1.15)))

    def _build_ui(self) -> None:
        self.root.columnconfigure(0, weight=1)

        top = ttk.Frame(self.root, padding=(12, 10, 12, 8))
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(2, weight=1)

        self.mode_button = ttk.Button(top, text="", command=self.toggle_mode)
        self.mode_button.grid(row=0, column=0, padx=(0, 8))
        self.live_check = ttk.Checkbutton(
            top,
            text="Enable live publish",
            variable=self.live_enabled,
            command=self.on_live_toggle,
        )
        self.live_check.grid(row=0, column=1, padx=(0, 8))
        ttk.Button(top, text="Sync from feedback", command=self.sync_all_from_feedback).grid(
            row=0, column=2, sticky="w"
        )

        sliders = ttk.LabelFrame(self.root, text="Live sliders (deg)", padding=(10, 8))
        sliders.grid(row=1, column=0, padx=12, pady=(0, 8), sticky="nsew")
        sliders.columnconfigure(0, weight=1)
        sliders.columnconfigure(1, weight=1)
        self.slider_widgets: list[tk.Widget] = []
        for col, side in enumerate(ARM_ORDER):
            frame = ttk.Frame(sliders)
            frame.grid(row=0, column=col, padx=8, sticky="nsew")
            self._build_arm_sliders(frame, side)

        manual = ttk.LabelFrame(self.root, text="Manual command boxes (deg)", padding=(10, 8))
        manual.grid(row=2, column=0, padx=12, pady=(0, 8), sticky="nsew")
        manual.columnconfigure(0, weight=1)
        manual.columnconfigure(1, weight=1)
        self.entry_widgets: list[tk.Widget] = []
        for col, side in enumerate(ARM_ORDER):
            frame = ttk.Frame(manual)
            frame.grid(row=0, column=col, padx=8, sticky="nsew")
            self._build_arm_entries(frame, side)

        bottom = ttk.Frame(self.root, padding=(12, 0, 12, 10))
        bottom.grid(row=3, column=0, sticky="ew")
        bottom.columnconfigure(1, weight=1)
        self.send_button = ttk.Button(bottom, text="Send manual command", command=self.send_manual)
        self.send_button.grid(row=0, column=0, padx=(0, 8))
        ttk.Label(bottom, textvariable=self.status).grid(row=0, column=1, sticky="w")

    def _build_arm_sliders(self, frame: ttk.Frame, side: str) -> None:
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text=side.upper()).grid(row=0, column=0, columnspan=3, sticky="w")
        enabled = self.node.arms[side].enabled
        for i in range(JOINT_COUNT):
            ttk.Label(frame, text=self.node.arms[side].joint_names[i]).grid(
                row=i + 1, column=0, pady=2, sticky="w"
            )
            scale = ttk.Scale(
                frame,
                from_=self.node.slider_min_deg,
                to=self.node.slider_max_deg,
                variable=self.slider_vars[side][i],
                command=lambda _value, arm_side=side: self.on_slider_changed(arm_side),
            )
            scale.grid(row=i + 1, column=1, padx=8, pady=2, sticky="ew")
            self.slider_widgets.append(scale)
            ttk.Label(frame, textvariable=self.current_vars[side][i], width=8).grid(
                row=i + 1, column=2, pady=2, sticky="e"
            )
            if not enabled:
                scale.state(["disabled"])

    def _build_arm_entries(self, frame: ttk.Frame, side: str) -> None:
        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text=side.upper()).grid(row=0, column=0, columnspan=2, sticky="w")
        enabled = self.node.arms[side].enabled
        for i in range(JOINT_COUNT):
            ttk.Label(frame, text=self.node.arms[side].joint_names[i]).grid(
                row=i + 1, column=0, pady=2, sticky="w"
            )
            entry = ttk.Entry(frame, textvariable=self.entry_vars[side][i], width=10)
            entry.grid(row=i + 1, column=1, padx=8, pady=2, sticky="ew")
            self.entry_widgets.append(entry)
            if not enabled:
                entry.state(["disabled"])

    def toggle_mode(self) -> None:
        self._set_mode("live" if self.mode.get() == "manual" else "manual")

    def _set_mode(self, mode: str) -> None:
        self.mode.set(mode)
        live_mode = mode == "live"
        if not live_mode:
            self.live_enabled.set(False)
        self.mode_button.configure(
            text="Switch to manual boxes" if live_mode else "Switch to live sliders"
        )
        self.live_check.state(["!disabled"] if live_mode else ["disabled"])
        self.send_button.state(["disabled"] if live_mode else ["!disabled"])
        for widget in self.slider_widgets:
            if live_mode:
                widget.state(["!disabled"])
            else:
                widget.state(["disabled"])
        for widget in self.entry_widgets:
            if live_mode:
                widget.state(["disabled"])
            else:
                widget.state(["!disabled"])
        for side in ARM_ORDER:
            if not self.node.arms[side].enabled:
                for i in range(JOINT_COUNT):
                    self.slider_widgets[(0 if side == "left" else JOINT_COUNT) + i].state(
                        ["disabled"]
                    )
                    self.entry_widgets[(0 if side == "left" else JOINT_COUNT) + i].state(
                        ["disabled"]
                    )
        self.status.set("Live mode." if live_mode else "Manual box mode.")

    def on_live_toggle(self) -> None:
        if self.live_enabled.get() and not self._all_enabled_arms_have_feedback():
            self.live_enabled.set(False)
            self.status.set("Live publish needs feedback from every enabled arm.")
            return
        self.status.set("Live publish enabled." if self.live_enabled.get() else "Live publish off.")

    def on_slider_changed(self, side: str) -> None:
        if self._updating_widgets:
            return
        if self.mode.get() == "live" and self.live_enabled.get():
            self.publish_slider_commands([side])

    def on_feedback(self, side: str) -> None:
        arm = self.node.arms[side]
        for i, value in enumerate(arm.current_rad):
            self.current_vars[side][i].set(f"{rad_to_deg(value):.2f}")
        if side not in self._feedback_loaded:
            self._feedback_loaded.add(side)
            self.sync_arm_from_feedback(side)
        self.status.set(f"Feedback received from {side} arm.")

    def on_publish(self, side: str) -> None:
        self.status.set(f"Published {side} arm command.")

    def sync_all_from_feedback(self) -> None:
        synced = False
        for side in ARM_ORDER:
            if self.node.arms[side].enabled and self.node.arms[side].feedback_received:
                self.sync_arm_from_feedback(side)
                synced = True
        self.status.set("Synced UI from feedback." if synced else "No feedback to sync yet.")

    def sync_arm_from_feedback(self, side: str) -> None:
        arm = self.node.arms[side]
        if not arm.feedback_received:
            return
        self._updating_widgets = True
        try:
            for i, value_rad in enumerate(arm.current_rad):
                value_deg = rad_to_deg(value_rad)
                self.slider_vars[side][i].set(value_deg)
                self.entry_vars[side][i].set(f"{value_deg:.3f}")
        finally:
            self._updating_widgets = False

    def send_manual(self) -> None:
        published = 0
        for side in ARM_ORDER:
            if not self.node.arms[side].enabled:
                continue
            values = self._entry_degrees(side)
            if values is None:
                return
            if self.node.publish_arm_command(side, [deg_to_rad(value) for value in values]):
                published += 1
        if published == 0:
            self.status.set("No arm command was published.")

    def publish_slider_commands(self, sides: list[str] | tuple[str, ...] = ARM_ORDER) -> None:
        for side in sides:
            if not self.node.arms[side].enabled:
                continue
            values = [var.get() for var in self.slider_vars[side]]
            self.node.publish_arm_command(side, [deg_to_rad(value) for value in values])

    def _entry_degrees(self, side: str) -> list[float] | None:
        values: list[float] = []
        for i, var in enumerate(self.entry_vars[side]):
            value = finite_float(var.get())
            if value is None:
                joint_name = self.node.arms[side].joint_names[i]
                messagebox.showerror("Invalid value", f"{joint_name} must be a finite number.")
                self.status.set(f"Invalid value for {joint_name}.")
                return None
            values.append(value)
        return values

    def _all_enabled_arms_have_feedback(self) -> bool:
        return all(
            (not arm.enabled) or arm.feedback_received for arm in self.node.arms.values()
        )

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
        self.root.after(20, self._schedule_ros_spin)

    def _schedule_live_publish(self) -> None:
        if self._closed:
            return
        if self.mode.get() == "live" and self.live_enabled.get():
            self.publish_slider_commands()
        period_ms = max(10, int(1000.0 / self.node.live_rate_hz))
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
    node = MarvinJointControlNode()
    try:
        app = MarvinJointControlApp(node)
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
