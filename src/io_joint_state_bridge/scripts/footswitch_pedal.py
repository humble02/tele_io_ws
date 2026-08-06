#!/usr/bin/env python3

from __future__ import annotations

from contextlib import suppress
from glob import glob
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import UInt8

try:
    import evdev
    from evdev import ecodes
except ImportError:  # Keep the ROS graph alive and report the missing runtime dependency.
    evdev = None
    ecodes = None


LEFT_FROZEN = 0x01
RIGHT_FROZEN = 0x02


class FootswitchPedal(Node):
    def __init__(self) -> None:
        super().__init__("footswitch_pedal")

        self.declare_parameter("device_path", "")
        self.declare_parameter("vendor_id", 0x3553)
        self.declare_parameter("product_id", 0xB001)
        self.declare_parameter("grab_device", True)
        self.declare_parameter("reconnect_interval_sec", 1.0)
        self.declare_parameter("freeze_mask_topic", "/io_teleop/freeze_mask")

        self._device_path = str(self.get_parameter("device_path").value)
        self._vendor_id = int(self.get_parameter("vendor_id").value)
        self._product_id = int(self.get_parameter("product_id").value)
        self._grab_device = bool(self.get_parameter("grab_device").value)
        reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        if reconnect_interval_sec <= 0.0:
            raise ValueError("reconnect_interval_sec must be > 0")

        freeze_mask_topic = str(self.get_parameter("freeze_mask_topic").value)
        self._mask_pub = self.create_publisher(UInt8, freeze_mask_topic, 10)
        self._freeze_mask = 0
        self._device: Optional[object] = None

        self.create_timer(reconnect_interval_sec, self._ensure_device)
        self.create_timer(0.005, self._poll_device)
        # A heartbeat lets a restarted bridge recover the current latched state.
        self.create_timer(0.5, self._publish_mask)

        self._publish_mask()
        self._ensure_device()
        self.get_logger().info(
            "PCsensor footswitch ready: "
            f"VID:PID={self._vendor_id:04x}:{self._product_id:04x}, "
            "A=freeze left, C=freeze right, B=resume all, "
            f"topic={freeze_mask_topic}"
        )

    def _ensure_device(self) -> None:
        if self._device is not None:
            return
        if evdev is None or ecodes is None:
            self.get_logger().error(
                "python3-evdev is not installed; run: sudo apt install python3-evdev",
                throttle_duration_sec=10.0,
            )
            return

        # evdev.list_devices() silently omits devices without read/write access,
        # which makes a permissions problem look like a disconnected pedal.
        paths = (
            [self._device_path]
            if self._device_path
            else sorted(glob("/dev/input/event*"))
        )
        permission_denied = False
        for path in paths:
            try:
                candidate = evdev.InputDevice(path)
            except PermissionError:
                permission_denied = True
                continue
            except OSError:
                continue

            if not self._is_matching_keyboard(candidate):
                candidate.close()
                continue

            if self._grab_device:
                try:
                    candidate.grab()
                except OSError as exc:
                    self.get_logger().error(
                        f"Cannot exclusively grab {path}: {exc}",
                        throttle_duration_sec=5.0,
                    )
                    candidate.close()
                    continue

            self._device = candidate
            self.get_logger().info(
                f"Connected to {candidate.name!r} at {candidate.path}"
            )
            return

        if permission_denied:
            self.get_logger().error(
                "Permission denied while opening /dev/input/event*; install the supplied "
                "udev rule and reconnect the footswitch",
                throttle_duration_sec=5.0,
            )
        else:
            requested = self._device_path or (
                f"VID:PID {self._vendor_id:04x}:{self._product_id:04x} keyboard interface"
            )
            self.get_logger().warning(
                f"Footswitch {requested} not found; waiting for it to be connected",
                throttle_duration_sec=5.0,
            )

    def _is_matching_keyboard(self, device: object) -> bool:
        if device.info.vendor != self._vendor_id or device.info.product != self._product_id:
            return False
        try:
            key_codes = set(device.capabilities().get(ecodes.EV_KEY, []))
        except OSError:
            return False
        return {ecodes.KEY_A, ecodes.KEY_B, ecodes.KEY_C}.issubset(key_codes)

    def _poll_device(self) -> None:
        if self._device is None:
            return
        try:
            for event in self._device.read():
                if event.type != ecodes.EV_KEY or event.value != 1:
                    continue
                self._on_key_press(event.code)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.get_logger().warning(f"Footswitch disconnected: {exc}")
            self._close_device()

    def _on_key_press(self, key_code: int) -> None:
        previous_mask = self._freeze_mask
        if key_code == ecodes.KEY_A:
            self._freeze_mask |= LEFT_FROZEN
        elif key_code == ecodes.KEY_C:
            self._freeze_mask |= RIGHT_FROZEN
        elif key_code == ecodes.KEY_B:
            self._freeze_mask = 0
        else:
            return

        self._publish_mask()
        if self._freeze_mask != previous_mask:
            self.get_logger().info(
                "Footswitch state changed: "
                f"left_frozen={bool(self._freeze_mask & LEFT_FROZEN)}, "
                f"right_frozen={bool(self._freeze_mask & RIGHT_FROZEN)}"
            )

    def _publish_mask(self) -> None:
        msg = UInt8()
        msg.data = self._freeze_mask
        self._mask_pub.publish(msg)

    def _close_device(self) -> None:
        if self._device is None:
            return
        if self._grab_device:
            with suppress(OSError):
                self._device.ungrab()
        with suppress(OSError):
            self._device.close()
        self._device = None

    def destroy_node(self) -> bool:
        self._close_device()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FootswitchPedal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
