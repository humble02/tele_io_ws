from __future__ import annotations

import importlib.util
import sys
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import UInt8


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "io_joint_state_bridge.py"
SPEC = importlib.util.spec_from_file_location(
    "io_joint_state_bridge_limit_promotion_script", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)


def test_bridge_resets_then_promotes_marvin_limits_atomically() -> None:
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "marvin_limit_promotion_delay_sec:=0.05",
        ]
    )
    driver = Node("marvin_driver", namespace="/marvin")
    driver.declare_parameter("velocity_ratio", 100)
    driver.declare_parameter("acceleration_ratio", 100)
    updates: list[tuple[float, dict[str, int]]] = []

    def record_update(parameters):
        values = {
            parameter.name: int(parameter.value)
            for parameter in parameters
            if parameter.name in {"velocity_ratio", "acceleration_ratio"}
        }
        if values:
            updates.append((time.monotonic(), values))
        return SetParametersResult(successful=True)

    driver.add_on_set_parameters_callback(record_update)
    bridge = BRIDGE.IoJointStateBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(bridge)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if (
                driver.get_parameter("velocity_ratio").value == 100
                and driver.get_parameter("acceleration_ratio").value == 100
                and len(updates) >= 2
                and bridge._marvin_limit_timer.is_canceled()
            ):
                break

        assert driver.get_parameter("velocity_ratio").value == 100
        assert driver.get_parameter("acceleration_ratio").value == 100
        assert updates[0][1] == {
            "velocity_ratio": 10,
            "acceleration_ratio": 10,
        }
        assert updates[1][1] == {
            "velocity_ratio": 100,
            "acceleration_ratio": 100,
        }
        assert updates[1][0] - updates[0][0] >= 0.04
        assert bridge._marvin_limit_phase == "done"
        assert bridge._marvin_limit_timer.is_canceled()
    finally:
        executor.remove_node(bridge)
        executor.remove_node(driver)
        bridge.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_footswitch_resume_restarts_limit_buffer_and_cancels_startup_promotion() -> None:
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "marvin_limit_promotion_delay_sec:=1.0",
            "-p",
            "footswitch_resume_limit_delay_sec:=0.05",
        ]
    )
    driver = Node("marvin_driver", namespace="/marvin")
    driver.declare_parameter("velocity_ratio", 100)
    driver.declare_parameter("acceleration_ratio", 100)
    updates: list[tuple[float, dict[str, int]]] = []

    def record_update(parameters):
        values = {
            parameter.name: int(parameter.value)
            for parameter in parameters
            if parameter.name in {"velocity_ratio", "acceleration_ratio"}
        }
        if values:
            updates.append((time.monotonic(), values))
        return SetParametersResult(successful=True)

    driver.add_on_set_parameters_callback(record_update)
    bridge = BRIDGE.IoJointStateBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(driver)
    executor.add_node(bridge)

    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if len(updates) == 1 and bridge._marvin_limit_phase == "startup_promotion":
                break

        assert [update[1] for update in updates] == [
            {"velocity_ratio": 10, "acceleration_ratio": 10}
        ]

        frozen = UInt8()
        frozen.data = BRIDGE.LEFT_FROZEN
        bridge._on_freeze_mask(frozen)

        resumed = UInt8()
        resumed.data = 0
        bridge._on_freeze_mask(resumed)
        resume_generation = bridge._marvin_limit_generation

        # The footswitch publishes a 0-mask heartbeat every 0.5 s. It must not
        # restart the buffer after the actual frozen-to-resumed edge.
        bridge._on_freeze_mask(resumed)
        assert bridge._marvin_limit_generation == resume_generation

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if len(updates) >= 3 and bridge._marvin_limit_phase == "done":
                break

        assert [update[1] for update in updates] == [
            {"velocity_ratio": 10, "acceleration_ratio": 10},
            {"velocity_ratio": 10, "acceleration_ratio": 10},
            {"velocity_ratio": 100, "acceleration_ratio": 100},
        ]
        assert updates[2][0] - updates[1][0] >= 0.04
        assert bridge._marvin_limit_phase == "done"
        assert bridge._marvin_limit_timer.is_canceled()
    finally:
        executor.remove_node(bridge)
        executor.remove_node(driver)
        bridge.destroy_node()
        driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
