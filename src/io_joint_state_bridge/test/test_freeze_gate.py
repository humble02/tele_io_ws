from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8


MODULE_PATH = Path(__file__).parents[1] / "scripts" / "io_joint_state_bridge.py"
SPEC = importlib.util.spec_from_file_location("io_joint_state_bridge_script", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
BRIDGE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = BRIDGE
SPEC.loader.exec_module(BRIDGE)

FOOTSWITCH_MODULE_PATH = Path(__file__).parents[1] / "scripts" / "footswitch_pedal.py"
FOOTSWITCH_SPEC = importlib.util.spec_from_file_location(
    "footswitch_pedal_script", FOOTSWITCH_MODULE_PATH
)
assert FOOTSWITCH_SPEC is not None and FOOTSWITCH_SPEC.loader is not None
FOOTSWITCH = importlib.util.module_from_spec(FOOTSWITCH_SPEC)
sys.modules[FOOTSWITCH_SPEC.name] = FOOTSWITCH
FOOTSWITCH_SPEC.loader.exec_module(FOOTSWITCH)


def joint_state(names: list[str], value: float) -> JointState:
    msg = JointState()
    msg.name = list(names)
    msg.position = [value] * len(names)
    return msg


def test_freeze_holds_last_command_and_resume_is_rate_limited() -> None:
    rclpy.init()
    node = BRIDGE.IoJointStateBridge()
    try:
        node._on_arm_state(
            node._left,
            joint_state(BRIDGE.LEFT_JOINT_NAMES, 0.1),
            node._left_state_topic,
        )
        node._on_arm_state(
            node._right,
            joint_state(BRIDGE.RIGHT_JOINT_NAMES, -0.1),
            node._right_state_topic,
        )
        node._on_hand_state(
            node._left_hand,
            node._left_gate,
            joint_state(BRIDGE.LEFT_HAND_JOINT_NAMES, 0.2),
            node._left_hand_state_topic,
        )
        node._on_hand_state(
            node._right_hand,
            node._right_gate,
            joint_state(BRIDGE.RIGHT_HAND_JOINT_NAMES, -0.2),
            node._right_hand_state_topic,
        )

        io_arm = JointState()
        io_arm.name = BRIDGE.DEFAULT_IO_COMMAND_JOINT_NAMES
        io_arm.position = [1.0] * len(io_arm.name)
        node._on_io_command(io_arm)
        node._on_io_hand_command(
            joint_state(BRIDGE.LEFT_HAND_JOINT_NAMES, 1.0),
            BRIDGE.LEFT_HAND_JOINT_NAMES,
            node._left_gate,
            node._left_hand_command_pub,
            node._io_left_hand_command_topic,
        )

        mask = UInt8()
        mask.data = BRIDGE.LEFT_FROZEN
        node._on_freeze_mask(mask)
        assert node._left_gate.frozen
        assert node._left_gate.arm_output.position == pytest.approx([1.0] * 7)
        assert node._left_gate.hand_output.position == pytest.approx([1.0] * 20)

        io_arm.position = [2.0] * len(io_arm.name)
        node._on_io_command(io_arm)
        assert node._left_gate.arm_output.position == pytest.approx([1.0] * 7)
        assert node._right_gate.arm_output.position == pytest.approx([2.0] * 7)

        mask.data = 0
        node._on_freeze_mask(mask)
        node._left_gate.arm_last_publish_ns = node.get_clock().now().nanoseconds - int(0.1e9)
        node._on_io_command(io_arm)
        assert not node._left_gate.frozen
        assert node._left_gate.arm_resuming
        assert node._left_gate.arm_output.position == pytest.approx([1.05] * 7, abs=0.002)
        assert node._right_gate.arm_output.position == pytest.approx([2.0] * 7)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_footswitch_keys_latch_sides_and_middle_resumes() -> None:
    original_ecodes = FOOTSWITCH.ecodes
    FOOTSWITCH.ecodes = SimpleNamespace(KEY_A=30, KEY_B=48, KEY_C=46)
    rclpy.init()
    node = FOOTSWITCH.FootswitchPedal()
    try:
        node._on_key_press(FOOTSWITCH.ecodes.KEY_A)
        assert node._freeze_mask == FOOTSWITCH.LEFT_FROZEN

        node._on_key_press(FOOTSWITCH.ecodes.KEY_C)
        assert node._freeze_mask == (
            FOOTSWITCH.LEFT_FROZEN | FOOTSWITCH.RIGHT_FROZEN
        )

        node._on_key_press(FOOTSWITCH.ecodes.KEY_B)
        assert node._freeze_mask == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        FOOTSWITCH.ecodes = original_ecodes
