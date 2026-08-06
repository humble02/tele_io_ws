import builtins
import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = (
    Path(__file__).parents[1] / "launch" / "io_joint_state_bridge.launch.py"
)
SPEC = importlib.util.spec_from_file_location(
    "io_joint_state_bridge_launch", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_confirmation_waits_for_enter(monkeypatch):
    prompts = []

    def accept(prompt):
        prompts.append(prompt)
        return ""

    monkeypatch.setattr(builtins, "input", accept)

    assert MODULE.confirm_vr_services_restarted(None) == []
    assert prompts == [MODULE.VR_RESTART_CONFIRMATION_PROMPT]
    assert "VR 所有服务已重启" in prompts[0]
    assert "回车键" in prompts[0]


def test_confirmation_rejects_noninteractive_input(monkeypatch):
    def end_of_input(_prompt):
        raise EOFError

    monkeypatch.setattr(builtins, "input", end_of_input)

    with pytest.raises(RuntimeError, match="交互式终端"):
        MODULE.confirm_vr_services_restarted(None)
