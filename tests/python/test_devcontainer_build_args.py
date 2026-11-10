"""Verify preservation of caller-owned Docker arguments during ROS changes.

Run the updater against temporary inputs; leave the checkout's container file alone.

Example:
    python3 -m pytest tests/python/test_devcontainer_build_args.py -q

Output:
    7 passed

Changelog:
    29-09-2026  Pietro Califano, Codex gpt-6  Cover argument preservation across ROS changes.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import cast

import pytest


def _run_updater(input_path: Path, ros_mode: str) -> dict[str, object]:
    """Run the repository helper against a disposable container configuration.

    Args:
        input_path: Temporary JSON or JSONC input; the helper leaves it unchanged.
        ros_mode: ROS selection forwarded to the helper's environment.

    Returns:
        Regenerated container configuration decoded from standard output.

    Raises:
        subprocess.CalledProcessError: If the updater exits unsuccessfully.
        subprocess.TimeoutExpired: If the updater exceeds the execution limit.
        json.JSONDecodeError: If the updater does not return valid JSON.
    """
    helper_path = Path(__file__).resolve().parents[2] / ".devcontainer/update_devcontainer_json.py"
    environment = dict(os.environ, DEVCONTAINER_JSON_PATH=str(input_path),
                       ROS_MODE=ros_mode, ROS_DISTRO="jazzy", ROS_PROFILE="desktop", CUDA="off")
    result = subprocess.run([sys.executable, str(helper_path)], env=environment,
                            capture_output=True, text=True, check=True, timeout=10)
    return cast(dict[str, object], json.loads(result.stdout))


@pytest.mark.parametrize("ros_mode", ["none", "overlay"])
@pytest.mark.parametrize("jsonc", [False, True])
def test_preserve_custom_build_arguments(tmp_path: Path, ros_mode: str, jsonc: bool) -> None:
    """Retain custom arguments and unrelated fields while updating ROS selection.

    Args:
        tmp_path: Fixture-owned directory for the disposable input.
        ros_mode: Enable or disable ROS without changing caller-owned keys.
        jsonc: Include a comment when exercising the JSONC reader.
    """
    input_path = tmp_path / "devcontainer.json"
    data = {
        "build": {
            "dockerfile": "Dockerfile", "context": "..",
            "args": {"CUSTOM_OPTION": "keep", "ROS_CUSTOM": "also-keep",
                     "ROS_MODE": "old", "ROS_DISTRO": "old", "ROS_PROFILE": "old"},
        },
        "mounts": ["custom-mount"],
    }
    contents = json.dumps(data)
    if jsonc:
        contents = "// Read the same argument mapping through JSONC.\n" + contents
    input_path.write_text(contents, encoding="utf-8")

    # Compare owned keys with the requested input and retain every unrelated field.
    output = _run_updater(input_path, ros_mode)
    build = cast(dict[str, object], output["build"])
    arguments = cast(dict[str, str], build["args"])
    assert arguments["CUSTOM_OPTION"] == data["build"]["args"]["CUSTOM_OPTION"]
    assert arguments["ROS_CUSTOM"] == data["build"]["args"]["ROS_CUSTOM"]
    assert build["context"] == data["build"]["context"]
    assert output["mounts"] == data["mounts"]
    if ros_mode == "none":
        assert all(key not in arguments for key in ("ROS_MODE", "ROS_DISTRO", "ROS_PROFILE"))
    else:
        assert arguments["ROS_MODE"] == ros_mode
        assert arguments["ROS_DISTRO"] == "jazzy"
        assert arguments["ROS_PROFILE"] == "desktop"


def test_ros_transitions_are_idempotent(tmp_path: Path) -> None:
    """Keep custom arguments across ROS transitions and repeated regeneration.

    Args:
        tmp_path: Fixture-owned directory for inputs to successive updates.
    """
    input_path = tmp_path / "devcontainer.json"
    input_path.write_text(json.dumps({"build": {"args": {"CUSTOM_OPTION": "keep"}}}),
                          encoding="utf-8")

    # Reuse the generated output so each transition exercises the previous mode's settings.
    for mode in ("overlay", "none"):
        output = _run_updater(input_path, mode)
        input_path.write_text(json.dumps(output), encoding="utf-8")
        assert _run_updater(input_path, mode) == output
        arguments = cast(dict[str, object], output["build"])["args"]
        assert cast(dict[str, str], arguments)["CUSTOM_OPTION"] == "keep"


@pytest.mark.parametrize("ros_mode", ["none", "overlay"])
def test_empty_build_arguments(tmp_path: Path, ros_mode: str) -> None:
    """Retain the helper's existing empty-mapping behavior without stale ROS keys.

    Args:
        tmp_path: Fixture-owned directory for the disposable input.
        ros_mode: Enable or disable the managed ROS arguments.
    """
    input_path = tmp_path / "devcontainer.json"
    input_path.write_text(json.dumps({"build": {"args": {}}}), encoding="utf-8")
    output = _run_updater(input_path, ros_mode)
    build = cast(dict[str, object], output["build"])
    if ros_mode == "none":
        assert "args" not in build
    else:
        assert set(cast(dict[str, str], build["args"])) == {"ROS_MODE", "ROS_DISTRO", "ROS_PROFILE"}
