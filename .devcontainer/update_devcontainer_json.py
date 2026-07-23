#!/usr/bin/env python3
"""Regenerate ``devcontainer.json`` while preserving target-owned settings.

The updater owns the selected base features, CUDA runtime arguments, and ROS
build/environment entries. Existing editor settings, extensions, mounts,
environment entries, features, and unrelated build arguments are retained.
JSONC comments are accepted on input; stable plain JSON is emitted on stdout.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path


DEFAULT_CUDA_VERSION = "12.9"
DEFAULT_GPU_RUNTIME = "docker"
SUPPORTED_GPU_RUNTIMES = ("docker", "podman")

CUDA_REMOTE_ENV = {
    "PATH": "/usr/local/cuda/bin:${containerEnv:PATH}",
    "LD_LIBRARY_PATH": "/usr/local/cuda/lib64:${containerEnv:LD_LIBRARY_PATH}",
    "CUDA_HOME": "/usr/local/cuda",
}
ROS_CONTAINER_ENV = {
    "ROS_LOCALHOST_ONLY": "1",
    "ROS_DOMAIN_ID": "42",
}
ROS_BUILD_ARGS = ("ROS_MODE", "ROS_DISTRO", "ROS_PROFILE")
CUDA_FEATURE_KEY = "ghcr.io/devcontainers/features/nvidia-cuda:2"

# Preserve the original SimulationGears extension set and add only the managed
# editor integrations useful for CMake and Python development.
DEFAULT_EXTENSIONS = [
    "ms-vscode.cpptools",
    "ms-vscode.cpptools-themes",
    "ms-vscode.cmake-tools",
    "twxs.cmake",
    "ms-python.python",
    "ms-python.vscode-pylance",
    "ms-python.debugpy",
    "donjayamanne.python-extension-pack",
]

DOCKER_GPU_RUN_ARGS = ["--gpus", "all"]
PODMAN_GPU_RUN_ARGS = [
    "--device",
    "nvidia.com/gpu=all",
    "--security-opt=label=disable",
]


def _Gpu_run_args(gpu_runtime_: str) -> list[str]:
    """Return managed GPU arguments for one supported container engine."""
    if gpu_runtime_ == "docker":
        return list(DOCKER_GPU_RUN_ARGS)
    if gpu_runtime_ == "podman":
        return list(PODMAN_GPU_RUN_ARGS)
    raise ValueError(
        "DEVCONTAINER_GPU_RUNTIME must be one of: "
        + ", ".join(SUPPORTED_GPU_RUNTIMES)
    )


def _Strip_gpu_run_args(arguments_: object) -> list[object]:
    """Remove updater-owned GPU arguments while preserving unrelated values."""
    if not isinstance(arguments_, list):
        return []

    remaining_: list[object] = []
    index_ = 0
    while index_ < len(arguments_):
        current_ = arguments_[index_]
        next_ = arguments_[index_ + 1] if index_ + 1 < len(arguments_) else None
        if current_ == "--gpus" and next_ == "all":
            index_ += 2
            continue
        if current_ == "--gpus=all":
            index_ += 1
            continue
        if current_ == "--device" and next_ == "nvidia.com/gpu=all":
            index_ += 2
            continue
        if current_ == "--device=nvidia.com/gpu=all":
            index_ += 1
            continue
        if current_ == "--security-opt" and next_ == "label=disable":
            index_ += 2
            continue
        if current_ == "--security-opt=label=disable":
            index_ += 1
            continue
        remaining_.append(current_)
        index_ += 1
    return remaining_


def _Strip_jsonc_comments(text_: str) -> str:
    """Remove line and block comments without altering JSON string values."""
    output_: list[str] = []
    in_string_ = False
    escape_next_ = False
    index_ = 0

    while index_ < len(text_):
        character_ = text_[index_]
        if in_string_:
            output_.append(character_)
            if escape_next_:
                escape_next_ = False
            elif character_ == "\\":
                escape_next_ = True
            elif character_ == '"':
                in_string_ = False
            index_ += 1
            continue

        if character_ == '"':
            in_string_ = True
            output_.append(character_)
            index_ += 1
            continue

        if character_ == "/" and index_ + 1 < len(text_):
            next_character_ = text_[index_ + 1]
            if next_character_ == "/":
                index_ += 2
                while index_ < len(text_) and text_[index_] not in "\r\n":
                    index_ += 1
                continue
            if next_character_ == "*":
                index_ += 2
                while index_ + 1 < len(text_) and not (
                    text_[index_] == "*" and text_[index_ + 1] == "/"
                ):
                    if text_[index_] in "\r\n":
                        output_.append(text_[index_])
                    index_ += 1
                if index_ + 1 < len(text_):
                    index_ += 2
                continue

        output_.append(character_)
        index_ += 1

    return "".join(output_)


def _Strip_trailing_commas(text_: str) -> str:
    """Remove structural trailing commas without changing string contents."""
    output_: list[str] = []
    in_string_ = False
    escape_next_ = False
    index_ = 0

    while index_ < len(text_):
        character_ = text_[index_]
        if in_string_:
            output_.append(character_)
            if escape_next_:
                escape_next_ = False
            elif character_ == "\\":
                escape_next_ = True
            elif character_ == '"':
                in_string_ = False
            index_ += 1
            continue

        if character_ == '"':
            in_string_ = True
            output_.append(character_)
            index_ += 1
            continue

        if character_ == ",":
            lookahead_ = index_ + 1
            while lookahead_ < len(text_) and text_[lookahead_].isspace():
                lookahead_ += 1
            if lookahead_ < len(text_) and text_[lookahead_] in "}]":
                index_ += 1
                continue

        output_.append(character_)
        index_ += 1

    return "".join(output_)


def _Load_existing(path_: Path) -> dict[str, object]:
    """Load an existing JSON or JSONC configuration, if present."""
    if not path_.is_file():
        return {}

    text_ = _Strip_jsonc_comments(path_.read_text(encoding="utf-8"))
    text_ = _Strip_trailing_commas(text_).strip()
    if not text_:
        return {}
    data_ = json.loads(text_)
    if not isinstance(data_, dict):
        raise ValueError(f"existing {path_} must contain a JSON object")
    return data_


def _As_mapping(value_: object) -> dict[str, object]:
    """Return a shallow mapping copy or an empty mapping for invalid input."""
    return dict(value_) if isinstance(value_, dict) else {}


def Main() -> int:
    """Merge managed options into the existing configuration and emit JSON."""
    cuda_ = os.environ.get("CUDA", "off")
    cuda_version_ = os.environ.get("CUDA_VERSION", DEFAULT_CUDA_VERSION)
    gpu_runtime_ = os.environ.get(
        "DEVCONTAINER_GPU_RUNTIME", DEFAULT_GPU_RUNTIME
    )
    ros_mode_ = os.environ.get("ROS_MODE", "none")
    ros_distro_ = os.environ.get("ROS_DISTRO", "")
    ros_profile_ = os.environ.get("ROS_PROFILE", "ros-base")
    existing_path_ = Path(
        os.environ.get(
            "DEVCONTAINER_JSON_PATH",
            str(Path(__file__).resolve().parent / "devcontainer.json"),
        )
    )

    try:
        data_ = _Load_existing(existing_path_)
        managed_gpu_arguments_ = _Gpu_run_args(gpu_runtime_)
    except (json.JSONDecodeError, OSError, ValueError) as error_:
        print(f"update_devcontainer_json.py: {error_}", file=sys.stderr)
        return 1

    data_.setdefault("name", "SimulationGears")

    # Preserve unrelated build arguments while managing the ROS selection.
    build_ = _As_mapping(data_.get("build"))
    build_["dockerfile"] = "Dockerfile"
    build_args_ = _As_mapping(build_.get("args"))
    if ros_mode_ != "none":
        build_args_.update(
            {
                "ROS_MODE": ros_mode_,
                "ROS_DISTRO": ros_distro_,
                "ROS_PROFILE": ros_profile_,
            }
        )
    else:
        for argument_ in ROS_BUILD_ARGS:
            build_args_.pop(argument_, None)
    if build_args_:
        build_["args"] = build_args_
    else:
        build_.pop("args", None)
    data_["build"] = build_

    # Preserve project features while updating the base Python/CUDA features.
    features_ = _As_mapping(data_.get("features"))
    features_["ghcr.io/devcontainers/features/conda:1"] = {
        "addCondaForge": True,
        "version": "latest",
    }
    features_["ghcr.io/devcontainers/features/python:1"] = {
        "installTools": True,
        "enableShared": True,
        "version": "3.12",
    }
    if cuda_ == "on":
        features_[CUDA_FEATURE_KEY] = {
            "installCudnn": True,
            "installCudnnDev": True,
            "installNvtx": True,
            "installToolkit": True,
            "cudaVersion": cuda_version_,
            "cudnnVersion": "automatic",
        }
    else:
        features_.pop(CUDA_FEATURE_KEY, None)
    data_["features"] = dict(sorted(features_.items()))

    run_args_ = _Strip_gpu_run_args(data_.get("runArgs"))
    if cuda_ == "on":
        run_args_ = managed_gpu_arguments_ + run_args_
    if run_args_:
        data_["runArgs"] = run_args_
    else:
        data_.pop("runArgs", None)

    remote_env_ = _As_mapping(data_.get("remoteEnv"))
    if cuda_ == "on":
        remote_env_.update(CUDA_REMOTE_ENV)
    else:
        for variable_ in CUDA_REMOTE_ENV:
            remote_env_.pop(variable_, None)
    if remote_env_:
        data_["remoteEnv"] = remote_env_
    else:
        data_.pop("remoteEnv", None)

    container_env_ = _As_mapping(data_.get("containerEnv"))
    if ros_mode_ != "none":
        container_env_.update(ROS_CONTAINER_ENV)
    else:
        for variable_ in ROS_CONTAINER_ENV:
            container_env_.pop(variable_, None)
    if container_env_:
        data_["containerEnv"] = container_env_
    else:
        data_.pop("containerEnv", None)

    customizations_ = _As_mapping(data_.get("customizations"))
    vscode_ = _As_mapping(customizations_.get("vscode"))
    existing_extensions_ = vscode_.get("extensions", [])
    if not isinstance(existing_extensions_, list):
        existing_extensions_ = []
    extra_extensions_ = [
        extension_
        for extension_ in existing_extensions_
        if extension_ not in DEFAULT_EXTENSIONS
    ]
    vscode_["extensions"] = DEFAULT_EXTENSIONS + extra_extensions_
    customizations_["vscode"] = vscode_
    data_["customizations"] = customizations_

    json.dump(data_, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(Main())
