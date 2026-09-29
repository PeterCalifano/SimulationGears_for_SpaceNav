"""Verify that packaged metadata consumes the existing CMake version composer.

Example:
    python3 -m pytest tests/python/test_python_package_version.py -q

Output:
    4 passed

Changelog:
    29-09-2026  Pietro Califano, Codex gpt-6  Cover release qualifiers in package metadata.
"""

from __future__ import annotations

import subprocess
import tomllib
from pathlib import Path

import pytest
from packaging.version import Version


@pytest.mark.parametrize(("prerelease", "metadata"),
                         [("", ""), ("rc.1", ""), ("", "build7"), ("feature.fix", "build7")])
def test_packaged_version_matches_composer(tmp_path: Path, prerelease: str, metadata: str) -> None:
    """Keep release qualifiers when materializing metadata with the existing composer.

    Args:
        tmp_path: Fixture-owned directory for the disposable CMake consumer.
        prerelease: Synthetic prerelease qualifier passed to the shared composer.
        metadata: Synthetic build metadata passed to the shared composer.
    """
    root = Path(__file__).resolve().parents[2]
    script_path = tmp_path / "compose.cmake"
    generated_path = tmp_path / "pyproject.toml"
    composed_path = tmp_path / "composed-version.txt"

    # Exercise the actual shared composer and project template in a disposable metadata consumer.
    script_path.write_text(f'''cmake_minimum_required(VERSION 3.25)
include("{(root / 'cmake/HandlePythonWrapper.cmake').as_posix()}")
set(PROJECT_NAME "package_under_test")
set(PROJECT_VERSION "1.2.3")
_compose_python_package_version(PYTHON_PACKAGE_VERSION "1.2.3" "{prerelease}" "{metadata}")
configure_file("{(root / 'python/pyproject.toml.in').as_posix()}" "{generated_path.as_posix()}" @ONLY)
file(WRITE "{composed_path.as_posix()}" "${{PYTHON_PACKAGE_VERSION}}")
''', encoding="utf-8")
    subprocess.run(["cmake", "-P", str(script_path)], capture_output=True, text=True,
                   check=True, timeout=10)

    # Match the composed value exactly and require a valid Python package version.
    generated = tomllib.loads(generated_path.read_text(encoding="utf-8"))
    composed = composed_path.read_text(encoding="utf-8")
    assert generated["project"]["version"] == composed
    Version(composed)
