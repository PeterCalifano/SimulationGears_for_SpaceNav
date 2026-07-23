# Testing, CI, and development environments

## Local verification

The native preset enables Catch2 and project-owned Python tests through CTest:

```bash
cmake --preset native-cpu
cmake --build --preset native-cpu --parallel 4
ctest --preset native-cpu --output-on-failure --no-tests=error
```

Focused Python checks can be run with `python3 -m pytest -q tests/python`.
MATLAB tests remain a separate surface and require MATLAB; see `AGENTS.md` for
the authoritative batch command.

## Continuous integration

The native CPU workflow uses full Git history for version resolution, disables
host-specific CPU tuning, enables Catch2 and Python CTest discovery, and tests a
downloaded build artifact in a separate job. Pull requests to `develop` and
`dev*` branches are covered, and release tags match `v*.*.*`.

CUDA CI runs only for manual dispatch or release tags and only when the
repository variable `CI_USE_SELF_HOSTED` is exactly `true`. Both CUDA jobs
require the self-hosted Linux/X64/GPU/CUDA labels. A skipped workflow means CUDA
coverage did not run; it is not evidence of a passing GPU build.

ROS 2 CI uses Jazzy, synchronizes project metadata, rejects manifest drift,
resolves dependencies with rosdep, and runs
`./build_ros2.sh --clean --no-version-sync`.

Documentation CI configures and builds the `docs` preset, verifies the HTML
index and XML output, and uploads a normal `simulation-gears-documentation`
documentation artifact. There is no Pages deployment.

## Containers

`.devcontainer/` owns the reproducible editor/development image. Use
`./configure_devcontainer.sh` to select CPU or CUDA configuration, and
`./run_in_container.sh` to execute project commands in the configured container.
CUDA setup is explicit; CPU development does not require an NVIDIA runtime.

The optional ROS 2 workspace is deliberately separate from the native build.
Its generated `ros2/build`, `ros2/install`, and `ros2/log` trees are disposable
and excluded from source releases.
