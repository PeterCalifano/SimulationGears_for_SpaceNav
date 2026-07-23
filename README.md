# SimulationGears for SpaceNav

SimulationGears is a MATLAB-first spacecraft navigation simulation library with
a growing native C++20 surface, optional CUDA support, wrappers, and an optional
ROS 2 Jazzy overlay. MATLAB sources remain under `matlab/`; the reusable native
package is built from the repository root.

## Quick start

Set up MATLAB with:

```matlab
run('matlab/SetupSimGears.m')
```

Build and test the native CPU package with:

```bash
./build_lib.sh
```

For a reviewable preset flow, use:

```bash
cmake --preset native-cpu
cmake --build --preset native-cpu
ctest --preset native-cpu --output-on-failure --no-tests=error
```

The native library exports
`SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav` for downstream
CMake consumers.

## Development surfaces

- [Native logging](doc/logging.md) describes `CLogger`, its level contract,
  streams, colors, and environment override.
- [Testing and CI](doc/testing_ci.md) covers native and Python CTest, CUDA runner
  gating, container usage, ROS 2, and the documentation artifact.
- [ROS 2 overlay](doc/ros2_overlay.md) defines the four-package Jazzy overlay.
- [Version and release](doc/version_release.md) defines semantic version
  resolution and the canonical source TGZ procedure.
- [Build script reference](doc/build_script_doc.md) documents `build_lib.sh`.

Wrappers are optional: `./build_lib.sh -p` requests Python wrapper generation,
and `./build_lib.sh -p -m` requests Python plus MATLAB wrapper generation. Each
request auto-disables when no valid `src/wrap_interface.i` is present. Supply an
explicit interface when needed, for example:

```bash
./build_lib.sh -p \
  -D SimulationGears_for_SpaceNav_WRAPPER_INTERFACE_FILES=/path/to/interface.i
```

Generated wrapper products are build artifacts and are not part of the
canonical source release.

## Containers and ROS 2

The devcontainer provides the native toolchain; run the same project command
inside an existing container with `./run_in_container.sh`. GPU configuration is
opt-in through the CUDA setup and the self-hosted CUDA CI variable. The ROS 2
overlay is independent of `./build_lib.sh` and is built with:

```bash
./build_ros2.sh --clean
```

See the linked documents for prerequisites and exact contracts.
