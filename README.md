# sim-gears-for-space-nav

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

Consume the native package with:

```cmake
find_package(sim-gears-for-space-nav CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE
  sim-gears-for-space-nav::sim-gears-for-space-nav)
```

Include native headers as `<sim-gears-for-space-nav/...>`. The shared library is named
`libsim-gears-for-space-nav` on Linux. Build targets, qualified CMake options and
Python/MATLAB wrapper modules use `sim_gears_for_space_nav`, since wrapper
identifiers cannot contain hyphens. Python distribution metadata uses
`sim-gears-for-space-nav`; import it with `import sim_gears_for_space_nav`.
The C++ and generated MATLAB class namespace remains `simulation_gears`.

Configure a fresh build directory when migrating from the former
`SimulationGears_for_SpaceNav` package. Update consumer package/target names,
qualified includes, Python imports and project-qualified CMake options. Existing
checkout paths and the four ROS package names retain their established spellings.

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
  -D sim_gears_for_space_nav_WRAPPER_INTERFACE_FILES=/path/to/interface.i
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
