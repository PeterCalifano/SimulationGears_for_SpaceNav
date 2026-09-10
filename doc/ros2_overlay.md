# SimulationGears ROS 2 Overlay

The optional ROS 2 Jazzy overlay is a four-package colcon workspace layered on
top of the C++ SimulationGears library. The native entry point remains
`./build_lib.sh`; it does not source ROS, inspect `ros2/`, or require a ROS
installation.

## Ownership and package layout

ROS integration is confined to `ros2/`, `build_ros2.sh`, the four
`COLCON_IGNORE` markers, and the dedicated overlay workflow. There is no root
`package.xml` and no root `ENABLE_ROS2` option.

| Package | Role |
|---|---|
| `simulation_gears` | Plain CMake shim that adds the repository root and installs the native core package into the colcon prefix. |
| `simulation_gears_interfaces` | Build-info message and service definitions. |
| `simulation_gears_ros` | Core metadata conversion seam, lifecycle component, and standalone executable. |
| `simulation_gears_spinup` | Configuration, standalone/composable launch files, and launch tests. |

The shim disables core tests and nested support submodules that the build-info
sample does not consume, only inside the overlay build. It does not change their
standalone defaults. The bridge consumes the real exported target:

```cmake
find_package(SimulationGears_for_SpaceNav REQUIRED)
target_link_libraries(target PRIVATE
  SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav)
```

## Public sample contract

The overlay deliberately demonstrates native package consumption through build
metadata and does not invent a simulation algorithm or product API.

`GetSampleBuildInfo.srv` has an empty request and returns:

```text
string version
string full_version
string status
```

`SampleBuildStatus.msg` contains:

```text
builtin_interfaces/Time stamp
uint64 request_count
string version
string state
```

The lifecycle node is named `simulation_gears_sample`. It exposes private names
`~/get_build_info` and `~/status`, so root launches resolve them as
`/simulation_gears_sample/get_build_info` and
`/simulation_gears_sample/status`. An enclosing namespace is applied normally,
for example `/integration/simulation_gears_sample/get_build_info`.

Both response versions come directly from the native package's generated
`config.h`: `PROJECT_VERSION` is the strict core version and `FULL_VERSION`
retains prerelease/build metadata. The deterministic sample status/state is
`active`. Every serviced request increments `request_count` and publishes the
corresponding status while the lifecycle publisher is active.

## Build and launch

Source a ROS environment or let the helper source
`/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash`:

```bash
./build_ros2.sh --clean
./build_ros2.sh --skip-tests
./build_ros2.sh --packages-select simulation_gears_ros
ROS_DISTRO=jazzy ./build_ros2.sh --debug
```

After a successful build, source the overlay and launch either form:

```bash
source ros2/install/setup.bash
ros2 launch simulation_gears_spinup simulation_gears.launch.py
ros2 launch simulation_gears_spinup simulation_gears_composition.launch.py
```

Both launch files request configure and then activate automatically. The
composable form retains the local Jazzy lifecycle-name compatibility adapter;
it can be removed after the supported `launch_ros` implementation resolves
composed autostart identities correctly.

CUDA and OptiX remain optional through the overlay facade:

| User flag | Colcon CMake argument | Core option |
|---|---|---|
| `--cuda` | `-DSIMULATION_GEARS_ENABLE_CUDA=ON` | `SimulationGears_for_SpaceNav_ENABLE_CUDA=ON` |
| `--optix` | CUDA plus `-DSIMULATION_GEARS_ENABLE_OPTIX=ON` | `SimulationGears_for_SpaceNav_ENABLE_OPTIX=ON` |

Use the helper flags rather than passing generic `ENABLE_CUDA` or
`ENABLE_OPTIX` values directly; the shim maps its stable facade to this
project's qualified core options. OptiX still requires an SDK root through the
native project's documented CMake/environment inputs.

## Project metadata synchronization

The root CMake project owns package descriptions, homepage, maintainer, license,
and the strict core version. A normal `./generate_version.sh` run synchronizes
all four immediate `ros2/*/package.xml` files when the complete helper is
present. The explicit compatibility form and opt-out are:

```bash
./generate_version.sh --sync-ros2
./generate_version.sh --no-sync-ros2
```

`build_ros2.sh` performs explicit synchronization unless
`--no-version-sync` is supplied. Synchronization preserves established package
names, dependencies, non-website URLs, XML processing instructions, and file
modes. Package manifest versions remain strict `X.Y.Z`; dirty and other full
build metadata is exposed only through `FULL_VERSION` at runtime.

Automatic mode quietly does nothing when `ros2/` or the complete helper is
absent, preserving compatibility with checkouts that do not carry the optional
overlay. The dedicated CI workflow synchronizes first, rejects manifest drift,
installs dependencies with `rosdep`, and runs the full colcon build/test gate.

## Colcon discovery boundaries

Tracked markers in `python/`, `lib/`, `examples/`, and `tests/` keep a parent
colcon crawl from mistaking non-ROS support trees for packages. The build helper
also creates best-effort markers in generated top-level build/install trees.

Generated `ros2/build`, `ros2/install`, and `ros2/log` trees are local artifacts.
`./build_ros2.sh --clean` removes only those three overlay paths before
rebuilding.
