# SimulationGears_for_SpaceNav {#mainpage}

SimulationGears provides spacecraft navigation simulation models in MATLAB and
reusable native C++20 utilities, with optional CUDA, wrappers, and a ROS 2 Jazzy
overlay. The repository README is the entry point for installation and project
layout.

## Native build and consumption

```bash
cmake --preset native-cpu
cmake --build --preset native-cpu
ctest --preset native-cpu --output-on-failure --no-tests=error
```

Installed consumers use:

```cmake
find_package(SimulationGears_for_SpaceNav CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE
  SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav)
```

Use `CPU_ENABLE_NATIVE_TUNING=OFF` for portable CPU artifacts. CUDA is enabled
with `ENABLE_CUDA=ON`; OptiX remains an independent opt-in.

## Project contracts

- @ref md_doc_2logging documents CLogger.
- @ref md_doc_2testing__ci documents tests, CI, containers, ROS 2, and the
  downloadable documentation artifact.
- @ref md_doc_2ros2__overlay documents the optional ROS 2 overlay.
- @ref md_doc_2version__release documents version resolution and the canonical
  source TGZ release.

The `doc` preset generates HTML and XML locally. CI uploads those outputs as a
normal documentation artifact; it does not publish a website.
