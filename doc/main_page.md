# SimulationGears_for_SpaceNav {#mainpage}

See the [README](../../README.md) for full usage documentation, or read on for the condensed reference.

## Installation

```bash
git clone <repo-url> my_project && cd my_project
./build_lib.sh -t release -i      # build + install to ./install
```

## Common Build Toggles

```bash
# Enable CUDA + NVCC optimization toggles
./build_lib.sh -D ENABLE_CUDA=ON -D CUDA_ENABLE_FMAD=ON -D CUDA_ENABLE_EXTRA_DEVICE_VECTORIZATION=ON

# Enable oneTBB and explicit SIMD/FMA
./build_lib.sh -D ENABLE_TBB=ON -D CPU_ENABLE_SIMD=ON -D CPU_SIMD_LEVEL=avx2 -D CPU_ENABLE_FMA=ON

# Disable native tuning for portable binaries
./build_lib.sh -D CPU_ENABLE_NATIVE_TUNING=OFF
```

## Wrapper Build

```bash
# Python wrapper
./build_lib.sh -p

# Python + MATLAB wrappers
./build_lib.sh -p -m

# Use a local wrap checkout instead of installed gtwrap
./build_lib.sh -p --gtwrap-root /path/to/wrap
```

Install Python package manually from the source Python package:

```bash
cd python
python -m pip install .
```

## Example usage (assuming installation worked)

```cmake
set(SimulationGears_for_SpaceNav_DIR "/path/to/install/lib/cmake/SimulationGears_for_SpaceNav")
find_package(SimulationGears_for_SpaceNav REQUIRED)
target_link_libraries(my_target PRIVATE SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav)
```

See `examples/template_consumer_project/` for a complete downstream CMake project.

## Scaffold Notes

This repository is aligned with the current fresh-init template layout.
The placeholder C++ sources live under `src/template_src/`, `src/template_src_kernels/`,
`src/wrapped_impl/`, and `src/bin/` and can be replaced as the real implementation lands.

Full details in `README.md`.
