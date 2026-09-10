cmake_minimum_required(VERSION 3.16)

project(simulation_gears_colcon_shim LANGUAGES NONE)

get_filename_component(SIMULATION_GEARS_REPOSITORY_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../.."
  ABSOLUTE)

option(SIMULATION_GEARS_ENABLE_CUDA "Enable CUDA support in the SimulationGears core." OFF)
option(SIMULATION_GEARS_ENABLE_OPTIX "Enable OptiX support in the SimulationGears core." OFF)

set(SimulationGears_for_SpaceNav_ENABLE_CUDA
    "${SIMULATION_GEARS_ENABLE_CUDA}" CACHE BOOL
    "Enable CUDA in the SimulationGears core." FORCE)
set(SimulationGears_for_SpaceNav_ENABLE_OPTIX
    "${SIMULATION_GEARS_ENABLE_OPTIX}" CACHE BOOL
    "Enable OptiX in the SimulationGears core." FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "Disable core tests in the ROS 2 overlay build." FORCE)
set(ENABLE_PYTHON_TESTS OFF CACHE BOOL "Disable core Python tests in the ROS 2 overlay build." FORCE)
set(ENABLE_FETCH_CATCH2 OFF CACHE BOOL "Do not fetch Catch2 for the ROS 2 overlay." FORCE)
set(ENABLE_SUBMODULES OFF CACHE BOOL "Keep MATLAB support submodules outside the ROS 2 overlay." FORCE)

# The nested core resolves its pre-project helpers relative to the repository.
# Keep that directory first without changing the standalone contract.
list(PREPEND CMAKE_MODULE_PATH "${SIMULATION_GEARS_REPOSITORY_ROOT}/cmake")

add_subdirectory(
  "${SIMULATION_GEARS_REPOSITORY_ROOT}"
  "${CMAKE_CURRENT_BINARY_DIR}/simulation_gears_core")
