cmake_minimum_required(VERSION 3.16)

project(simulation_gears_colcon_shim LANGUAGES NONE)

get_filename_component(SIMULATION_GEARS_REPOSITORY_ROOT
  "${CMAKE_CURRENT_LIST_DIR}/../.."
  ABSOLUTE
)

option(SIMULATION_GEARS_ENABLE_CUDA "Enable CUDA support in the SimulationGears core." OFF)
option(SIMULATION_GEARS_ENABLE_OPTIX "Enable OptiX support in the SimulationGears core." OFF)

set(ENABLE_CUDA "${SIMULATION_GEARS_ENABLE_CUDA}" CACHE BOOL "Enable CUDA in the SimulationGears core." FORCE)
set(ENABLE_OPTIX "${SIMULATION_GEARS_ENABLE_OPTIX}" CACHE BOOL "Enable OptiX in the SimulationGears core." FORCE)
set(ENABLE_TESTS OFF CACHE BOOL "Disable core tests in the ROS 2 overlay build." FORCE)
set(ENABLE_PYTHON_TESTS OFF CACHE BOOL "Disable core Python tests in the ROS 2 overlay build." FORCE)
set(ENABLE_FETCH_CATCH2 OFF CACHE BOOL "Do not fetch Catch2 for the ROS 2 overlay." FORCE)
set(ENABLE_SUBMODULES OFF CACHE BOOL "Keep MATLAB support submodules outside the ROS 2 overlay." FORCE)

# The root project appends PROJECT_SOURCE_DIR/cmake before its project() call,
# which initially points at this shim. Keep the real helper directory first.
list(PREPEND CMAKE_MODULE_PATH "${SIMULATION_GEARS_REPOSITORY_ROOT}/cmake")

add_subdirectory(
  "${SIMULATION_GEARS_REPOSITORY_ROOT}"
  "${CMAKE_CURRENT_BINARY_DIR}/simulation_gears_core"
)

# Mirror a legacy top-level build VERSION file into the nested project directory
# expected by the core install rule without changing the standalone project.
set(_ros2_overlay_legacy_version_file "${CMAKE_BINARY_DIR}/VERSION")
set(_ros2_overlay_nested_version_file
    "${CMAKE_CURRENT_BINARY_DIR}/simulation_gears_core/VERSION")
if(EXISTS "${_ros2_overlay_legacy_version_file}"
   AND NOT EXISTS "${_ros2_overlay_nested_version_file}")
  configure_file(
    "${_ros2_overlay_legacy_version_file}"
    "${_ros2_overlay_nested_version_file}"
    COPYONLY
  )
endif()
unset(_ros2_overlay_legacy_version_file)
unset(_ros2_overlay_nested_version_file)
