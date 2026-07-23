cmake_minimum_required(VERSION 3.16)

project(simulation_gears_interfaces)

option(SIMULATION_GEARS_ENABLE_CUDA "Accepted for workspace build-helper compatibility." OFF)
option(SIMULATION_GEARS_ENABLE_OPTIX "Accepted for workspace build-helper compatibility." OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SampleBuildStatus.msg"
  "srv/GetSampleBuildInfo.srv"
  DEPENDENCIES
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
