cmake_minimum_required(VERSION 3.16)

project(simulation_gears_spinup)

option(SIMULATION_GEARS_ENABLE_CUDA "Accepted for workspace build-helper compatibility." OFF)
option(SIMULATION_GEARS_ENABLE_OPTIX "Accepted for workspace build-helper compatibility." OFF)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_spinup_launch.py
    TIMEOUT 60
  )
endif()

ament_package()
