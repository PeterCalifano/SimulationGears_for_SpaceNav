cmake_minimum_required(VERSION 3.16)

project(simulation_gears_ros)

option(SIMULATION_GEARS_ENABLE_CUDA "Accepted for workspace build-helper compatibility." OFF)
option(SIMULATION_GEARS_ENABLE_OPTIX "Accepted for workspace build-helper compatibility." OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(SimulationGears_for_SpaceNav REQUIRED)
find_package(simulation_gears_interfaces REQUIRED)

add_library(simulation_gears_ros_conversions SHARED
  src/conversions.cpp
)
target_include_directories(simulation_gears_ros_conversions
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(simulation_gears_ros_conversions
  SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav
)
ament_target_dependencies(simulation_gears_ros_conversions
  simulation_gears_interfaces
)

add_library(simulation_gears_ros_component SHARED
  src/CSimulationGearsLifecycleNode.cpp
)
target_include_directories(simulation_gears_ros_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(simulation_gears_ros_component
  simulation_gears_ros_conversions
  SimulationGears_for_SpaceNav::SimulationGears_for_SpaceNav
)
ament_target_dependencies(simulation_gears_ros_component
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  simulation_gears_interfaces
)
rclcpp_components_register_nodes(simulation_gears_ros_component
  "simulation_gears_ros::CSimulationGearsLifecycleNode"
)

add_executable(simulation_gears_node src/simulation_gears_node_main.cpp)
target_link_libraries(simulation_gears_node
  simulation_gears_ros_component
)
ament_target_dependencies(simulation_gears_node
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_build_info_conversions test/test_build_info_conversions.cpp)
  if(TARGET test_build_info_conversions)
    target_link_libraries(test_build_info_conversions
      simulation_gears_ros_conversions
    )
  endif()
endif()

install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    simulation_gears_ros_conversions
    simulation_gears_ros_component
    simulation_gears_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  lifecycle_msgs
  SimulationGears_for_SpaceNav
  simulation_gears_interfaces
)
ament_package()
