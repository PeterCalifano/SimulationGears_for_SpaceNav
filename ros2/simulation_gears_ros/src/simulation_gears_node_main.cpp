/// @file simulation_gears_node_main.cpp
/// @brief Runs the standalone SimulationGears lifecycle sample node.

#include "simulation_gears_ros/CSimulationGearsLifecycleNode.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>

/// @brief Initialize ROS, spin the lifecycle node, and shut down cleanly.
/// @param argc Command-line argument count.
/// @param argv Command-line argument vector.
/// @return Zero after normal shutdown.
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<simulation_gears_ros::CSimulationGearsLifecycleNode>()
          ->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
