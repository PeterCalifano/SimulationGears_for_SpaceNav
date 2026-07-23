/// @file CSimulationGearsLifecycleNode.h
/// @brief Declares the lifecycle-managed SimulationGears build-info sample node.

#pragma once

#include "simulation_gears_interfaces/msg/sample_build_status.hpp"
#include "simulation_gears_interfaces/srv/get_sample_build_info.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <cstdint>
#include <memory>

namespace simulation_gears_ros {

/// @brief Lifecycle node exposing generated core build metadata through ROS 2.
class CSimulationGearsLifecycleNode final
    : public rclcpp_lifecycle::LifecycleNode {
 public:
  /// @brief Construct the node with the required `simulation_gears_sample` name.
  /// @param objOptions_ ROS node construction options.
  explicit CSimulationGearsLifecycleNode(
      const rclcpp::NodeOptions& objOptions_ = rclcpp::NodeOptions());

  /// @brief Create the private service and lifecycle status publisher.
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& objPreviousState_) override;

  /// @brief Activate status publication.
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& objPreviousState_) override;

  /// @brief Deactivate status publication.
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& objPreviousState_) override;

  /// @brief Release ROS entities and reset the request counter.
  CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& objPreviousState_) override;

 private:
  using GetSampleBuildInfo =
      simulation_gears_interfaces::srv::GetSampleBuildInfo;
  using SampleBuildStatus =
      simulation_gears_interfaces::msg::SampleBuildStatus;

  void handleGetBuildInfo(
      const std::shared_ptr<GetSampleBuildInfo::Request> objRequest_,
      std::shared_ptr<GetSampleBuildInfo::Response> objResponse_);

  rclcpp_lifecycle::LifecyclePublisher<SampleBuildStatus>::SharedPtr
      objStatusPublisher_;
  rclcpp::Service<GetSampleBuildInfo>::SharedPtr objService_;
  std::uint64_t uiRequestCount_;
};

}  // namespace simulation_gears_ros
