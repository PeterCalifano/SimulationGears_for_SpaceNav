/// @file CSimulationGearsLifecycleNode.cpp
/// @brief Implements lifecycle wiring for the SimulationGears build-info sample.

#include "simulation_gears_ros/CSimulationGearsLifecycleNode.h"

#include "simulation_gears_ros/conversions.h"

#include <rclcpp_components/register_node_macro.hpp>

#include <utility>

namespace simulation_gears_ros {

CSimulationGearsLifecycleNode::CSimulationGearsLifecycleNode(
    const rclcpp::NodeOptions& objOptions_)
    : rclcpp_lifecycle::LifecycleNode("simulation_gears_sample", objOptions_),
      uiRequestCount_(0U) {}

CSimulationGearsLifecycleNode::CallbackReturn
CSimulationGearsLifecycleNode::on_configure(const rclcpp_lifecycle::State&) {
  uiRequestCount_ = 0U;

  objStatusPublisher_ =
      create_publisher<SampleBuildStatus>("~/status", rclcpp::QoS(10));
  objService_ = create_service<GetSampleBuildInfo>(
      "~/get_build_info",
      [this](const std::shared_ptr<GetSampleBuildInfo::Request> objRequest_,
             std::shared_ptr<GetSampleBuildInfo::Response> objResponse_) {
        handleGetBuildInfo(objRequest_, std::move(objResponse_));
      });

  RCLCPP_INFO(get_logger(), "Configured SimulationGears build-info sample");
  return CallbackReturn::SUCCESS;
}

CSimulationGearsLifecycleNode::CallbackReturn
CSimulationGearsLifecycleNode::on_activate(const rclcpp_lifecycle::State&) {
  if (objStatusPublisher_) {
    objStatusPublisher_->on_activate();
  }
  RCLCPP_INFO(get_logger(), "Activated SimulationGears build-info sample");
  return CallbackReturn::SUCCESS;
}

CSimulationGearsLifecycleNode::CallbackReturn
CSimulationGearsLifecycleNode::on_deactivate(const rclcpp_lifecycle::State&) {
  if (objStatusPublisher_) {
    objStatusPublisher_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "Deactivated SimulationGears build-info sample");
  return CallbackReturn::SUCCESS;
}

CSimulationGearsLifecycleNode::CallbackReturn
CSimulationGearsLifecycleNode::on_cleanup(const rclcpp_lifecycle::State&) {
  objService_.reset();
  objStatusPublisher_.reset();
  uiRequestCount_ = 0U;
  RCLCPP_INFO(get_logger(), "Cleaned up SimulationGears build-info sample");
  return CallbackReturn::SUCCESS;
}

void CSimulationGearsLifecycleNode::handleGetBuildInfo(
    const std::shared_ptr<GetSampleBuildInfo::Request>,
    std::shared_ptr<GetSampleBuildInfo::Response> objResponse_) {
  const auto objBuildInfo_ = ReadCoreBuildInfo();
  ++uiRequestCount_;

  *objResponse_ = MakeGetSampleBuildInfoResponse(objBuildInfo_);
  if (objStatusPublisher_ && objStatusPublisher_->is_activated()) {
    objStatusPublisher_->publish(MakeSampleBuildStatus(
        objBuildInfo_, uiRequestCount_, get_clock()->now()));
  }
}

}  // namespace simulation_gears_ros

RCLCPP_COMPONENTS_REGISTER_NODE(
    simulation_gears_ros::CSimulationGearsLifecycleNode)
