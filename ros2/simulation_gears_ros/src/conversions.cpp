/// @file conversions.cpp
/// @brief Implements the core-to-ROS build-info conversion seam.

#include "simulation_gears_ros/conversions.h"

#include <config.h>

#include <utility>

namespace simulation_gears_ros {

CSampleBuildInfo::CSampleBuildInfo(
    std::string charVersion_,
    std::string charFullVersion_,
    std::string charState_)
    : charVersion_(std::move(charVersion_)),
      charFullVersion_(std::move(charFullVersion_)),
      charState_(std::move(charState_)) {}

const std::string& CSampleBuildInfo::version() const noexcept {
  return charVersion_;
}

const std::string& CSampleBuildInfo::fullVersion() const noexcept {
  return charFullVersion_;
}

const std::string& CSampleBuildInfo::state() const noexcept {
  return charState_;
}

CSampleBuildInfo ReadCoreBuildInfo() {
  return CSampleBuildInfo(PROJECT_VERSION, FULL_VERSION, "active");
}

simulation_gears_interfaces::srv::GetSampleBuildInfo::Response
MakeGetSampleBuildInfoResponse(const CSampleBuildInfo& objBuildInfo_) {
  simulation_gears_interfaces::srv::GetSampleBuildInfo::Response objResponse_;
  objResponse_.version = objBuildInfo_.version();
  objResponse_.full_version = objBuildInfo_.fullVersion();
  objResponse_.status = objBuildInfo_.state();
  return objResponse_;
}

simulation_gears_interfaces::msg::SampleBuildStatus MakeSampleBuildStatus(
    const CSampleBuildInfo& objBuildInfo_,
    std::uint64_t uiRequestCount_,
    const simulation_gears_interfaces::msg::SampleBuildStatus::_stamp_type& objStamp_) {
  simulation_gears_interfaces::msg::SampleBuildStatus objStatus_;
  objStatus_.stamp = objStamp_;
  objStatus_.request_count = uiRequestCount_;
  objStatus_.version = objBuildInfo_.version();
  objStatus_.state = objBuildInfo_.state();
  return objStatus_;
}

}  // namespace simulation_gears_ros
