/// @file conversions.h
/// @brief Converts generated SimulationGears build metadata into ROS payloads.

#pragma once

#include "simulation_gears_interfaces/msg/sample_build_status.hpp"
#include "simulation_gears_interfaces/srv/get_sample_build_info.hpp"

#include <cstdint>
#include <string>

namespace simulation_gears_ros {

/// @brief Immutable build metadata returned by the ROS sample seam.
class CSampleBuildInfo final {
 public:
  /// @brief Construct one build-info value.
  /// @param charVersion_ Strict core project version.
  /// @param charFullVersion_ Full project version including optional metadata.
  /// @param charState_ Deterministic runtime sample state.
  CSampleBuildInfo(
      std::string charVersion_,
      std::string charFullVersion_,
      std::string charState_);

  /// @return Strict core project version.
  const std::string& version() const noexcept;

  /// @return Full project version.
  const std::string& fullVersion() const noexcept;

  /// @return Deterministic runtime sample state.
  const std::string& state() const noexcept;

 private:
  std::string charVersion_;
  std::string charFullVersion_;
  std::string charState_;
};

/// @brief Read build metadata from the core package's generated config.h.
/// @return Build metadata with the deterministic active sample state.
CSampleBuildInfo ReadCoreBuildInfo();

/// @brief Convert build metadata into the public build-info service response.
/// @param objBuildInfo_ Core build metadata to expose.
/// @return Populated service response.
simulation_gears_interfaces::srv::GetSampleBuildInfo::Response
MakeGetSampleBuildInfoResponse(const CSampleBuildInfo& objBuildInfo_);

/// @brief Convert one serviced request into a status publication.
/// @param objBuildInfo_ Core build metadata to expose.
/// @param uiRequestCount_ Monotonic count including the serviced request.
/// @param objStamp_ ROS timestamp assigned by the lifecycle node.
/// @return Populated status message.
simulation_gears_interfaces::msg::SampleBuildStatus MakeSampleBuildStatus(
    const CSampleBuildInfo& objBuildInfo_,
    std::uint64_t uiRequestCount_,
    const simulation_gears_interfaces::msg::SampleBuildStatus::_stamp_type& objStamp_);

}  // namespace simulation_gears_ros
