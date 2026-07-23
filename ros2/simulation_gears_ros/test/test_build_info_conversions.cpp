/// @file test_build_info_conversions.cpp
/// @brief Verifies the ROS build-info conversion seam against generated core metadata.

#include "simulation_gears_ros/conversions.h"

#include <config.h>
#include <gtest/gtest.h>

#include <cstdint>

TEST(SimulationGearsBuildInfoConversions, ReadsGeneratedCoreBuildMetadata) {
  const auto objBuildInfo_ = simulation_gears_ros::ReadCoreBuildInfo();

  EXPECT_EQ(objBuildInfo_.version(), PROJECT_VERSION);
  EXPECT_EQ(objBuildInfo_.fullVersion(), FULL_VERSION);
  EXPECT_EQ(objBuildInfo_.state(), "active");
}

TEST(SimulationGearsBuildInfoConversions, BuildsServiceAndStatusPayloads) {
  const auto objBuildInfo_ = simulation_gears_ros::ReadCoreBuildInfo();
  const auto objResponse_ =
      simulation_gears_ros::MakeGetSampleBuildInfoResponse(objBuildInfo_);

  EXPECT_EQ(objResponse_.version, PROJECT_VERSION);
  EXPECT_EQ(objResponse_.full_version, FULL_VERSION);
  EXPECT_EQ(objResponse_.status, "active");

  builtin_interfaces::msg::Time objStamp_;
  objStamp_.sec = 12;
  objStamp_.nanosec = 34U;
  constexpr std::uint64_t uiRequestCount_ = 2U;
  const auto objStatus_ = simulation_gears_ros::MakeSampleBuildStatus(
      objBuildInfo_, uiRequestCount_, objStamp_);

  EXPECT_EQ(objStatus_.stamp.sec, 12);
  EXPECT_EQ(objStatus_.stamp.nanosec, 34U);
  EXPECT_EQ(objStatus_.request_count, uiRequestCount_);
  EXPECT_EQ(objStatus_.version, PROJECT_VERSION);
  EXPECT_EQ(objStatus_.state, "active");
}
