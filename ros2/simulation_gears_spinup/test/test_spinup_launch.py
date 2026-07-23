"""End-to-end launch tests for both SimulationGears lifecycle launch paths."""

from __future__ import annotations

import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_ros.actions import PushRosNamespace
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest
import rclpy
from rclpy.node import Node
from simulation_gears_interfaces.msg import SampleBuildStatus
from simulation_gears_interfaces.srv import GetSampleBuildInfo


@pytest.mark.launch_test
@launch_testing.parametrize(
    "charLaunchFile_, charNamespace_",
    [
        ("simulation_gears.launch.py", ""),
        ("simulation_gears_composition.launch.py", ""),
        ("simulation_gears.launch.py", "integration"),
        ("simulation_gears_composition.launch.py", "integration"),
    ],
)
def generate_test_description(
    charLaunchFile_: str,
    charNamespace_: str,
) -> LaunchDescription:
    """Launch one standalone/composable and root/namespaced test variant."""
    charPackageShare_ = get_package_share_directory("simulation_gears_spinup")
    objLaunchSource_ = PythonLaunchDescriptionSource(
        f"{charPackageShare_}/launch/{charLaunchFile_}"
    )
    objIncludeLaunch_ = IncludeLaunchDescription(objLaunchSource_)
    if charNamespace_:
        objLaunchAction_ = GroupAction(
            [PushRosNamespace(charNamespace_), objIncludeLaunch_]
        )
    else:
        objLaunchAction_ = objIncludeLaunch_

    return LaunchDescription([objLaunchAction_, ReadyToTest()])


class TestSpinupLaunch(unittest.TestCase):
    """Validate lifecycle, build-info service, and status count progression."""

    objNode_: Node

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()
        cls.objNode_ = rclpy.create_node("simulation_gears_spinup_launch_test")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.objNode_.destroy_node()
        rclpy.shutdown()

    def _waitForActive(
        self,
        charNodePath_: str,
        charCase_: str,
        dTimeoutSec_: float = 10.0,
    ) -> None:
        """Wait until the launched lifecycle node reaches the active state."""
        objStateClient_ = self.objNode_.create_client(
            GetState,
            f"{charNodePath_}/get_state",
        )
        self.assertTrue(
            objStateClient_.wait_for_service(timeout_sec=dTimeoutSec_),
            f"Lifecycle state service was unavailable for {charCase_}",
        )

        dDeadline_ = time.monotonic() + dTimeoutSec_
        uiLastState_ = State.PRIMARY_STATE_UNKNOWN
        while time.monotonic() < dDeadline_:
            objFuture_ = objStateClient_.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self.objNode_, objFuture_, timeout_sec=1.0)
            if objFuture_.done() and objFuture_.exception() is None:
                objResponse_ = objFuture_.result()
                self.assertIsNotNone(objResponse_)
                uiLastState_ = objResponse_.current_state.id
                if uiLastState_ == State.PRIMARY_STATE_ACTIVE:
                    return
            time.sleep(0.1)

        self.fail(
            f"Lifecycle node did not become active for {charCase_}; "
            f"last state was {uiLastState_}"
        )

    def testLaunchPathServesBuildInfoAndPublishesProgress(
        self,
        charLaunchFile_: str,
        charNamespace_: str,
    ) -> None:
        """Verify service payload and status request counts for one launch variant."""
        charNamespacePrefix_ = f"/{charNamespace_}" if charNamespace_ else ""
        charNodePath_ = f"{charNamespacePrefix_}/simulation_gears_sample"
        charCase_ = (
            f"launch={charLaunchFile_}, namespace={charNamespace_ or '<root>'}"
        )
        self._waitForActive(charNodePath_, charCase_)

        objBuildInfoClient_ = self.objNode_.create_client(
            GetSampleBuildInfo,
            f"{charNodePath_}/get_build_info",
        )
        self.assertTrue(
            objBuildInfoClient_.wait_for_service(timeout_sec=5.0),
            f"Build-info service was unavailable for {charCase_}",
        )

        listStatusMessages_: list[SampleBuildStatus] = []
        objStatusSubscription_ = self.objNode_.create_subscription(
            SampleBuildStatus,
            f"{charNodePath_}/status",
            listStatusMessages_.append,
            10,
        )
        try:
            dDiscoveryDeadline_ = time.monotonic() + 5.0
            while (
                objStatusSubscription_.get_publisher_count() == 0
                and time.monotonic() < dDiscoveryDeadline_
            ):
                rclpy.spin_once(self.objNode_, timeout_sec=0.1)
            self.assertGreater(
                objStatusSubscription_.get_publisher_count(),
                0,
                f"Status publisher was undiscovered for {charCase_}",
            )

            dDiscoverySettleDeadline_ = time.monotonic() + 0.5
            while time.monotonic() < dDiscoverySettleDeadline_:
                rclpy.spin_once(self.objNode_, timeout_sec=0.05)

            listResponses_: list[GetSampleBuildInfo.Response] = []
            for uiExpectedCount_ in (1, 2):
                objFuture_ = objBuildInfoClient_.call_async(
                    GetSampleBuildInfo.Request()
                )
                dResponseDeadline_ = time.monotonic() + 5.0
                while (
                    (
                        not objFuture_.done()
                        or not any(
                            objStatus_.request_count == uiExpectedCount_
                            for objStatus_ in listStatusMessages_
                        )
                    )
                    and time.monotonic() < dResponseDeadline_
                ):
                    rclpy.spin_once(self.objNode_, timeout_sec=0.1)

                self.assertTrue(objFuture_.done(), f"Response timed out for {charCase_}")
                self.assertIsNone(objFuture_.exception(), charCase_)
                objResponse_ = objFuture_.result()
                self.assertIsNotNone(objResponse_, charCase_)
                listResponses_.append(objResponse_)

            for objResponse_ in listResponses_:
                self.assertRegex(objResponse_.version, r"^\d+\.\d+\.\d+$", charCase_)
                self.assertTrue(
                    objResponse_.full_version == objResponse_.version
                    or objResponse_.full_version.startswith(
                        (f"{objResponse_.version}-", f"{objResponse_.version}+")
                    ),
                    charCase_,
                )
                self.assertEqual(objResponse_.status, "active", charCase_)

            self.assertEqual(listResponses_[0].version, listResponses_[1].version)
            self.assertEqual(
                listResponses_[0].full_version, listResponses_[1].full_version
            )

            listProgress_ = [
                objStatus_.request_count
                for objStatus_ in listStatusMessages_
                if objStatus_.request_count in (1, 2)
            ]
            self.assertEqual(listProgress_[-2:], [1, 2], charCase_)
            objLastStatus_ = listStatusMessages_[-1]
            self.assertEqual(objLastStatus_.version, listResponses_[-1].version, charCase_)
            self.assertEqual(objLastStatus_.state, "active", charCase_)
            self.assertTrue(
                objLastStatus_.stamp.sec > 0 or objLastStatus_.stamp.nanosec > 0,
                f"Status timestamp was not populated for {charCase_}",
            )
        finally:
            self.objNode_.destroy_subscription(objStatusSubscription_)
