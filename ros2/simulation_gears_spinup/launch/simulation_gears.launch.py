"""Launch the standalone SimulationGears lifecycle build-info sample."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description() -> LaunchDescription:
    """Configure and activate the standalone lifecycle node automatically."""
    charPackageShare_ = get_package_share_directory("simulation_gears_spinup")
    charParamsFile_ = os.path.join(
        charPackageShare_, "config", "simulation_gears.yaml"
    )

    return LaunchDescription(
        [
            LifecycleNode(
                package="simulation_gears_ros",
                executable="simulation_gears_node",
                name="simulation_gears_sample",
                namespace="",
                output="screen",
                parameters=[charParamsFile_],
                autostart=True,
            )
        ]
    )
