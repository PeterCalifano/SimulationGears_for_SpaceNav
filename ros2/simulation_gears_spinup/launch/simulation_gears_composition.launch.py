"""Launch the composable SimulationGears lifecycle build-info sample."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import ComposableNodeContainer, LifecycleTransition
from launch_ros.descriptions import (
    ComposableLifecycleNode as _RosComposableLifecycleNode,
)
from launch_ros.utilities import (
    LifecycleEventManager,
    make_namespace_absolute,
    prefix_namespace,
)
from lifecycle_msgs.msg import Transition


class _LifecycleNodeIdentity:
    """Expose the fully qualified name expected by Jazzy's lifecycle manager."""

    def __init__(self, charFullyQualifiedName_: str) -> None:
        self.charFullyQualifiedName_ = charFullyQualifiedName_

    @property
    def node_name(self) -> str:
        """Return the fully qualified lifecycle node name."""
        return self.charFullyQualifiedName_


class ComposableLifecycleNode(_RosComposableLifecycleNode):
    """Apply Jazzy-compatible autostart identity handling to one component."""

    def __init__(self, *, autostart: bool = False, **kwargs: object) -> None:
        self.bAutostart_ = autostart
        self.charFullyQualifiedName_ = ""

        # Jazzy joins composed autostart namespaces without a separator. The
        # local action below replaces only that transition identity handling.
        super().__init__(autostart=False, **kwargs)

    def init_lifecycle_event_manager(self, objContext_: LaunchContext) -> None:
        """Initialize the lifecycle manager with an absolute component name."""
        charNodeName_ = perform_substitutions(objContext_, self.node_name)
        charNodeNamespace_ = ""
        if self.node_namespace is not None:
            charNodeNamespace_ = perform_substitutions(
                objContext_, self.node_namespace
            )

        charBaseNamespace_ = objContext_.launch_configurations.get(
            "ros_namespace", None
        )
        charCombinedNamespace_ = make_namespace_absolute(
            prefix_namespace(charBaseNamespace_, charNodeNamespace_)
        )
        self.charFullyQualifiedName_ = (
            prefix_namespace(charCombinedNamespace_, charNodeName_) or charNodeName_
        )
        if not self.charFullyQualifiedName_.startswith("/"):
            self.charFullyQualifiedName_ = f"/{self.charFullyQualifiedName_}"

        # Jazzy launch_ros#481 otherwise matches a relative component identity.
        self.objLifecycleEventManager_ = LifecycleEventManager(
            _LifecycleNodeIdentity(self.charFullyQualifiedName_)
        )
        self.objLifecycleEventManager_.setup_lifecycle_manager(objContext_)

    def makeAutostartAction(self) -> OpaqueFunction:
        """Return the configure-then-activate action for this component."""
        return OpaqueFunction(function=self._autostart)

    def _autostart(
        self, objContext_: LaunchContext
    ) -> list[LifecycleTransition]:
        """Create ordered configure and activate lifecycle transitions."""
        if not self.bAutostart_:
            return []

        self.init_lifecycle_event_manager(objContext_)
        return [
            LifecycleTransition(
                lifecycle_node_names=[self.charFullyQualifiedName_],
                transition_ids=[
                    Transition.TRANSITION_CONFIGURE,
                    Transition.TRANSITION_ACTIVATE,
                ],
            )
        ]


def generate_launch_description() -> LaunchDescription:
    """Load, configure, and activate the composable lifecycle node."""
    charPackageShare_ = get_package_share_directory("simulation_gears_spinup")
    charParamsFile_ = os.path.join(
        charPackageShare_, "config", "simulation_gears.yaml"
    )
    objLifecycleNode_ = ComposableLifecycleNode(
        package="simulation_gears_ros",
        plugin="simulation_gears_ros::CSimulationGearsLifecycleNode",
        name="simulation_gears_sample",
        parameters=[charParamsFile_],
        autostart=True,
    )

    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="simulation_gears_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[objLifecycleNode_],
                output="screen",
            ),
            objLifecycleNode_.makeAutostartAction(),
        ]
    )
