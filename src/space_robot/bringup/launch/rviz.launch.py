#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    pkg_path = get_package_share_directory('space_robot')

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(pkg_path, 'bringup/config/space_robot.rviz'),
        )
    )

    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    return ld