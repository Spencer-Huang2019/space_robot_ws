#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # ld.add_action(
    #     Node(
    #         package="controller_manager",
    #         executable="spawner",
    #         arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
    #         output="screen",
    #     )
    # )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=['space_robot_arm_controller', '--controller-manager', '/controller_manager'],
            output="screen",
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output="screen",
        )
    )

    return ld