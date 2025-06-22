# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_rsp_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("space_robot", package_name="space_robot_config").to_moveit_configs()
#     return generate_rsp_launch(moveit_config)

#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def print_descrition(context):
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('space_robot_config'), 'config', 'space_robot.urdf.xacro']),
            ' ',
            'use_gazebo:=',
            LaunchConfiguration('use_gazebo'),
        ]
    ).perform(context)

    print("\n" + "="*80)
    print("Generated robot_description URDF:")
    print("="*80)
    print(robot_description_content)
    print("="*80 + "\n")

    return []


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg('use_gazebo', default_value=True))

    # ld.add_action(OpaqueFunction(function=print_descrition))
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([FindPackageShare('space_robot_config'), 'config', 'space_robot.urdf.xacro']),
            ' ',
            'use_gazebo:=',
            LaunchConfiguration('use_gazebo'),
        ]
    )

    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        respawn=True,
        output='screen',
        parameters=[
            robot_description,
        ]
    )
    ld.add_action(robot_state_publisher_node)

    return ld