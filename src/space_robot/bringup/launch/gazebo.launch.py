#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command,FindExecutable
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():
    pkg_path = get_package_share_directory('space_robot')

    ld = LaunchDescription()

    # gz server
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(
        pkg_path,
        'description',
        'sdf',
        'space_world.sdf'
    )
    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world], 
            'on_exit_shutdown': 'true'
        }.items()
    )
    ld.add_action(gzserver_node)

    # gz client
    gzclient_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items(),
    )
    ld.add_action(gzclient_node)

    start_gazebo_ros_spawner_node = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', '/robot_description',  # 必须的模型来源参数
            '-name', 'space_robot',
            '-allow_renaming', 'true'
        ]
    )
    ld.add_action(start_gazebo_ros_spawner_node)

    bridge_params = os.path.join(pkg_path, 'bringup', 'config', 'ros_gz_bridge.yaml')
    start_gazebo_ros_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}'
        ],
    )
    ld.add_action(start_gazebo_ros_bridge_node)

    start_gazebo_ros_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgbd_camera/image'],
        output='screen',
    )
    ld.add_action(start_gazebo_ros_image_bridge_node)

    start_gazebo_ros_depth_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgbd_camera/depth_image'],
        output='screen',
    )
    ld.add_action(start_gazebo_ros_depth_image_bridge_node)
   
    return ld