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
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )
    gui = LaunchConfiguration('gui')

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use gazebo clock.',
        )
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_path = get_package_share_directory('space_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world = os.path.join(
        pkg_path,
        'description',
        'sdf',
        'space_world.sdf'
    )

    # add gazebo model source
    # set_env_vars_resources = AppendEnvironmentVariable(
    #     'GZ_SIM_RESOURCE_PATH',
    #     os.path.join(pkg_path,
    #                 'models')
    # )

    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v4 ', world], 
            'on_exit_shutdown': 'true',
            'use_sim_time': use_sim_time
        }.items()
    )

    gzclient_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items(),

    )

    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup', 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(gui)
    )

    start_gazebo_ros_spawner_node = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=[
            '-topic', '/robot_description',  # 必须的模型来源参数
            '-name', 'space_robot',
            '-allow_renaming', 'true'
        ]
    )

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

    start_gazebo_ros_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgbd_camera/image'],
        output='screen',
    )

    start_gazebo_ros_depth_image_bridge_node = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/rgbd_camera/depth_image'],
        output='screen',
    )

    start_gazebo_ros_point_cloud_bridge_node = Node(
        package='ros_gz_point_cloud',
        executable='pointcloud_bridge',
        arguments=['/rgbd_camera/points'],
        output='screen',
    )

    controller_config = PathJoinSubstitution([
        FindPackageShare('space_robot'),
        'bringup/config/controllers.yaml'
    ])

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': use_sim_time},  # 如果使用 Gazebo/Ignition 仿真，设为 True
            controller_config
        ],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # arguments=['joint_state_broadcaster'],
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_sim_time)
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        # arguments=['forward_position_controller'],
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_sim_time)
    )

    load_controllers_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                robot_controller_spawner,
                joint_state_broadcaster_spawner,
            ]
        )
    )


    nodes = [
        gzserver_node,
        gzclient_node,

        controller_manager_node,
        robot_state_publisher_node,
        start_gazebo_ros_spawner_node,

        load_controllers_handler,
        start_gazebo_ros_bridge_node,
        start_gazebo_ros_image_bridge_node,
        start_gazebo_ros_depth_image_bridge_node,
        # start_gazebo_ros_point_cloud_bridge_node
    ]
   
    return LaunchDescription(declared_arguments + nodes)