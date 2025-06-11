from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command, FindExecutable
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import SetEnvironmentVariable

import os


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        SetEnvironmentVariable(name='ROS_USE_SIM_TIME', value='1')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    gui = LaunchConfiguration("gui")

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use gazebo clock.",
        )
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = get_package_share_directory('space_robot')
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        # 当其他节点提供 joint_states 时，添加以下行：
        # remappings=[('/joint_states', '/dummy_joint_states')]  # 重映射以避免冲突
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # parameters=[{'use_sim_time': use_sim_time}],
        # remappings=[
        #     ('/joint_states', '/forward_position_controller/commands')
        # ],
        output=['screen']
    )

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup', 'launch', 'gazebo.launch.py')),
            launch_arguments={'gui': gui, 'use_sim_time': use_sim_time}.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("space_robot"), 'bringup', 'config', 'space_robot.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(gui)
    )

    nodes = [
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        gz_sim_node,
        rviz_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)