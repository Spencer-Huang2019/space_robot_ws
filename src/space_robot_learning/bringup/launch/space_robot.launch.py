from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg

import os


def generate_launch_description():
    pkg_path = get_package_share_directory('space_robot_learning')

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_rviz",
            default_value=True
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "publish_frequency",
            default_value="15.0"
        )
    )

    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_gazebo",
            default_value=True
        )
    )
    
    # robot_state_publisher
    robot_state_publisher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup/launch/rps.launch.py')
        ),
        launch_arguments={'publish_frequency': LaunchConfiguration('publish_frequency')}.items(),
    )
    ld.add_action(robot_state_publisher_node)

    # ros2 control, Fake joint driver
    controller_config = PathJoinSubstitution([
        FindPackageShare('space_robot_learning'),
        'bringup/config/controllers.yaml'
    ])
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            controller_config
        ],
        output='both',
    )
    ld.add_action(controller_manager_node)

    # ros2 control, spawn controllers
    spawn_controllers_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup/launch/spawn_controllers.launch.py')
        )
    )
    ld.add_action(spawn_controllers_node)

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup', 'launch', 'gazebo.launch.py')),
            condition=IfCondition(LaunchConfiguration("use_gazebo"))
    )
    ld.add_action(gz_sim_node)

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'bringup/launch/rviz.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    ld.add_action(rviz_node)

    return ld