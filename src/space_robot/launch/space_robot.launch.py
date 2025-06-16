from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from moveit_configs_utils import MoveItConfigsBuilder
from pathlib import Path

import os


def generate_launch_description():
    pkg_path = get_package_share_directory('space_robot')

    ld = LaunchDescription()

    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_rviz",
            default_value=False
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_gazebo",
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
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        Path(pkg_path) / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, "launch/rsp.launch.py")
            ),
            launch_arguments={
                'publish_frequency': LaunchConfiguration('publish_frequency'),
                'use_gazebo': LaunchConfiguration('use_gazebo'),
            }.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                # moveit_config.robot_description,
                os.path.join(pkg_path, "config/ros2_controllers.yaml"),
            ],
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_path, "launch/spawn_controllers.launch.py")
            ),
        )
    )

    gz_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'gazebo.launch.py')),
            # launch_arguments={'gui': gui, 'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(LaunchConfiguration("use_gazebo"))
    )
    ld.add_action(gz_sim_node)

    return ld