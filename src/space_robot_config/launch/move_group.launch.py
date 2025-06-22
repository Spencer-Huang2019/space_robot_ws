# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_move_group_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("space_robot", package_name="space_robot_config").to_moveit_configs()
#     return generate_move_group_launch(moveit_config)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path
import os


def generate_launch_description():
    urdf_path = Path(get_package_share_directory("space_robot_config")) / "config" / "space_robot.urdf.xacro"
    moveit_config = (
        MoveItConfigsBuilder("space_robot", package_name="space_robot_config")
        .robot_description(
            file_path=str(urdf_path),
            mappings={"use_gazebo": LaunchConfiguration('use_gazebo')}  # 传递参数
        )
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    moveit_params.update({"use_sim_time": LaunchConfiguration('use_gazebo')})

    ld = LaunchDescription()

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_params,
        move_group_configuration,
    ]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    return ld