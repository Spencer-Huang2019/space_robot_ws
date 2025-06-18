set(SOURCE_LIST
    src/custom_trajectory_publisher.cpp
    src/planning_node.cpp
)

add_executable(sp_planning ${SOURCE_LIST})

ament_target_dependencies(sp_planning
    rclcpp
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    trajectory_msgs
)

install(
    TARGETS sp_planning
    DESTINATION lib/${PROJECT_NAME}
)