#ifndef PLANNING_CUSTOM_TRAJ_PUBLISHER_H
#define PLANNING_CUSTOM_TRAJ_PUBLISHER_H

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "macro.h"


namespace space_robot {
namespace planning {

CLASS_FORWARD(CustomTrajectoryPublisher);
class CustomTrajectoryPublisher: public rclcpp::Node
{
public:
    explicit CustomTrajectoryPublisher(const rclcpp::NodeOptions& options);

    void publish_custom_trajectory();

    void execute_trajectory(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory);

private:
    moveit::planning_interface::MoveGroupInterfacePtr move_group_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}
}

#endif