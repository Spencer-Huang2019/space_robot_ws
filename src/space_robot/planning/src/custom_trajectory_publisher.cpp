#include "custom_trajectory_publisher.h"


namespace space_robot {
namespace planning {

    CustomTrajectoryPublisher::CustomTrajectoryPublisher(const rclcpp::NodeOptions& options): Node("custom_trajectory_publisher", options)
    {
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this](){
                move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    shared_from_this(), "space_robot_arm"
                );

                traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                    "/custom_trajectory", 10
                );
                
                timer_->cancel();
                // timer_ = this->create_wall_timer(
                //     std::chrono::seconds(1),
                //     std::bind(&CustomTrajectoryPublisher::publish_custom_trajectory, this)
                // );
                this->publish_custom_trajectory();
                std::chrono::seconds(1);
            });
    }

    void CustomTrajectoryPublisher::publish_custom_trajectory()
    {
        std::vector<std::string> joint_names = move_group_->getJointNames();
        RCLCPP_INFO(this->get_logger(), "joing names: %s, %s", joint_names[0].c_str(), joint_names[1].c_str());
        // std::vector<double> current_joint_values = move_group_->getCurrentJointValues();
        std::vector<double> current_joint_values = {0.0, 0.0};
        

        auto trajectory = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
        trajectory->joint_names = joint_names;

        const int num_points = 10;
        const double duration = 5.0;

        for (int i = 0; i <= num_points; ++i)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds((duration * i) / num_points);

            for (size_t j = 0; j < joint_names.size(); ++j)
            {
                double amplitude = 0.5;
                double offset = current_joint_values[j];
                double value = offset + amplitude * sin(2.0 * M_PI * i / num_points);
                point.positions.push_back(value); 

                if (i == 0 || i == num_points)
                {
                    point.velocities.push_back(0.0);
                    point.accelerations.push_back(0.0);
                }
                else
                {
                    point.velocities.push_back(amplitude * 2.0 * M_PI / duration * 
                                             cos(2.0 * M_PI * i / num_points));
                    point.accelerations.push_back(-amplitude * pow(2.0 * M_PI / duration, 2) * 
                                                sin(2.0 * M_PI * i / num_points));
                }
            }
            trajectory->points.push_back(point);
        }

        traj_pub_->publish(*trajectory);
        RCLCPP_INFO(this->get_logger(), "Custom trajectory published");

        execute_trajectory(trajectory);
    }

    void CustomTrajectoryPublisher::execute_trajectory(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory>& trajectory)
    {
        moveit_msgs::msg::RobotTrajectory robot_trajectory;
        robot_trajectory.joint_trajectory = *trajectory;

        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);

        auto result = move_group_->execute(robot_trajectory);
        
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Trajectory executed");
        }
        else
        {

            RCLCPP_INFO(this->get_logger(), "Failed to execute trajectory");
        }
    }

}
}