#include <rclcpp/rclcpp.hpp>
#include "custom_trajectory_publisher.h"


using namespace space_robot::planning;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    CustomTrajectoryPublisherPtr node = std::make_shared<CustomTrajectoryPublisher>(rclcpp::NodeOptions());

    rclcpp::sleep_for(std::chrono::seconds(5));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}