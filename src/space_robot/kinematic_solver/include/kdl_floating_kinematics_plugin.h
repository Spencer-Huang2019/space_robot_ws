#ifndef KDL_FLOATING_KINEMATICS_PLUGIN_H
#define KDL_FLOATING_KINEMATICS_PLUGIN_H

#pragma
#include <rclcpp/rclcpp.hpp>
#include <random_numbers/random_numbers.h>


#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/kinematic_solver_info.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <cfloat>

#include ".macro.h"
#include ".KDL_structs.h"


namespace space_robot {
namespace kinematics_solver {

CLASS_FORWARD(KDLFloadingKinematicsPlugin);
class KDLFloadingKinematicsPlugin : public moveit::kinematics::KinematicsBase 
{
public:

    KDLFloadingKinematicsPlugin();

    bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                    std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                    const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                    std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                    const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                    const std::vector<double>& consistency_limits, std::vector<double>& solution,
                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                    const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                    std::vector<double>& solution, const IKCallbackFn& solution_callback,
                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                    const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                    const std::vector<double>& consistency_limits, std::vector<double>& solution,
                    const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                    const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

    bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                std::vector<geometry_msgs::msg::Pose>& poses) const override;

    bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                    const std::string& group_name, const std::string& base_frame,
                    const std::vector<std::string>& tip_frames, double search_discretization) override;

    const std::vector<std::string>& getJointNames() const override;

    const std::vector<std::string>& getLinkNames() const override;

protected:
    typedef Eigen::Matrix<double, 6, 1> Twist;

    int cart_to_joint(FloatingChainIkSolverVel& ik_solver, const KDL::JntArray& q_init, const KDL::Frame& p_in,
                KDL::JntArray& q_out, const unsigned int max_iter, const Eigen::VectorXd& joint_weights,
                const Twist& cartesian_weights) const;

private:
    void get_joint_weights();

    bool timed_out(const rclcpp::Time& start_time, double duration) const;

    bool check_consistency(const Eigen::VectorXd& seed_state, const std::vector<double>& consistency_limits,
                        const Eigen::VectorXd& solution) const;

    void get_random_configuration(Eigen::VectorXd& jnt_array) const;

    void get_random_configuration(const Eigen::VectorXd& seed_state, const std::vector<double>& consistency_limits,
                              Eigen::VectorXd& jnt_array) const;

    void clip_to_joint_limits(const conmmon::JntArray& q, KDL::JntArray& q_delta, Eigen::ArrayXd& weighting) const;

private:
    static rclcpp::Clock steady_clock_;
    bool initialized_;

    unsigned int dimension_;
    moveit_msgs::msg::KinematicSovlerInfo solver_info_;

    const moveit::core::JointModelGroup* joint_model_group_;
    moveit::core::RobotStatePtr state_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<ChainFkSovlerPose> fk_solver_;
    std::vector<double> joint_weights_;
    Eigen::VectorXd& joint_min_, joint_max_;

    int max_solver_iterations_;
    double epsilon_;
    double orientation_vs_position_weight_;

};
}
}

#endif