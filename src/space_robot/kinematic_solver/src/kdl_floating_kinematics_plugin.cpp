#include ".kdl_floating_kinematics_plugin.h"


namespace space_robot {
namespace kinematics_solver {
static rclcpp::Logger LOGGER = rclcpp::get_logger("space_robot_kinematics_plugin");

rclcpp::Clock KDLFloadingKinematicsPlugin::steady_clock_{ RCL_STEADY_TIME };

KDLFloadingKinematicsPlugin::KDLFloadingKinematicsPlugin(): initialized_(false)
{
}

void KDLFloadingKinematicsPlugin::get_random_configuration(Eigen::VectorXd& jnt_array) const
{
//   state_->setToRandomPositions(joint_model_group_);
//   state_->copyJointGroupPositions(joint_model_group_, &jnt_array[0]);
}

void KDLFloadingKinematicsPlugin::get_random_configuration(const Eigen::VectorXd& seed_state,
                                                const std::vector<double>& consistency_limits,
                                                Eigen::VectorXd& jnt_array) const
{
//   joint_model_group_->getVariableRandomPositionsNearBy(state_->getRandomNumberGenerator(), &jnt_array[0],
//                                                        &seed_state[0], consistency_limits);
}

bool KDLFloadingKinematicsPlugin::check_consistency(const Eigen::VectorXd& seed_state,
                                           const std::vector<double>& consistency_limits,
                                           const Eigen::VectorXd& solution) const
{
    for (std::size_t i = 0; i < dimension_; ++i)
        if (fabs(seed_state(i) - solution(i)) > consistency_limits[i])
            return false;
    return true;
}

void KDLFloadingKinematicsPlugin::get_joint_weights()
{
    const std::vector<std::string>& active_names = joint_model_group_->getActiveJointModelNames();
    std::vector<std::string> names;
    std::vector<double> weights;
    if (lookupParam(node_, "joint_weights.weights", weights, weights))
    {
        if (!lookupParam(node_, "joint_weights.names", names, names) || (names.size() != weights.size()))
        {
            RCLCPP_ERROR(LOGGER, "Expecting list parameter joint_weights.names of same size as list joint_weights.weights");
            // fall back to default weights
            weights.clear();
        }
    }
    else if (lookupParam(node_, "joint_weights", weights,
                        weights))  // try reading weight lists (for all active joints) directly
    {
        std::size_t num_active = active_names.size();
        if (weights.size() == num_active)
        {
            joint_weights_ = weights;
            return;
        }
        else if (!weights.empty())
        {
            RCLCPP_ERROR(LOGGER, "Expecting parameter joint_weights to list weights for all active joints (%zu) in order",
                        num_active);
            // fall back to default weights
            weights.clear();
        }
    }

    // by default assign weights of 1.0 to all joints
    joint_weights_ = std::vector<double>(active_names.size(), 1.0);
    if (weights.empty())  // indicates default case
        return;

    // modify weights of listed joints
    assert(names.size() == weights.size());
    for (size_t i = 0; i != names.size(); ++i)
    {
        auto it = std::find(active_names.begin(), active_names.end(), names[i]);
        if (it == active_names.cend())
            RCLCPP_WARN(LOGGER, "Joint '%s' is not an active joint in group '%s'", names[i].c_str(),
                        joint_model_group_->getName().c_str());
        else if (weights[i] < 0.0)
            RCLCPP_WARN(LOGGER, "Negative weight %f for joint '%s' will be ignored", weights[i], names[i].c_str());
        else
            joint_weights_[it - active_names.begin()] = weights[i];
    }
    RCLCPP_INFO_STREAM(
        LOGGER, "Joint weights for group '"
                    << getGroupName() << "': \n"
                    << Eigen::Map<const Eigen::VectorXd>(joint_weights_.data(), joint_weights_.size()).transpose());
}

bool KDLFloadingKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                const std::string& group_name, const std::string& base_frame,
                const std::vector<std::string>& tip_frames, double search_discretization)
{
    node_ = node;
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
    joint_model_group_ = robot_model_->getJointModelGroup(group_name);
    if (!joint_model_group_)
        return false;

    if (!joint_model_group_->isChain())
    {
        RCLCPP_ERROR(LOGGER, "Group '%s' is not a chain", group_name.c_str());
        return false;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(*robot_model.getURDF(), kdl_tree))
    {
        RCLCPP_ERROR(LOGGER, "Could not initialize chain object");
        return false;
    }
    if (!kdl_tree.getChain(base_frame_, getTipFrame(), kdl_chain_))
    {
        RCLCPP_ERROR(LOGGER, "Could not initialize chain object");
        return false;
    }

    // add floating joint for chain_root
    Segment base = kdl_chain_.getSegment(0);
    Joint floating_jnt = base.getJoint();


    dimension_ = joint_model_group_->getActiveJointModels().size() + joint_model_group_->getMimicJointModels().size();
    for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
    {
        if (joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::REVOLUTE ||
            joint_model_group_->getJointModels()[i]->getType() == moveit::core::JointModel::PRISMATIC)
        {
            solver_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
            const std::vector<moveit_msgs::msg::JointLimits>& jvec =
                joint_model_group_->getJointModels()[i]->getVariableBoundsMsg();
            solver_info_.limits.insert(solver_info_.limits.end(), jvec.begin(), jvec.end());
        }
    }

    // [TODO] add floating joint

    // [TODO] add tip frame in urdf

    if (!joint_model_group_->hasLinkModel(getTipFrame()))
    {
        RCLCPP_ERROR(LOGGER, "Could not find tip name in joint group '%s'", group_name.c_str());
        return false;
    }
    solver_info_.link_names.push_back(getTipFrame());

    joint_min_.resize(solver_info_.limits.size());
    joint_max_.resize(solver_info_.limits.size());

    for (unsigned int i = 0; i < solver_info_.limits.size(); ++i)
    {
        joint_min_(i) = solver_info_.limits[i].min_position;
        joint_max_(i) = solver_info_.limits[i].max_position;
    }

    // Get Solver Parameters
    lookupParam(node_, "max_solver_iterations", max_solver_iterations_, 500);
    lookupParam(node_, "epsilon", epsilon_, 1e-5);
    lookupParam(node_, "orientation_vs_position", orientation_vs_position_weight_, 1.0);

    bool position_ik;
    lookupParam(node_, "position_only_ik", position_ik, false);
    if (position_ik)  // position_only_ik overrules orientation_vs_position
        orientation_vs_position_weight_ = 0.0;
    if (orientation_vs_position_weight_ == 0.0)
        RCLCPP_INFO(LOGGER, "Using position only ik");

    get_joint_weights();

    // Setup the joint state groups that we need
    state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

    fk_solver_ = std::make_unique<FloatingChainFkSolver>(kdl_chain_);

    initialized_ = true;
    RCLCPP_DEBUG(LOGGER, "KDL solver initialized");
    return true;

}

bool KDLFloadingKinematicsPlugin::timed_out(const rclcpp::Time& start_time, double duration) const
{
  return ((steady_clock_.now() - start_time).seconds() >= duration);
}

bool KDLFloadingKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options) const 
{
    std::vector<double> consistency_limits;

    // limit search to a single attempt by setting a timeout of zero
    return searchPositionIK(ik_pose, ik_seed_state, 0.0, consistency_limits, solution, IKCallbackFn(), error_code,
                            options);
}

bool KDLFloadingKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options) const
{
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                            options);
}

bool KDLFloadingKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::msg::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options) const
{
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                            options);
}

bool KDLFloadingKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback,
                   moveit_msgs::msg::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options) const
{
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                            options);
}

bool KDLFloadingKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options) const
{
    const rclcpp::Time start_time = steady_clock_.now();
    if (!initialized_)
    {
        RCLCPP_ERROR(LOGGER, "kinematics solver not initialized");
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    if (ik_seed_state.size() != dimension_)
    {
        RCLCPP_ERROR(LOGGER, "Seed state must have size %d instead of size %zu\n", dimension_, ik_seed_state.size());
        error_code.val = error_code.NO_IK_SOLUTION;
        return false;
    }

    Eigen::Matrix<double, 6, 1> cartesian_weights;
    cartesian_weights.topRows<3>().setConstant(1.0);
    cartesian_weights.bottomRows<3>().setConstant(orientation_vs_position_weight_);

    common::JntArray jnt_seed_state(dimension_);
    common::JntArray jnt_pos_in(dimension_);
    common::JntArray jnt_pos_out(dimension_);
    jnt_seed_state.data = Eigen::Map<const Eigen::VectorXd>(ik_seed_state.data(), ik_seed_state.size());
    jnt_pos_in = jnt_seed_state;

    ChainIkSolverVel ik_solver_vel(kdl_chain_);
    solution.resize(dimension_);

    KDL::Frame pose_desired;
    tf2::fromMsg(ik_pose, pose_desired);

    RCLCPP_DEBUG_STREAM(LOGGER, "searchPositionIK: Position request pose is "
                                    << ik_pose.position.x << " " << ik_pose.position.y << " " << ik_pose.position.z << " "
                                    << ik_pose.orientation.x << " " << ik_pose.orientation.y << " "
                                    << ik_pose.orientation.z << " " << ik_pose.orientation.w);

    unsigned int attempt = 0;
    do
    {
        ++attempt;
        if (attempt > 1)  // randomly re-seed after first attempt
        {
            if (!consistency_limits_mimic.empty())
                getRandomConfiguration(jnt_seed_state.data, consistency_limits_mimic, jnt_pos_in.data);
            else
                getRandomConfiguration(jnt_pos_in.data);
            RCLCPP_DEBUG_STREAM(LOGGER, "New random configuration (" << attempt << "): " << jnt_pos_in);
        }

        int ik_valid =
            CartToJnt(ik_solver_vel, jnt_pos_in, pose_desired, jnt_pos_out, max_solver_iterations_,
                    Eigen::Map<const Eigen::VectorXd>(joint_weights_.data(), joint_weights_.size()), cartesian_weights);
        if (ik_valid == 0 || options.return_approximate_solution)  // found acceptable solution
        {
            if (!consistency_limits_mimic.empty() &&
                !checkConsistency(jnt_seed_state.data, consistency_limits_mimic, jnt_pos_out.data))
                continue;

            Eigen::Map<Eigen::VectorXd>(solution.data(), solution.size()) = jnt_pos_out.data;
            if (solution_callback)
            {
                solution_callback(ik_pose, solution, error_code);
                if (error_code.val != error_code.SUCCESS)
                continue;
            }

            // solution passed consistency check and solution callback
            error_code.val = error_code.SUCCESS;
            RCLCPP_DEBUG_STREAM(LOGGER, "Solved after " << (steady_clock_.now() - start_time).seconds() << " < " << timeout
                                                        << "s and " << attempt << " attempts");
            return true;
        }
    } while (!timedOut(start_time, timeout));

    RCLCPP_DEBUG_STREAM(LOGGER, "IK timed out after " << (steady_clock_.now() - start_time).seconds() << " > " << timeout
                                                        << "s and " << attempt << " attempts");
    error_code.val = error_code.TIMED_OUT;
    return false;
}

bool KDLFloadingKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::msg::Pose>& poses) const
{
    if (!initialized_)
    {
        RCLCPP_ERROR(LOGGER, "kinematics solver not initialized");
        return false;
    }
    poses.resize(link_names.size());
    if (joint_angles.size() != dimension_)
    {
        RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", dimension_);
        return false;
    }

    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in(dimension_);
    jnt_pos_in.data = Eigen::Map<const Eigen::VectorXd>(joint_angles.data(), joint_angles.size());

    bool valid = true;
    for (unsigned int i = 0; i < poses.size(); ++i)
    {
        if (fk_solver_->JntToCart(jnt_pos_in, p_out) >= 0)
        {
            poses[i] = tf2::toMsg(p_out);
        }
        else
        {
            CPP_ERROR(LOGGER, "Could not compute FK for %s", link_names[i].c_str());
            valid = false;
        }
    }
    return valid;
}


const std::vector<std::string>& KDLFloadingKinematicsPlugin::getJointNames() const
{
  return solver_info_.joint_names;
}

const std::vector<std::string>& KDLFloadingKinematicsPlugin::getLinkNames() const
{
    return solver_info_.link_names;
}

int KDLFloadingKinematicsPlugin::CartToJnt(FloatingChainIkSolverVel& ik_solver, const common::JntArray& q_init, const common::Frame& p_in,
            common::JntArray& q_out, const unsigned int max_iter, const Eigen::VectorXd& joint_weights,
            const Twist& cartesian_weights) const
{
    double last_delta_twist_norm = DBL_MAX;
    double step_size = 1.0;
    KDL::Frame f;
    KDL::Twist delta_twist;
    KDL::JntArray delta_q(q_out.rows()), q_backup(q_out.rows());
    Eigen::ArrayXd extra_joint_weights(joint_weights.rows());
    extra_joint_weights.setOnes();

    q_out = q_init;
    RCLCPP_DEBUG_STREAM(LOGGER, "Input: " << q_init);

    unsigned int i;
    bool success = false;
    for (i = 0; i < max_iter; ++i)
    {
        fk_solver_->JntToCart(q_out, f);
        delta_twist = diff(f, p_in);
        RCLCPP_DEBUG_STREAM(LOGGER, "[" << std::setw(3) << i << "] delta_twist: " << delta_twist);

        // check norms of position and orientation errors
        const double position_error = delta_twist.vel.Norm();
        const double orientation_error = ik_solver.isPositionOnly() ? 0 : delta_twist.rot.Norm();
        const double delta_twist_norm = std::max(position_error, orientation_error);
        if (delta_twist_norm <= epsilon_)
        {
            success = true;
            break;
        }

        if (delta_twist_norm >= last_delta_twist_norm)
        {
            // if the error increased, we are close to a singularity -> reduce step size
            double old_step_size = step_size;
            step_size *= std::min(0.2, last_delta_twist_norm / delta_twist_norm);  // reduce scale;
            KDL::Multiply(delta_q, step_size / old_step_size, delta_q);
            RCLCPP_DEBUG(LOGGER, "      error increased: %f -> %f, scale: %f", last_delta_twist_norm, delta_twist_norm,
                        step_size);
            q_out = q_backup;  // restore previous unclipped joint values
        }
        else
        {
            q_backup = q_out;  // remember joint values of last successful step
            step_size = 1.0;   // reset step size
            last_delta_twist_norm = delta_twist_norm;

            ik_solver.CartToJnt(q_out, delta_twist, delta_q, extra_joint_weights * joint_weights.array(), cartesian_weights);
        }

        clipToJointLimits(q_out, delta_q, extra_joint_weights);

        const double delta_q_norm = delta_q.data.lpNorm<1>();
        RCLCPP_DEBUG(LOGGER, "[%3d] pos err: %f  rot err: %f  delta_q: %f", i, position_error, orientation_error,
                    delta_q_norm);
        if (delta_q_norm < epsilon_)  // stuck in singularity
        {
            if (step_size < epsilon_)  // cannot reach target
                break;
            // wiggle joints
            last_delta_twist_norm = DBL_MAX;
            delta_q.data.setRandom();
            delta_q.data *= std::min(0.1, delta_twist_norm);
            clipToJointLimits(q_out, delta_q, extra_joint_weights);
            extra_joint_weights.setOnes();
        }

        KDL::Add(q_out, delta_q, q_out);

        RCLCPP_DEBUG_STREAM(LOGGER, "      delta_q: " << delta_q);
        RCLCPP_DEBUG_STREAM(LOGGER, "      q: " << q_out);
    }

    int result = (i == max_iter) ? -3 : (success ? 0 : -2);
    RCLCPP_DEBUG_STREAM(LOGGER, "Result " << result << " after " << i << " iterations: " << q_out);

    return result;
}

void KDLFloadingKinematicsPlugin::clip_to_joint_limits(const conmmon::JntArray& q, common::JntArray& q_delta, Eigen::ArrayXd& weighting) const
{
    weighting.setOnes();
    for (std::size_t i = 0; i < q.rows(); ++i)
    {
        const double delta_max = joint_max_(i) - q(i);
        const double delta_min = joint_min_(i) - q(i);
        if (q_delta(i) > delta_max)
            q_delta(i) = delta_max;
        else if (q_delta(i) < delta_min)
            q_delta(i) = delta_min;
        else
            continue;

        weighting[mimic_joints_[i].map_index] = 0.01;
    }
}


#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(space_robot::kinematic_solver::KDLFloadingKinematicsPlugin, kinematics::KinematicsBase)

}
}