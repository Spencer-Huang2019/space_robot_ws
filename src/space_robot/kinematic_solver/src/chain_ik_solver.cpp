#include ".chain_ik_solver.h"

namespace space_robot {
namespace kinematics_sovler {

int FloatingChainIkSolverVel::CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out)
{
    // Let the ChainJntToJacSolver calculate the Jacobian for the current joint positions q_in.
    if (num_mimic_joints_ > 0)
    {
        jnt2jac_.JntToJac(q_in, jac_);
        // Now compute the actual jacobian that involves only the active DOFs
        jacToJacReduced(jac_, jac_reduced_);
    }
    else
        jnt2jac_.JntToJac(q_in, jac_reduced_);

    // weight Jacobian
    auto& jac = jac_reduced_.data;
    const Eigen::Index rows = svd_.rows();  // only operate on position rows?
    jac.topRows(rows) *= joint_weights.asDiagonal();
    jac.topRows(rows).transpose() *= cartesian_weights.topRows(rows).asDiagonal();

    // transform v_in to 6D Eigen::Vector
    Eigen::Matrix<double, 6, 1> vin;
    vin.topRows<3>() = Eigen::Map<const Eigen::Array3d>(v_in.vel.data, 3) * cartesian_weights.topRows<3>().array();
    vin.bottomRows<3>() = Eigen::Map<const Eigen::Array3d>(v_in.rot.data, 3) * cartesian_weights.bottomRows<3>().array();

    // Do a singular value decomposition: J = U*S*V^t
    svd_.compute(jac.topRows(rows));

    if (num_mimic_joints_ > 0)
    {
        qdot_out_reduced_.noalias() = svd_.solve(vin.topRows(rows));
        qdot_out_reduced_.array() *= joint_weights.array();
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
        qdot_out(i) = qdot_out_reduced_[mimic_joints_[i].map_index] * mimic_joints_[i].multiplier;
    }
    else
    {
        qdot_out.data.noalias() = svd_.solve(vin.topRows(rows));
        qdot_out.data.array() *= joint_weights.array();
    }

    return 0;
}

int FloatingChainIkSolverVel::CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out)
{

}

void FloatingChainIkSolverVel::updateInternalDataStructures()
{

}

}
}//end of namespace space_robot

#endif
