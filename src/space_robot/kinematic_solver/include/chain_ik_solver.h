#ifndef SROBOT_CHAIN_IKSOLVER_H
#define SROBOT_CHAIN_IKSOLVER_H

#include <kdl/config.h>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <moveit/kdl_kinematics_plugin/joint_mimic.hpp>
#include <Eigen/SVD>


using namespace KDL;

namespace space_robot {
namespace kinematics_sovler {

class FloatingFloatingChainIkSolverVel : public SolverI
{
public:
    FloatingChainIkSolverVel(const Chain& chain): chain_(chain){}
    ~FloatingChainIkSolverVel(){}

    int CartToJnt(const JntArray& q_in, const Twist& v_in, JntArray& qdot_out) override;
    int CartToJnt(const JntArray& q_init, const FrameVel& v_in, JntArrayVel& q_out) override { return -1; };
    
    void updateInternalDataStructures()  override;

private:
    const Chain& chain_;
    ChainJntToJacSolver jnt2jac_;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_;

    Jacobian jac_;          // full Jacobian
};

}
}//end of namespace space_robot

#endif
