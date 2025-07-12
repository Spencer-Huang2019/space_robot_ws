#ifndef SRBOT_CHAIN_FKSOLVER_H
#define SRBOT_CHAIN_FKSOLVER_H

#include <kdl/chain.hpp>
#include <kdl/framevel.hpp>
#include <kdl/frameacc.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/solveri.hpp>


using namespace KDL;

namespace space_robot {
namespace kinematics_sovler {

class FloatingFloatingChainFkSolver : public SolverI
{
public:
    FloatingChainFkSolver(const Chain& chain) : chain_(chain){}
    ~FloatingChainFkSolver(){}

    int JntToCart(const JntArray& q_in, Frame& p_out, int segmentNr=-1) override;
    int JntToCart(const JntArray& q_in, std::vector<Frame>& p_out, int segmentNr=-1) override;

    void updateInternalDataStructures() override {}

private:
    const Chain& chain_;
};

}
}//end of namespace space_robot

#endif
