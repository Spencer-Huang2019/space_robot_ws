#include ".chain_fk_solver.h"


namespace space_robot {
namespace kinematics_sovler {

int FloatingChainFkSolver::joint_to_cart(const JntArray& q_in, Frame& p_out, int segmentNr=-1)
{
    unsigned int segmentNr;
    if(seg_nr<0)
        segmentNr=chain.getNrOfSegments();
    else
        segmentNr = seg_nr;

    p_out = Frame::Identity();

    if(q_in.rows()!=chain.getNrOfJoints())
        return (error = E_SIZE_MISMATCH);
    else if(segmentNr>chain.getNrOfSegments())
        return (error = E_OUT_OF_RANGE);
    else
    {
        int j=0;
        for(unsigned int i=0;i<segmentNr;i++)
        {
            if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed) 
            {
                p_out = p_out*chain.getSegment(i).pose(q_in(j));
                j++;
            }
            else
            {
                p_out = p_out*chain.getSegment(i).pose(0.0);
            }
        }

        return (error = E_NOERROR);
    }
}

int FloatingChainFkSolver::joint_to_cart(const JntArray& q_in, std::vector<Frame>& p_out, int segmentNr=-1)
{
    unsigned int segmentNr;
    if(seg_nr<0)
        segmentNr=chain.getNrOfSegments();
    else
        segmentNr = seg_nr;

    if(q_in.rows()!=chain.getNrOfJoints())
        return -1;
    else if(segmentNr>chain.getNrOfSegments())
        return -1;
    else if(p_out.size() != segmentNr)
        return -1;
    else if(segmentNr == 0)
        return -1;
    else
    {
        int j=0;
        // Initialization
        if(chain.getSegment(0).getJoint().getType()!=Joint::Fixed) 
        {
            p_out[0] = chain.getSegment(0).pose(q_in(j));
            j++;
        }
        else
            p_out[0] = chain.getSegment(0).pose(0.0);

        for(unsigned int i=1;i<segmentNr;i++)
        {
            if(chain.getSegment(i).getJoint().getType()!=Joint::Fixed) 
            {
                p_out[i] = p_out[i-1]*chain.getSegment(i).pose(q_in(j));
                j++;
            }
            else
            {
                p_out[i] = p_out[i-1]*chain.getSegment(i).pose(0.0);
            }
        }
        return 0;
    }
}

}
}//end of namespace space_robot

#endif
