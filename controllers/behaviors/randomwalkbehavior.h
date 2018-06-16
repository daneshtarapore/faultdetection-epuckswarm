#ifndef RANDOMWALKBEHAVIOR_H_
#define RANDOMWALKBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CRandomWalkBehavior : public CBehavior 
{
public:
    CRandomWalkBehavior(double f_change_direction_probability);
    
    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    void SimulationStep()
    {

    }

    virtual void PrintBehaviorIdentity();


protected:
    double m_fChangeDirectionProbability;
};

/******************************************************************************/
/******************************************************************************/

#endif 
