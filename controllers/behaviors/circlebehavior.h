#ifndef CIRCLEBEHAVIOR_H_
#define CIRCLEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

class CCircleBehavior : public CBehavior
{
public:
    CCircleBehavior();
    
    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    void SimulationStep()
    {

    }

    virtual void PrintBehaviorIdentity();


protected:

};

/******************************************************************************/
/******************************************************************************/

#endif 
