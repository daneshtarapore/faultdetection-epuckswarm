#ifndef PHOTOTAXISBEHAVIOR_H_
#define PHOTOTAXISBEHAVIOR_H

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CPhototaxisBehavior : public CBehavior
{
public:
    CPhototaxisBehavior();

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    void SimulationStep()
    {

    }

    virtual void PrintBehaviorIdentity();


protected:

    CVector2         m_cLightVector;


};


/******************************************************************************/
/******************************************************************************/

#endif 
