#ifndef ANTIPHOTOTAXISBEHAVIOR_H_
#define ANTIPHOTOTAXISBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CAntiPhototaxisBehavior : public CBehavior
{
public:
    CAntiPhototaxisBehavior();

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
