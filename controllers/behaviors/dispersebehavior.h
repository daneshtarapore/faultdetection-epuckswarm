#ifndef DISPERSEBEHAVIOR_H_
#define DISPERSEBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/


using namespace argos;

class CProximitySensorEquippedEntity;

class CDisperseBehavior : public CBehavior 
{
public:
    CDisperseBehavior(Real m_fProximitySensorThreshold, CRadians m_cGoStraightAngleThreshold);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    void SimulationStep()
    {

    }

    virtual void PrintBehaviorIdentity();


protected:
    Real           m_fProximitySensorThreshold;
    CRadians       m_cGoStraightAngleThreshold;
    CVector2       m_cDiffusionVector;
};


/******************************************************************************/
/******************************************************************************/

#endif 
