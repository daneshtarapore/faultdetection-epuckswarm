#include "phototaxisbehavior.h"


/******************************************************************************/
/******************************************************************************/

CPhototaxisBehavior::CPhototaxisBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

bool CPhototaxisBehavior::TakeControl()
{
    return false;
}

/******************************************************************************/
/******************************************************************************/

void CPhototaxisBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{

}

/******************************************************************************/
/******************************************************************************/

void CPhototaxisBehavior::PrintBehaviorIdentity()
{
    std::cout << "Phototaxis - behaviour to be implemented";
}

/******************************************************************************/
/******************************************************************************/
