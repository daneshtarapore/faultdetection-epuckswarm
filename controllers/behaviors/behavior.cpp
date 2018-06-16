#include "behavior.h"

/******************************************************************************/
/******************************************************************************/

CBehavior::CBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

CBehavior::~CBehavior()
{
}

/******************************************************************************/
/******************************************************************************/

void CBehavior::Suppress()
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::PrintBehaviorIdentity()
{}

/******************************************************************************/
/******************************************************************************/

void CBehavior::WheelSpeedsFromHeadingVector(CVector2 &m_cHeadingVector, Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    /* Get the heading angle */
    CRadians cHeadingAngle = m_cHeadingVector.Angle().SignedNormalize();
    /* Get the length of the heading vector */
    Real fHeadingLength = m_cHeadingVector.Length();
    /* Clamp the speed so that it's not greater than MaxSpeed */
    Real fBaseAngularWheelSpeed = Min<Real>(fHeadingLength, m_sRobotData.MaxSpeed);

    int TurningMechanism;

    /* Turning state switching conditions */
    if(cHeadingAngle.GetAbsoluteValue() <= m_sRobotData.m_cNoTurnOnAngleThreshold.GetValue())
    {
        /* No Turn, heading angle very small */
        TurningMechanism = NO_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " NO_TURN " << std::endl;
    }
    else if(cHeadingAngle.GetAbsoluteValue() > m_sRobotData.m_cNoTurnOnAngleThreshold.GetValue() &&
            cHeadingAngle.GetAbsoluteValue() <= m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue())
    {
        /* Soft Turn, heading angle in between the two cases */
        TurningMechanism = SOFT_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " SOFT_TURN " << std::endl;
    }
    else if(cHeadingAngle.GetAbsoluteValue() > m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue()) // m_sWheelTurningParams.SoftTurnOnAngleThreshold
    {
        /* Hard Turn, heading angle very large */
        TurningMechanism = HARD_TURN;
        //std::cout << " cHeadingAngle " << cHeadingAngle << " fBaseAngularWheelSpeed " << fBaseAngularWheelSpeed << " HARD_TURN " << std::endl;
    }


    /* Wheel speeds based on current turning state */
    Real fSpeed1, fSpeed2;
    switch(TurningMechanism)
    {
    case NO_TURN:
    {
        /* Just go straight */
        fSpeed1 = fBaseAngularWheelSpeed;
        fSpeed2 = fBaseAngularWheelSpeed;
        break;
    }

    case SOFT_TURN:
    {
        /* Both wheels go straight, but one is faster than the other */ //HardTurnOnAngleThreshold
        Real fSpeedFactor = (m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue() - cHeadingAngle.GetAbsoluteValue()) /
                             m_sRobotData.m_cSoftTurnOnAngleThreshold.GetValue();
        fSpeed1 = fBaseAngularWheelSpeed - fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);
        fSpeed2 = fBaseAngularWheelSpeed + fBaseAngularWheelSpeed * (1.0 - fSpeedFactor);


        /*
         * to make sure the speeds do not exceed the max speed
         *
         */
        if (fSpeed1 > m_sRobotData.MaxSpeed)
            fSpeed1 = m_sRobotData.MaxSpeed;

        if (fSpeed2 > m_sRobotData.MaxSpeed)
            fSpeed2 = m_sRobotData.MaxSpeed;

        if (fSpeed1 < -m_sRobotData.MaxSpeed)
            fSpeed1 = -m_sRobotData.MaxSpeed;

        if (fSpeed2 < -m_sRobotData.MaxSpeed)
            fSpeed2 = -m_sRobotData.MaxSpeed;


        break;
    }

    case HARD_TURN:
    {
        /* Opposite wheel speeds */
        fSpeed1 = -m_sRobotData.MaxSpeed;
        fSpeed2 =  m_sRobotData.MaxSpeed;
        break;
    }
    }

    /* Apply the calculated speeds to the appropriate wheels */
    if(cHeadingAngle > CRadians::ZERO)
    {
        /* Turn Left */
        fLeftWheelSpeed  = fSpeed1;
        fRightWheelSpeed = fSpeed2;
    }
    else {
        /* Turn Right */
        fLeftWheelSpeed  = fSpeed2;
        fRightWheelSpeed = fSpeed1;
    }




//    Other approach to get wheel speeds from heading
//    if (m_cGoStraightAngleRangeDegrees.WithinMinBoundIncludedMaxBoundIncluded(ToDegrees(m_cHeadingVector.Angle().SignedNormalize())))
//    {
//        std::cout << " P Straight m_cHeadingVector.Angle().SignedNormalize() " << (ToDegrees(m_cHeadingVector.Angle().SignedNormalize())) << std::endl;
//        fLeftWheelSpeed  = m_sRobotData.MaxSpeed;
//        fRightWheelSpeed = m_sRobotData.MaxSpeed;
//        return;
//    }


//    Real fSpeed;
//    CRadians angle = m_cHeadingVector.Angle();

//    //     std::cout << " Light homing behavior: m_cHeadingVector.Angle() " << m_cHeadingVector.Angle() << std::endl;
//    angle = angle.SignedNormalize();
//    //     std::cout << " Light homing behavior: angle.SignedNormalize() " << angle.SignedNormalize() << std::endl;

//    //     std::cout << " Light homing behavior: angle.GetAbsoluteValue() " << angle.GetAbsoluteValue() << std::endl;


//    fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE / m_sRobotData.seconds_per_tick;
//    fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
//    fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);

//    if(angle.GetValue() > 0.0f) // heading right
//    {
//        std::cout << "light source in the right: turn right " << angle.GetValue() << std::endl;
//        fLeftWheelSpeed  =  -fSpeed;
//        fRightWheelSpeed = fSpeed;
//        std::cout << "P: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
//    }
//    else // heading left
//    {
//        std::cout << "light source in the left: turn left " << angle.GetValue() << std::endl;
//        fLeftWheelSpeed  = fSpeed;
//        fRightWheelSpeed =  -fSpeed;
//        std::cout << "P: fLeftWheelSpeed " << fLeftWheelSpeed <<  " fRightWheelSpeed " << fRightWheelSpeed  << std::endl;
//    }
}

/******************************************************************************/
/******************************************************************************/
