#include "randomwalkbehavior.h"
//#include "random.h"


/******************************************************************************/
/******************************************************************************/

CRandomWalkBehavior::CRandomWalkBehavior(double f_change_direction_probability) :
    m_fChangeDirectionProbability(f_change_direction_probability) 
{    
}

/******************************************************************************/
/******************************************************************************/
    
bool CRandomWalkBehavior::TakeControl() 
{
    return true;
}

/******************************************************************************/
/******************************************************************************/

void CRandomWalkBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    if (m_fChangeDirectionProbability >= m_sSensoryData.m_pcRNG->Uniform(CRange<Real>(0.0, 1.0)))
    {
        Real fSpeed;

        CRadians angle = m_sSensoryData.m_pcRNG->Uniform(CRange<CRadians>(-CRadians::PI, CRadians::PI));


        //fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE / m_sRobotData.seconds_per_iterations;
        fSpeed = angle.GetAbsoluteValue() * m_sRobotData.HALF_INTERWHEEL_DISTANCE;
        fSpeed = fSpeed * 100.0; // converting to cm/s - as used in SetLinearVelocity
        fSpeed = Min<Real>(fSpeed, m_sRobotData.MaxSpeed);

        //std::cout << "fSpeed  " << fSpeed << std::endl;


        if(angle.GetValue() > 0.0f) //turn right
        {
            //std::cout << " turn right " << angle.GetValue() << std::endl;
            fLeftWheelSpeed  = fSpeed;
            fRightWheelSpeed = -fSpeed;
        }
        else
        {
            //std::cout << " turn left " << angle.GetValue() << std::endl;
            fLeftWheelSpeed  = -fSpeed;
            fRightWheelSpeed =  fSpeed;
        }

    }
    else
    {
        //std::cout << " move straight " << std::endl;
        fLeftWheelSpeed  = m_sRobotData.MaxSpeed;
        fRightWheelSpeed = m_sRobotData.MaxSpeed;
    }

    //std::cout << "RavdLS:  " << fLeftWheelSpeed << " RS:  " << fRightWheelSpeed << std::endl;


}

/******************************************************************************/
/******************************************************************************/

void CRandomWalkBehavior::PrintBehaviorIdentity()
{
    std::cout << "Randomwalk behavior taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/
