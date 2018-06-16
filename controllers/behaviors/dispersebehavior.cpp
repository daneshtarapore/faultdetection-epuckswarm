#include "dispersebehavior.h"

#include <algorithm>

/******************************************************************************/
/******************************************************************************/

CDisperseBehavior::CDisperseBehavior(Real m_fProximitySensorThreshold, CRadians m_cGoStraightAngleThreshold) :
    m_fProximitySensorThreshold(m_fProximitySensorThreshold),
    m_cGoStraightAngleThreshold(m_cGoStraightAngleThreshold)
{
}

/******************************************************************************/
/******************************************************************************/

bool CDisperseBehavior::TakeControl() 
{
    /* Get readings from proximity sensor */
    /* Sum them together */
    //    m_cDiffusionVector.Set(0.0f, 0.0f);
    //    for(size_t i = 0; i <  m_sSensoryData.m_ProximitySensorData.size(); ++i)
    //    {
    //        m_cDiffusionVector += CVector2(m_sSensoryData.m_ProximitySensorData[i], m_sSensoryData.m_ProximitySensorData[i].Angle);
    //    }
    //    m_cDiffusionVector /= m_sSensoryData.m_ProximitySensorData.size();



    /* If the angle of the vector is small enough and the closest obstacle
      is far enough, ignore the vector and go straight, otherwise return
      it */
    //    if(m_cDiffusionVector.Angle().GetAbsoluteValue() < m_cGoStraightAngleThreshold.GetValue() && m_cDiffusionVector.Length() < m_fProximitySensorThreshold)
    //    {
    //        return false;
    //    }
    //    else
    //    {
    //        if(m_cDiffusionVector.Length() < 0.05) /* because of noise, we can have very small non-zero sensor readings. but we don't want to responmd to them*/
    //            return false;

    //        //std::cout << " m_cDiffusionVector length " << m_cDiffusionVector.Length() << " and threshold " << m_fProximitySensorThreshold << std::endl;
    //        //std::cout << " m_cDiffusionVector angle " <<  m_cDiffusionVector.Angle().GetAbsoluteValue() << " and threshold " << m_cGoStraightAngleThreshold.GetValue() << std::endl;

    //        return true;
    //    }

    std::vector<Real>::iterator max_sensor_reading_it = std::max_element(m_sSensoryData.m_ProximitySensorData.begin(), m_sSensoryData.m_ProximitySensorData.end());
    Real max_sensor_reading = *max_sensor_reading_it;

    if(max_sensor_reading < m_fProximitySensorThreshold)
        return false;
    else
        return true;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CDisperseBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    //    CVector2 m_cHeadingVector = -m_cDiffusionVector.Normalize() * m_sRobotData.MaxSpeed;
    //    WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);

    /* Get the highest reading in front of the robot, which corresponds to the closest object */
    Real fMaxReadVal = m_sSensoryData.m_ProximitySensorData[0];
    UInt32 unMaxReadIdx = 0;
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[1])
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[1];
        unMaxReadIdx = 1;
    }
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[7])
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[7];
        unMaxReadIdx = 7;
    }
    if(fMaxReadVal < m_sSensoryData.m_ProximitySensorData[6])
    {
        fMaxReadVal = m_sSensoryData.m_ProximitySensorData[6];
        unMaxReadIdx = 6;
    }
    /* Do we have an obstacle in front? */
    if(fMaxReadVal > 0.0f) {
        /* Yes, we do: avoid it */
        if(unMaxReadIdx == 0 || unMaxReadIdx == 1)
        {
            /* The obstacle is on the left, turn right */
            fLeftWheelSpeed = m_sRobotData.MaxSpeed; fRightWheelSpeed = 0.0f;
        }
        else
        {
            /* The obstacle is on the left, turn right */
            fLeftWheelSpeed = 0.0f; fRightWheelSpeed = m_sRobotData.MaxSpeed;
        }
    }
    else
    {
        /* No, we don't: go straight */
        fLeftWheelSpeed = m_sRobotData.MaxSpeed; fRightWheelSpeed = m_sRobotData.MaxSpeed;
    }
}

/******************************************************************************/
/******************************************************************************/

void CDisperseBehavior::PrintBehaviorIdentity()
{
    std::cout << "Disperse taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/

