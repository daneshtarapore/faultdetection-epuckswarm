#include "homingtofoodbeaconbehavior.h"


/******************************************************************************/
/******************************************************************************/

CHomingToFoodBeaconBehavior::CHomingToFoodBeaconBehavior(UInt8 BeaconData, Real MAX_BEACON_SIGNAL_RANGE)
{
    m_iBeaconData        = BeaconData;
    m_fBeaconSignalRange = MAX_BEACON_SIGNAL_RANGE * 100.0f; // converting max range to cm
}

/******************************************************************************/
/******************************************************************************/

bool CHomingToFoodBeaconBehavior::TakeControl()
{
    /* Get readings from RAB sensor */
    /* Set heading to beacon with smallest range */

    bool controltaken(false);

    Real closestBeaconRange = 1000000.0f;  CRadians closestBeaconBearing; /*Range of 1000000.0cm will never be exceeded */
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        //BEACON_ESTABLISHED = m_iBeaconData
        //std::cout << "Received - byte  0 " << m_sSensoryData.m_RABSensorData[i].Data[0] << std::endl;

        if(m_sSensoryData.m_RABSensorData[i].Data[0] == m_iBeaconData && m_sSensoryData.m_RABSensorData[i].Range < m_fBeaconSignalRange)
        {
            controltaken = true;

            if(m_sSensoryData.m_RABSensorData[i].Range < closestBeaconRange)
            {
                closestBeaconRange   = m_sSensoryData.m_RABSensorData[i].Range;
                closestBeaconBearing = m_sSensoryData.m_RABSensorData[i].HorizontalBearing;
            }
        }
    }

    if(controltaken)
    {
        m_cHomingVector = CVector2(closestBeaconRange, closestBeaconBearing); // range is in cm, but since we are going to normalise the vector it does not matter
        // std::cout << " HomingToFoodBeacon Behavior  - closestBeaconBearing" << ToDegrees(closestBeaconBearing).UnsignedNormalize().GetValue() << std::endl;
    }

    return controltaken;
}

/******************************************************************************/
/******************************************************************************/

// Move in the opposite direction of CoM
void CHomingToFoodBeaconBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
     //CVector2 m_cHeadingVector = m_sRobotData.MaxSpeed * m_cHomingVector.Normalize(); // static aggregate formed around the beacon. may be needed for the missing RAB receiver fault
     CVector2 m_cHeadingVector = m_sRobotData.MaxSpeed * m_cHomingVector.Normalize() + m_sRobotData.MaxSpeed * CVector2(1.0f, 0.0f);

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);
}

/******************************************************************************/
/******************************************************************************/

void CHomingToFoodBeaconBehavior::PrintBehaviorIdentity()
{
    std::cout << "Homing behavior taking over" << std::endl;
}

/******************************************************************************/
/******************************************************************************/
