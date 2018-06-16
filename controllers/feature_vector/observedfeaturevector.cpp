#include "observedfeaturevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

#define MODELSTARTTIME 450.0 // used for your sliding window - computing features the old method with ComputeFeatureValues_Old()

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned CObservedFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
unsigned CObservedFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double   CObservedFeatureVector::FEATURE_RANGE             = 15.0; //cm // was 60 cm for the old features

/******************************************************************************/
/******************************************************************************/

Real CObservedFeatureVector::m_fVelocityThreshold            = 0.0f;
Real CObservedFeatureVector::m_fAccelerationThreshold        = 0.0f;

/*Real CObservedFeatureVector::m_tAngularVelocityThreshold     = 0.0f;
Real CObservedFeatureVector::m_tAngularAccelerationThreshold = 0.0f;*/


int CObservedFeatureVector::m_iEventSelectionTimeWindow      = 0u;
int CObservedFeatureVector::m_iDistTravelledTimeWindow       = 0u;

// the time window is in ticks and spped is in cm / tick
Real CObservedFeatureVector::m_fCumulativeDistThreshold = 0.0f;


int CObservedFeatureVector::m_iShortTimeWindowLength  = 0u;
int CObservedFeatureVector::m_iMediumTimeWindowLength = 0u;
int CObservedFeatureVector::m_iLongTimeWindowLength   = 0u;


/******************************************************************************/
/******************************************************************************/
#define ROBOTID_NOT_IN_SIGNAL          999
#define ROBOTSELFBEARING_NOT_IN_SIGNAL 999.0f
#define ROBOTSELFACCELERATION_NOT_IN_SIGNAL 999.0f
/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::CObservedFeatureVector()
{
    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    // too small thresholded distance. robot moves very little in one control-cycle. better to intergrate distance over time and use threshold on that
    m_fVelocityThreshold            = 0.5f  * (m_sRobotData.MaxLinearSpeed); // max speed in  cm per control cycle
    m_fAccelerationThreshold        = 0.2f  * (m_sRobotData.MaxLinearAcceleration); // max change in speed is \[PlusMinus]MaxLinearSpeed per control cycle per control cycle

    m_iEventSelectionTimeWindow      = MODELSTARTTIME; // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_iDistTravelledTimeWindow       = 100;            // keeping track of distance travelled by bot in last 100 time-steps

    // the time window is in ticks and spped is in cm / tick
    m_fCumulativeDistThreshold = (0.05f * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow));

    m_pcListObservedRobots.clear();


    /* Length of time windows for observing neighbours of your neighbours and the distance they travel */
    m_iShortTimeWindowLength  = (unsigned)m_sRobotData.iterations_per_second;
    m_iMediumTimeWindowLength = (unsigned)m_sRobotData.iterations_per_second * 5u;
    m_iLongTimeWindowLength   = (unsigned)m_sRobotData.iterations_per_second * 10u;
}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::~CObservedFeatureVector()
{}

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::SimulationStep()
{
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        bool     b_ObservedRobotFound(false);
        unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i);
        for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
        {
            if(u_ObservedRobotId == it_listobrob->m_unRobotId)
            {
                b_ObservedRobotFound = true;
                break; // each robot id is represented only once in the m_pcListObservedRobots list
            }
        }

        if (!b_ObservedRobotFound && u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL)
            m_pcListObservedRobots.push_back(ObservedRobots_FeatureVector((*this), m_sSensoryData.m_rTime, u_ObservedRobotId));
    }


    /*
    BUT WHEN DO WE DELETE ObservedRobots_FeatureVector FROM THE LIST?
    FOR NOW WE DELETE IT IMMEDIATELY UPON NOT BEING OBSERVED
    */
    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    while (it_listobrob != m_pcListObservedRobots.end())
    {
        unsigned RobotId = it_listobrob->m_unRobotId;
        bool b_ObservedRobotFound(false);
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            if (RobotId == GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i))
            {
                b_ObservedRobotFound = true;
                break;
            }
        }

        if (!b_ObservedRobotFound)
        {
            it_listobrob = m_pcListObservedRobots.erase(it_listobrob);
            continue;
        }

        ++it_listobrob;
    }



    ObservedRobotIDs.clear(); ObservedRobotFVs.clear();
    for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
    {
        it_listobrob->ComputeFeatureValues();

        /*
         *  ONLY REPORT ON ID AND FV OF OBSERVED ROBOT IF OBSERVATION TIME EXCEEDS 45s?
         */

        /* if((m_sSensoryData.m_rTime - it_listobrob->m_fTimeFirstObserved) >= m_iEventSelectionTimeWindow)  - used for the ComputeFeatureValues_Old */
        if((m_sSensoryData.m_rTime - it_listobrob->m_fTimeFirstObserved) >= m_iEventSelectionTimeWindow)
        {
            ObservedRobotIDs.push_back(it_listobrob->m_unRobotId);
            ObservedRobotFVs.push_back(it_listobrob->GetValue());
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned CObservedFeatureVector::GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index)
{
    if ((rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER) ||
        (rab_packet[rab_packet_index].Data[0] == m_sRobotData.NEST_BEACON_SIGNAL_MARKER))
    {
        if (rab_packet[rab_packet_index].Data[1] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[1] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            return rab_packet[rab_packet_index].Data[2];
        }
        else
        {
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
    else
    {
        if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[0] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            return rab_packet[rab_packet_index].Data[1];
        }
        else
        {
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
}

/******************************************************************************/
/******************************************************************************/

float CObservedFeatureVector::GetSelfBearingFromRABPacket(unsigned observer_robot_id, CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index, unsigned observerd_robot_id)
{
    size_t index_start = 0;
    if ((rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER) ||
        (rab_packet[rab_packet_index].Data[0] == m_sRobotData.NEST_BEACON_SIGNAL_MARKER))
        index_start = 1;


    if (rab_packet[rab_packet_index].Data[index_start] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[index_start] == m_sRobotData.SELF_INFO_PACKET_MARKER)
    {
        unsigned chk_observerd_robot_id = rab_packet[rab_packet_index].Data[index_start+1]; // the id comes after the message header.
        assert(chk_observerd_robot_id == observerd_robot_id);
    }

    // search for the appropriate message header. once found read until the end of this message header until you find observer_robot_id. once you find

    bool message_header_found(false);
    for(size_t data_index = index_start; data_index < rab_packet[rab_packet_index].Data.Size(); ++data_index)
    {
        if(rab_packet[rab_packet_index].Data[data_index] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            index_start = data_index;
            message_header_found = true;
            break;
        }
    }

    if(message_header_found == false)
        return ROBOTSELFBEARING_NOT_IN_SIGNAL;

    for(size_t data_index = index_start; data_index < rab_packet[rab_packet_index].Data.Size(); data_index+=2)
    {
        if((rab_packet[rab_packet_index].Data[data_index] == m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER) ||
           (rab_packet[rab_packet_index].Data[data_index+1] == m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER))
            break;

        if(rab_packet[rab_packet_index].Data[data_index] == observer_robot_id)
        {
            /*if(observerd_robot_id == 15 && observer_robot_id == 17)
                std::cout << "R " << rab_packet[rab_packet_index].Data[data_index+1] << " " << rab_packet[rab_packet_index].Data[data_index+1] * 360.0f / m_sRobotData.DATA_BYTE_BOUND_MARKER <<std::endl;*/
            return rab_packet[rab_packet_index].Data[data_index+1] * 360.0f / m_sRobotData.DATA_BYTE_BOUND_MARKER;
        }
    }

    return ROBOTSELFBEARING_NOT_IN_SIGNAL;
}

/******************************************************************************/
/******************************************************************************/

float CObservedFeatureVector::GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index, unsigned observerd_robot_id)
{
    size_t index_start = 0;
    if ((rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER) ||
        (rab_packet[rab_packet_index].Data[0] == m_sRobotData.NEST_BEACON_SIGNAL_MARKER))
        index_start = 1;


    if (rab_packet[rab_packet_index].Data[index_start] == m_sRobotData.SELF_INFO_PACKET_MARKER)
    {
        unsigned chk_observerd_robot_id = rab_packet[rab_packet_index].Data[index_start+1]; // the id comes after the message header.
        assert(chk_observerd_robot_id == observerd_robot_id);
        return (((Real)(rab_packet[rab_packet_index].Data[index_start+2]) / m_sRobotData.DATA_BYTE_BOUND_MARKER) * 2.0f) - 1.0f; // return [-1, +1]
    }

    return ROBOTSELFACCELERATION_NOT_IN_SIGNAL;
}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::ObservedRobots_FeatureVector::ObservedRobots_FeatureVector(CObservedFeatureVector &owner_class, Real TimeFirstObserved, unsigned ObservedRobotId):
    owner(owner_class), m_fTimeFirstObserved(TimeFirstObserved), m_unRobotId(ObservedRobotId), m_unValue(0)
{

    //EstimatedLinearSpeed = 0.0f; EstimatedAngularSpeed = 0.0f; EstimatedLinearAcceleration = 0.0f; EstimatedAngularAcceleration = 0.0f;
    //estimated_bearing.SetValue(0.0f); prev_bearing.SetValue(0.0f);
    estimated_pos  = CVector2(0.0, 0.0); prev_pos  = CVector2(0.0, 0.0); estimated_dist = Real(0.0f);
    average_speed  = 0.0f; prev_average_speed = 0.0f; average_acceleration = 0.0f, m_fMeanOfAverageSpeeds = 0.0f;
    prev_prev_average_speed = 0.0f; prev_prev_prev_average_speed = 0.0f;
    m_pfAverageSpeedAtTimeStep  = new Real[owner.m_iDistTravelledTimeWindow];

    average_angularspeed = (0.0f); prev_average_angularspeed = (0.0f);
    average_angularacceleration = (0.0f);

    estimated_heading = 0.0f;  prev_estimated_heading = 0.0f; prev_prev_estimated_heading = 0.0f;


    m_pfFeatureValues         = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceEvent    = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceNegEvent = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];

    m_pfAllFeatureValues     = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CObservedFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

    m_unNbrsCurrQueueIndex = 0;
    m_unSumTimeStepsNbrsRange0to30 = 0;
    m_unSumTimeStepsNbrsRange30to60 = 0;
    m_punNbrsRange0to30AtTimeStep  = new unsigned int[owner.m_iEventSelectionTimeWindow];
    m_punNbrsRange30to60AtTimeStep = new unsigned int[owner.m_iEventSelectionTimeWindow];


    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_unQueueIndex_ShortRangeTimeWindow = 0u; m_unQueueIndex_MediumRangeTimeWindow = 0u; m_unQueueIndex_LongRangeTimeWindow = 0u;
    m_unSumTimeStepsNbrs_ShortRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_MediumRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_LongRangeTimeWindow = 0u;
    m_punNbrs_ShortRangeTimeWindow   = new unsigned int[owner.m_iShortTimeWindowLength];
    m_punNbrs_MediumRangeTimeWindow  = new unsigned int[owner.m_iMediumTimeWindowLength];
    m_punNbrs_LongRangeTimeWindow    = new unsigned int[owner.m_iLongTimeWindowLength];
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/



    // keeping track of distance travelled by bot in last 100 time-steps
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;
    m_fCumulativeDistTravelled = 0.0f;

    m_pvecCoordAtTimeStep = new CVector2[owner.m_iDistTravelledTimeWindow];
    m_pfDistAtTimeStep    = new Real[owner.m_iDistTravelledTimeWindow];

    vec_RobotRelativePosition.resize((unsigned)owner.m_sRobotData.iterations_per_second);
}

/******************************************************************************/
/******************************************************************************/

/* Use the copy constructor since your structure has raw pointers and push_back the structure instance to list will just copy the pointer value */
CObservedFeatureVector::ObservedRobots_FeatureVector::ObservedRobots_FeatureVector(const ObservedRobots_FeatureVector &ClassToCopy):
    owner(ClassToCopy.owner), m_fTimeFirstObserved(ClassToCopy.m_fTimeFirstObserved), m_unRobotId(ClassToCopy.m_unRobotId), m_unValue(0)
{
    estimated_pos  = ClassToCopy.estimated_pos; prev_pos  = ClassToCopy.prev_pos; estimated_dist = ClassToCopy.estimated_dist;
    m_fMeanOfAverageSpeeds = ClassToCopy.m_fMeanOfAverageSpeeds;

    m_pfAverageSpeedAtTimeStep  = new Real[owner.m_iDistTravelledTimeWindow];

    average_speed  = ClassToCopy.average_speed; prev_average_speed = ClassToCopy.prev_average_speed;
    prev_prev_average_speed = ClassToCopy.prev_prev_average_speed; prev_prev_prev_average_speed = ClassToCopy.prev_prev_prev_average_speed;
    average_acceleration = ClassToCopy.average_acceleration;

    average_angularspeed = ClassToCopy.average_angularspeed; prev_average_angularspeed = ClassToCopy.prev_average_angularspeed;
    average_angularacceleration = ClassToCopy.average_angularacceleration;

    estimated_heading = 0.0f;  prev_estimated_heading = 0.0f; prev_prev_estimated_heading = 0.0f;


    m_pfFeatureValues         = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceEvent    = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];
    m_piLastOccuranceNegEvent = new int [CObservedFeatureVector::NUMBER_OF_FEATURES];

    m_pfAllFeatureValues     = new Real[CObservedFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CObservedFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }

    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to30 = 0;
    m_unSumTimeStepsNbrsRange30to60 = 0;

    m_punNbrsRange0to30AtTimeStep  = new unsigned int[owner.m_iEventSelectionTimeWindow];
    m_punNbrsRange30to60AtTimeStep = new unsigned int[owner.m_iEventSelectionTimeWindow];



    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_unQueueIndex_ShortRangeTimeWindow = 0u; m_unQueueIndex_MediumRangeTimeWindow = 0u; m_unQueueIndex_LongRangeTimeWindow = 0u;
    m_unSumTimeStepsNbrs_ShortRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_MediumRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_LongRangeTimeWindow = 0u;
    m_punNbrs_ShortRangeTimeWindow   = new unsigned int[owner.m_iShortTimeWindowLength];
    m_punNbrs_MediumRangeTimeWindow  = new unsigned int[owner.m_iMediumTimeWindowLength];
    m_punNbrs_LongRangeTimeWindow    = new unsigned int[owner.m_iLongTimeWindowLength];
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/



    // keeping track of distance travelled by bot in last 100 time-steps
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;
    m_fCumulativeDistTravelled = 0.0f;

    m_pvecCoordAtTimeStep = new CVector2[owner.m_iDistTravelledTimeWindow];
    m_pfDistAtTimeStep    = new Real[owner.m_iDistTravelledTimeWindow];

    vec_RobotRelativePosition.resize((unsigned)owner.m_sRobotData.iterations_per_second);
}

/******************************************************************************/
/******************************************************************************/

CObservedFeatureVector::ObservedRobots_FeatureVector::~ObservedRobots_FeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to30AtTimeStep;
    delete m_punNbrsRange30to60AtTimeStep;

    delete m_pvecCoordAtTimeStep;
    delete m_pfDistAtTimeStep;

    delete m_pfAverageSpeedAtTimeStep;


    delete m_punNbrs_ShortRangeTimeWindow;
    delete m_punNbrs_MediumRangeTimeWindow;
    delete m_punNbrs_LongRangeTimeWindow;
}

/******************************************************************************/
/******************************************************************************/

void CObservedFeatureVector::ObservedRobots_FeatureVector::ComputeFeatureValues_Old()
{
    Real DistToNearestNbr, CoM_nbrs;
    Real  unCloseRangeNbrCount = CountNeighbors(0.0, FEATURE_RANGE/2.0f, DistToNearestNbr, CoM_nbrs);
    Real  unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE/2.0f, FEATURE_RANGE, DistToNearestNbr, CoM_nbrs);

    assert(unCloseRangeNbrCount != -1.0f); assert(unFarRangeNbrCount != -1.0f);

    bool neighbours_present = ((unCloseRangeNbrCount + unFarRangeNbrCount) > 0.0f) ? true : false;

    /*
     * Time since the robot was first observed
     */
    Real CurrentStepNumber = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps

    // Perhaps we do not need to wait until the window is filled. We can set the features based on information gathered so far. In that case 0.5*m_iEventSelectionTimeWindow is replaced by 0.5*owner.m_sSensoryData.m_rTime; - m_fTimeFirstObserved;    Real m_fTimeWindow = std::min((Real)m_iEventSelectionTimeWindow, CurrentStepNumber);

    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps
        if(m_unSumTimeStepsNbrsRange0to30 > (unsigned)(0.5*(Real)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[0] = 1.0;
        else
            m_pfAllFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange30to60 > (unsigned)(0.5*(Real)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[1] = 1.0;
        else
            m_pfAllFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to30  -=  m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange30to60 -=  m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    if (unCloseRangeNbrCount > 0.0f)
    {
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to30++;
    }
    else
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    if (unFarRangeNbrCount > 0.0f)
    {
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange30to60++;
    }
    else
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;



    EstimateOdometry();


    //    // Motors
    //    //5th: distance travelled by bot in past 100 time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.

    m_fCumulativeDistTravelled += estimated_dist;
    m_fMeanOfAverageSpeeds     += average_speed;


    // distance travelled in last 100 time-steps
    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // removing the distance travelled in the first time-step of the moving time-window from the queue
        m_fCumulativeDistTravelled -= m_pfDistAtTimeStep[m_unCoordCurrQueueIndex];
        m_fMeanOfAverageSpeeds     -= m_pfAverageSpeedAtTimeStep[m_unCoordCurrQueueIndex];


        // decision based on distance travelled in the last 100 time-steps
        if(m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold)
            m_pfAllFeatureValues[2] = 1.0;
        else
            m_pfAllFeatureValues[2] = 0.0;

        if((m_fMeanOfAverageSpeeds/m_iDistTravelledTimeWindow) >  m_fVelocityThreshold)
            m_pfAllFeatureValues[3] = 1.0;
        else
            m_pfAllFeatureValues[3] = 0.0;
    }


    if (m_unRobotId == 1u)
    {
        /*if ((EstimatedLinearAcceleration >  m_fAccelerationThreshold ||
             EstimatedLinearAcceleration < -m_fAccelerationThreshold))*/
        {
            /*std::cout << " m_unRobotId " << m_unRobotId << std::endl;
            std::cout << " estimated_dist in last time-step " << estimated_dist << " cumulative distance " << m_fCumulativeDistTravelled << std::endl;
            std::cout << " LinearSpeed " <<  EstimatedLinearSpeed << " angular speed " << EstimatedAngularSpeed << std::endl;
            std::cout << " LinearAcceleration " <<  EstimatedLinearAcceleration << " angular acceleration " << EstimatedAngularAcceleration << std::endl;*/

            //std::cout << owner.m_sSensoryData.m_rTime << " " << estimated_dist << " " << m_fCumulativeDistTravelled << " " <<  EstimatedLinearSpeed << " " <<  EstimatedAngularSpeed << " " <<  EstimatedLinearAcceleration << " " << EstimatedAngularAcceleration << std::endl;

            std::cout << " Average speed " <<  average_speed << std::endl;
            std::cout << " Mean of average speeds " << m_fMeanOfAverageSpeeds / m_iDistTravelledTimeWindow << std::endl;
            std::cout << " Cumulative distance " << m_fCumulativeDistTravelled << std::endl;
            std::cout << " Average acceleration " << average_acceleration << std::endl;
        }
    }


    // adding distance travelled at last time-step into queue
    m_pfDistAtTimeStep[m_unCoordCurrQueueIndex]         = estimated_dist;
    m_pfAverageSpeedAtTimeStep[m_unCoordCurrQueueIndex] = average_speed;

    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;



    /*if( ((average_acceleration >  m_fAccelerationThreshold ||
         average_acceleration < -m_fAccelerationThreshold)) && (m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold)) // we add the last condition as in some instances acceleration may be wrongly detected for a stationary robot. happens when observer robot velocity is reduced from non-zero to 0 or vice-versa; even though robot is supposed to not move, it still moves a little (seen in renderer) and registers a change in the RAB sensor values. Usually the change is qquite small, but sometimes its large. the m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold condition can be relaxed to only look at distances covered by the robots in fewer than 100 past time-steps.
    {
         m_piLastOccuranceEvent[5] = CurrentStepNumber;
    }

    // 6th: set if the robot changed its linear speed (per control cycle) atleast once in the past time-window.
    if ((CurrentStepNumber - m_piLastOccuranceEvent[5]) <= m_iEventSelectionTimeWindow)
    {
        m_pfAllFeatureValues[5] = 1.0;
    }
    else
    {
        m_pfAllFeatureValues[5] = 0.0;
    }*/



    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration
    /*if(neighbours_present &&
            (EstimatedAngularAcceleration > m_tAngularAccelerationThreshold ||
             EstimatedAngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(!neighbours_present &&
            (EstimatedAngularAcceleration > m_tAngularAccelerationThreshold ||
             EstimatedAngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }*/

    if(neighbours_present &&
            ((average_acceleration >  m_fAccelerationThreshold  ||
              average_acceleration  < -m_fAccelerationThreshold) &&
             (m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold))) // we add the last condition as in some instances acceleration may be wrongly detected for a stationary robot. happens when robot velocity is reduced from non-zero to 0; even though robot is supposed to not move, it still moves a little (seen in rendere) and registers a change in the RAB sensor values. Usually the change is qquite small, but sometimes its large.
    {
        m_piLastOccuranceEvent[4] = CurrentStepNumber;
    }

    if(!neighbours_present &&
            ((average_acceleration >  m_fAccelerationThreshold  ||
              average_acceleration  < -m_fAccelerationThreshold) &&
             (m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold))) // we add the last condition as in some instances acceleration may be wrongly detected for a stationary robot. happens when robot velocity is reduced from non-zero to 0; even though robot is supposed to not move, it still moves a little (seen in rendere) and registers a change in the RAB sensor values. Usually the change is qquite small, but sometimes its large.
    {
        m_piLastOccuranceEvent[5] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 4; featureindex <=5; featureindex++)
    {
        if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfAllFeatureValues[featureindex] = 0.0;
        }
    }



    // adding the selected features into the feature vector
    for(size_t i = 0; i <  NUMBER_OF_FEATURES; ++i)
        m_pfFeatureValues[i] = m_pfAllFeatureValues[i];


    m_unValue = 0;
    for (unsigned int i = 0; i < CObservedFeatureVector::NUMBER_OF_FEATURES; i++)
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

void CObservedFeatureVector::ObservedRobots_FeatureVector::ComputeFeatureValues()
{
    FEATURE_RANGE = 30.0f; // cm
    Real DistToNearestNbr, CoM_nbrs, CoM_closerangenbrs = 0.0f, CoM_farrangenbrs = 0.0f;

    Real  unCloseRangeNbrCount = CountNeighbors(0.0f, FEATURE_RANGE/2.0f, DistToNearestNbr, CoM_closerangenbrs);
    Real  unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE/2.0f, FEATURE_RANGE, DistToNearestNbr, CoM_farrangenbrs);

    assert(unCloseRangeNbrCount != -1.0f); assert(unFarRangeNbrCount != -1.0f);


    Real   uNbrCount           = CountNeighbors(0.0f, 30.0f, DistToNearestNbr, CoM_nbrs); // 30 or 10 cm???
    assert(uNbrCount != -1.0f);



    //bool neighbours_present = ((unCloseRangeNbrCount + unFarRangeNbrCount) > 0u) ? true : false;
    bool neighbours_present = (uNbrCount > 0.0f) ? true : false;

    /*
     * Time since the robot was first observed
     */
    Real CurrentStepNumber = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        if(m_unSumTimeStepsNbrsRange0to30 > (unsigned)(0.5*(Real)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[0] = 1.0;
        else
            m_pfAllFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange30to60 > (unsigned)(0.5*(Real)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[1] = 1.0;
        else
            m_pfAllFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to30  -=  m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange30to60 -=  m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    if (unCloseRangeNbrCount > 0.0f)
    {
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to30++;
    }
    else
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    if (unFarRangeNbrCount > 0.0f)
    {
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange30to60++;
    }
    else
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;

    EstimateOdometry();
    list_Dist_ShortRangeTimeWindow.push_back(m_fEstimated_Dist_ShortTimeWindow);
    if(list_Dist_ShortRangeTimeWindow.size() >= 100u)
        list_Dist_ShortRangeTimeWindow.pop_front();

    list_Dist_MediumRangeTimeWindow.push_back(m_fEstimated_Dist_MediumTimeWindow);
    if(list_Dist_MediumRangeTimeWindow.size() >= 100u)
        list_Dist_MediumRangeTimeWindow.pop_front();

    list_Dist_LongRangeTimeWindow.push_back(m_fEstimated_Dist_LongTimeWindow);
    if(list_Dist_LongRangeTimeWindow.size() >= 100u)
        list_Dist_LongRangeTimeWindow.pop_front();


    Real meandist_shorttimewindow, meandist_mediumtimewindow, meandist_longtimewindow;
    meandist_shorttimewindow  = std::accumulate(list_Dist_ShortRangeTimeWindow.begin(), list_Dist_ShortRangeTimeWindow.end(), 0.0) / list_Dist_ShortRangeTimeWindow.size();
    meandist_mediumtimewindow = std::accumulate(list_Dist_MediumRangeTimeWindow.begin(), list_Dist_MediumRangeTimeWindow.end(), 0.0) / list_Dist_MediumRangeTimeWindow.size();
    meandist_longtimewindow   = std::accumulate(list_Dist_LongRangeTimeWindow.begin(), list_Dist_LongRangeTimeWindow.end(), 0.0) / list_Dist_LongRangeTimeWindow.size();


    Real m_fThreshold;
    if (m_sRobotData.OBSERVATION_MODE_TYPE == 1) // pure observation mode
        m_fThreshold = 10.0f; // in degrees
    else if (m_sRobotData.OBSERVATION_MODE_TYPE == 2) // combined proprioceptive +  observation mode
        m_fThreshold = 0.15f;  // proportion

    if (m_unRobotId == 15u)
    {
            /*std::cout << " Observed robot id " << m_unRobotId << std::endl;
            std::cout << " Estimated heading " << estimated_heading << std::endl;
            std::cout << " Average angular speed " <<  average_angularspeed << std::endl;*/

            if((average_angularacceleration  >   m_fThreshold)  ||
               (average_angularacceleration  <  -m_fThreshold))
            {
               /* std::cout << owner.m_sSensoryData.m_unRobotId << std::endl;
                std::cout << average_angularacceleration << std::endl;
                std::cout << prev_prev_estimated_heading << "; " << prev_estimated_heading << "; " << estimated_heading << std::endl;
                std::cout << prev_average_angularspeed << "; " << average_angularspeed << std::endl;*/

            }
    }

    Real f_MotorOutput      = fabs(average_angularacceleration) * (meandist_mediumtimewindow / (m_iMediumTimeWindowLength * m_sRobotData.MaxLinearSpeed));

    /* we need to discretise the data. we assume that a motor interaction occurs if f_MotorOutput exceeds 10% of max motor output */
    bool un_MotorOutput      = (f_MotorOutput > 0.10f)? 1u : 0u;


    if(neighbours_present && un_MotorOutput)
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if((!neighbours_present) && un_MotorOutput)
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
        for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
        {
            if ((CurrentStepNumber - m_piLastOccuranceEvent[featureindex]) <= m_iEventSelectionTimeWindow)
            {
                m_pfAllFeatureValues[featureindex] = 1.0;
            }
            else
            {
                m_pfAllFeatureValues[featureindex] = 0.0;
            }
        }



    if(un_MotorOutput)
    {
        m_piLastOccuranceEvent[5] = CurrentStepNumber;
    }


    // 6th: set if the robot changed its angular speed (per control cycle) atleast once in the past time-window.
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
        if ((CurrentStepNumber - m_piLastOccuranceEvent[5]) <= m_iEventSelectionTimeWindow)
        {
            m_pfAllFeatureValues[5] = 1.0;
        }
        else
        {
            m_pfAllFeatureValues[5] = 0.0;
        }




    //Real disp_ShortWindow_Threshold  = 0.25f;
    /*Real disp_MediumWindow_Threshold = 0.25f;
    Real disp_LongWindow_Threshold   = 0.10f;*/
    // MaxLinearSpeed in cm / control cycle
    //Real f5 = (meandist_mediumtimewindow >  2.5f) ? 1.0f: 0.0f;
    Real f6 = (meandist_longtimewindow   >  (0.15f*(m_iLongTimeWindowLength * m_sRobotData.MaxLinearSpeed))) ? 1.0f: 0.0f;

    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
        if (m_unRobotId == 15u) //7
        {
//            //std::cout << " average_angularacceleration " << average_angularacceleration << " " << f6 << std::endl;
//            if ((f5 == 1.0f) && ((average_angularacceleration >  m_fThreshold ||
//                                  average_angularacceleration < -m_fThreshold)) )
//            {
//                //std::cerr  << " sw " << meandist_shorttimewindow <<  " mw " << meandist_mediumtimewindow << " lw " << meandist_longtimewindow << std::endl;
        }


    /* CountNeighbors(0.0, 30.0f, DistToNearestNbr, CoM_nbrs); */

    /*if(CurrentStepNumber > m_iLongTimeWindowLength)
    printf("\n%f\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f \n", owner.m_sSensoryData.m_rTime, m_unRobotId, owner.m_sSensoryData.m_unRobotId, m_fEstimated_Dist_ShortTimeWindow,
                                                                        m_fEstimated_Dist_MediumTimeWindow,
                                                                        m_fEstimated_Dist_LongTimeWindow, average_angularacceleration, DistToNearestNbr,
                                                                        CoM_closerangenbrs, CoM_farrangenbrs, unCloseRangeNbrCount, unFarRangeNbrCount);*/

    /*if(CurrentStepNumber >= m_iLongTimeWindowLength)
        if (m_unRobotId == 15)
            printf("%f %f %f \n\n", m_fEstimated_Dist_ShortTimeWindow,m_fEstimated_Dist_MediumTimeWindow,m_fEstimated_Dist_LongTimeWindow);*/




    if (m_sRobotData.OBSERVATION_MODE_TYPE == 1) // pure observation mode
    {
        m_pfFeatureValues[0] = m_pfAllFeatureValues[0];
        m_pfFeatureValues[1] = m_pfAllFeatureValues[1];
        m_pfFeatureValues[2] = m_pfAllFeatureValues[2];
        m_pfFeatureValues[3] = m_pfAllFeatureValues[3];
        m_pfFeatureValues[4] = f6;
        m_pfFeatureValues[5] = m_pfAllFeatureValues[5];
    }
    else if (m_sRobotData.OBSERVATION_MODE_TYPE == 2) // combined proprioceptive + observation mode
    {
        m_pfFeatureValues[0] = m_pfAllFeatureValues[0];
        m_pfFeatureValues[1] = m_pfAllFeatureValues[1];
        m_pfFeatureValues[2] = m_pfAllFeatureValues[2];
        m_pfFeatureValues[3] = m_pfAllFeatureValues[3];
        m_pfFeatureValues[4] = f6;
        m_pfFeatureValues[5] = m_pfAllFeatureValues[5];
    }

    assert(CObservedFeatureVector::NUMBER_OF_FEATURES == 6);

    m_unValue = 0;
    for (unsigned int i = 0; i < CObservedFeatureVector::NUMBER_OF_FEATURES; i++)
        m_unValue += (unsigned)m_pfFeatureValues[i] * (1 << i);

    /*if (m_unRobotId == 15)
        std::cerr << "Observer " << owner.m_sSensoryData.m_unRobotId << " observing " << m_unRobotId << "FV " << m_unValue << std::endl;
    else
        std::cout << "Observer " << owner.m_sSensoryData.m_unRobotId << " observing " << m_unRobotId << "FV " << m_unValue << std::endl;*/

}

/******************************************************************************/
/******************************************************************************/

Real CObservedFeatureVector::ObservedRobots_FeatureVector::CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real& dist_nearest_nbr, Real& CoM_nbrs)
{
    Real count_nbrs = 0.0f; // Use Real instead of unsigned so that we can return -1 if the robot is not observed at the current time-step.
    CoM_nbrs   = 0.0f;

    dist_nearest_nbr = 1000000u;



    /*
     * counting the number of neighbours to observedRobotId_1
     */
    unsigned observedRobotId_1 = m_unRobotId;
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing; Real observedRobotId_1_SelfBearingOrAngularAcceleration;

    bool b_DataAvailable = GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing, observedRobotId_1_SelfBearingOrAngularAcceleration);
    // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP
    if(!b_DataAvailable)
        return -1.0f;


    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        unsigned observedRobotId_2 = owner.GetIdFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i);

        if(observedRobotId_1 == observedRobotId_2 || observedRobotId_2 == ROBOTID_NOT_IN_SIGNAL)
            continue;

        Real observedRobotId_2_Range       = owner.m_sSensoryData.m_RABSensorData[i].Range;
        CRadians observedRobotId_2_Bearing = owner.m_sSensoryData.m_RABSensorData[i].HorizontalBearing;

        CRadians diffBearing = observedRobotId_1_Bearing - observedRobotId_2_Bearing;

        Real Dist_ObsRobId1_ObsRobId2 = sqrt(observedRobotId_1_Range * observedRobotId_1_Range +
                                             observedRobotId_2_Range * observedRobotId_2_Range -
                                             2.0f * observedRobotId_1_Range * observedRobotId_2_Range * Cos(diffBearing));

        if((Dist_ObsRobId1_ObsRobId2 > lb_sensor_range) && (Dist_ObsRobId1_ObsRobId2 <= hb_sensor_range))
        {
            count_nbrs+=1.0f;
            CoM_nbrs += Dist_ObsRobId1_ObsRobId2;
        }

        if(Dist_ObsRobId1_ObsRobId2 < dist_nearest_nbr)
            dist_nearest_nbr = Dist_ObsRobId1_ObsRobId2;
    }


    // dont forget to add youself as neighbour of observedRobotId_1
    if((observedRobotId_1_Range > lb_sensor_range) && (observedRobotId_1_Range <= hb_sensor_range))
    {
        count_nbrs+=1.0f;
        CoM_nbrs  += observedRobotId_1_Range;

    }

    if(observedRobotId_1_Range < dist_nearest_nbr)
        dist_nearest_nbr = observedRobotId_1_Range;

    if(dist_nearest_nbr == 1000000u)
        dist_nearest_nbr = hb_sensor_range;


    if (count_nbrs > 0.0f)
        CoM_nbrs = CoM_nbrs / count_nbrs;

    return count_nbrs;
}

/******************************************************************************/
/******************************************************************************/

Real CObservedFeatureVector::ObservedRobots_FeatureVector::TrackNeighborsInQueue(Real step, unsigned current_num_nbrs, unsigned num_nbrs_threshold,
                                                                                 unsigned queue_length, Real queue_length_threshold,
                                                                                 unsigned int& sum_nbrs, unsigned int& queue_index, unsigned int* queue_nbrs)
{
    Real feature_value = 0.0f;

    if(step >= (Real)queue_length)
    {
        // decision based on the last X time-steps
        if(sum_nbrs > (unsigned)(queue_length_threshold*((Real)queue_length)))
            feature_value = 1.0f;
        else
            feature_value = 0.0f;

        // removing the fist entry of the moving time window  from the sum
        sum_nbrs  -=  queue_nbrs[queue_index];
    }

    // adding new values into the queue
    if (current_num_nbrs >= num_nbrs_threshold)
    {
        queue_nbrs[queue_index] = 1;
        sum_nbrs++;
    }
    else
        queue_nbrs[queue_index] = 0;

    queue_index = (queue_index + 1) % queue_length;

    return feature_value;
}

/******************************************************************************/
/******************************************************************************/

template <typename T> Real sgn(T val)
{
    /* val >= 0: return 1 else return -1  */
    return (Real)((T(0) <= val) - (val < T(0)));
}

template <typename T> Real diff_angle(T estimated_heading, T prev_estimated_heading)
{
    Real arg = fmod(estimated_heading-prev_estimated_heading, 360.00f);
    if (arg < 0.0f )  arg  = arg + 360.00f;
    if (arg > 180.0f) arg  = arg - 360.00f;
    return (-arg);
}


/******************************************************************************/
/******************************************************************************/

void CObservedFeatureVector::ObservedRobots_FeatureVector::EstimateOdometry()
{
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing; Real observedRobotId_1_SelfBearingOrAngularAcceleration;
    assert(GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing, observedRobotId_1_SelfBearingOrAngularAcceleration)); // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP

    if (m_sRobotData.OBSERVATION_MODE_TYPE == 1)
    {
        prev_prev_estimated_heading = prev_estimated_heading;
        prev_estimated_heading = estimated_heading;
        estimated_heading = observedRobotId_1_SelfBearingOrAngularAcceleration;
        if ((estimated_heading != ROBOTSELFBEARING_NOT_IN_SIGNAL) && (prev_estimated_heading != ROBOTSELFBEARING_NOT_IN_SIGNAL) &&
            (prev_prev_estimated_heading != ROBOTSELFBEARING_NOT_IN_SIGNAL))
        {
            prev_average_angularspeed = diff_angle(prev_estimated_heading, prev_prev_estimated_heading);;
            average_angularspeed = diff_angle(estimated_heading, prev_estimated_heading);
            average_angularacceleration = average_angularspeed - prev_average_angularspeed;

            /*if (m_unRobotId == 0u)
                printf("%f\n", average_angularspeed);*/
        }
        else
        {
            average_angularspeed = 0.0f; average_angularacceleration = 0.0f;
        }
    }
    else if (m_sRobotData.OBSERVATION_MODE_TYPE == 2)
    {
        assert(observedRobotId_1_SelfBearingOrAngularAcceleration != ROBOTSELFACCELERATION_NOT_IN_SIGNAL);
        average_angularacceleration = observedRobotId_1_SelfBearingOrAngularAcceleration;
    }
    else
    {
        std::cerr << "Observation modes can be of one of two types";
        exit(-1);
    }

    /*
     * Computing angle rotated by robot in one tick
    */
    CRadians delta_orientation = CRadians(owner.m_sRobotData.seconds_per_iterations * ((-owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed)
                                                                                       / (owner.m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

    /*if (owner.m_sSensoryData.m_unRobotId == 0u)
        printf("%f\t",ToDegrees(delta_orientation).GetValue());*/


    Real step = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;

    m_fEstimated_Dist_ShortTimeWindow  = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation, vec_RobPos_ShortRangeTimeWindow);
    m_fEstimated_Dist_MediumTimeWindow = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation, vec_RobPos_MediumRangeTimeWindow);
    m_fEstimated_Dist_LongTimeWindow   = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation, vec_RobPos_LongRangeTimeWindow);
}

/******************************************************************************/
/******************************************************************************/

Real CObservedFeatureVector::ObservedRobots_FeatureVector::TrackRobotDisplacement(Real step, Real observedRobotId_1_Range, CRadians observedRobotId_1_Bearing, CRadians delta_orientation, std::vector<RobotRelativePosData>& displacement_vector)
{
    Real displacement = 0.0f;

    /*
     * Computing average velocity
     */
    if(step < (Real)displacement_vector.size())
    {
        //vec_RobotRelativePosition[(unsigned)(owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved)]

        for (size_t t = 0 ; t < (unsigned)(step); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }

        displacement_vector[(unsigned)(step)].Range_At_Start           = observedRobotId_1_Range;
        displacement_vector[(unsigned)(step)].Bearing_At_Start         = observedRobotId_1_Bearing;
        displacement_vector[(unsigned)(step)].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing),
                                                               observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));

        displacement_vector[(unsigned)(step)].TimeSinceStart           = 0.0f;
        displacement_vector[(unsigned)(step)].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[(unsigned)(step)].NetRotationSinceStart.SetValue(0.0f);
    }
    else
    {
        for (size_t t = 0 ; t < displacement_vector.size(); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = owner.m_sRobotData.seconds_per_iterations *
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }


        size_t t = ((unsigned)(step)%displacement_vector.size());

        CVector2 tmp_pos           = CVector2(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        CVector2 tmp_pos_rot       = tmp_pos.Rotate(displacement_vector[t].NetRotationSinceStart);
        CVector2 pos_rot_trans     = tmp_pos_rot + displacement_vector[t].NetTranslationSinceStart;

        CVector2 Pos_At_Start      = displacement_vector[t].Pos_At_Start;

        /*
         * Computing average displacement
         */
        displacement              = (pos_rot_trans - Pos_At_Start).Length();

        displacement_vector[t].TimeSinceStart = 0.0f;
        displacement_vector[t].Range_At_Start           = observedRobotId_1_Range;
        displacement_vector[t].Bearing_At_Start         = observedRobotId_1_Bearing;
        displacement_vector[t].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        displacement_vector[t].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[t].NetRotationSinceStart.SetValue(0.0f);
    }

    return displacement;
}

/******************************************************************************/
/******************************************************************************/

bool CObservedFeatureVector::ObservedRobots_FeatureVector::GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing,
                                                                                        Real&  observedRobotId_1_SelfBearingORAngularAcceleration)
{
    bool observedRobotFound(false);
    for(size_t i = 0; i <  owner.m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_unRobotId == owner.GetIdFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i))
        {
            observedRobotId_1_Range   = owner.m_sSensoryData.m_RABSensorData[i].Range;
            observedRobotId_1_Bearing = owner.m_sSensoryData.m_RABSensorData[i].HorizontalBearing;
            observedRobotFound = true;


            if (m_sRobotData.OBSERVATION_MODE_TYPE == 1)
                // go through the packet of data received from this robot. search for the bearing at which this robot observed robot owner.m_sSensoryData.m_unRobotId
                observedRobotId_1_SelfBearingORAngularAcceleration = owner.GetSelfBearingFromRABPacket(owner.m_sSensoryData.m_unRobotId,
                                                                                                       owner.m_sSensoryData.m_RABSensorData, i, m_unRobotId);
            else if (m_sRobotData.OBSERVATION_MODE_TYPE == 2)
                observedRobotId_1_SelfBearingORAngularAcceleration = owner.GetAnguAccelerFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i, m_unRobotId);

            else
            {
                std::cerr << "Observation modes can be of one of two types";
                exit(-1);
            }

            break;
        }
    }

    return observedRobotFound;
}

/******************************************************************************/
/******************************************************************************/

