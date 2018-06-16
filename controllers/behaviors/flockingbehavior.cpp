#include "flockingbehavior.h"
#include "assert.h"
#include <iostream>


/******************************************************************************/
/******************************************************************************/
#define ROBOTID_NOT_IN_SIGNAL          999
#define ROBOTSELFBEARING_NOT_IN_SIGNAL 999.0f
#define ROBOTSELFACCELERATION_NOT_IN_SIGNAL 999.0f
/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::CFlockingBehavior(unsigned VelocityTimeWindowLength) :
    m_cFlockingVector(0.0, 0.0),
    m_uVelocityTimeWindowLength(VelocityTimeWindowLength)
{
}

/******************************************************************************/
/******************************************************************************/
    
bool CFlockingBehavior::TakeControl()
{
    bool controltaken(false);

    m_cFlockingVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_flock = 0;


    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    for (it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
    {

        // Only update the FlockingVector using robots you are observing
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i);
            if(u_ObservedRobotId == (it_listobrob->m_unRobotId))
            {

                assert(u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL);

                if (it_listobrob->GetVelocity_MediumTimeWindow().GetX() != -10000.0f)// && m_sSensoryData.m_RABSensorData[i].Range < 25.0f)
                {
                    m_cFlockingVector +=it_listobrob->GetVelocity_MediumTimeWindow();
                    robotsinrange_flock++;
                }

            }
        }
    }


    m_fAvgProximityNbrs = 0.0f;
    m_cAggregationVector.Set(0.0f, 0.0f);
    unsigned robotsinrange_aggr = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        m_cAggregationVector += CVector2(m_sSensoryData.m_RABSensorData[i].Range, m_sSensoryData.m_RABSensorData[i].HorizontalBearing);
        m_fAvgProximityNbrs  += m_sSensoryData.m_RABSensorData[i].Range;
        robotsinrange_aggr++;
    }

    if(robotsinrange_aggr > 3)
    {
        m_cAggregationVector /= robotsinrange_aggr;
        m_fAvgProximityNbrs = m_fAvgProximityNbrs / robotsinrange_aggr;
    }
    else
    {
        m_cAggregationVector = CVector2(0.0f, 0.0f);
        m_fAvgProximityNbrs = 0.0f;
    }


    if(robotsinrange_flock > 0u)
    {
        m_cFlockingVector /= robotsinrange_flock;
        return true;
    }
    else
        return false;
}

/******************************************************************************/
/******************************************************************************/

// Move at average velocity of your neighbours
void CFlockingBehavior::Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed)
{
    //if neighbours very far away, put more weight on aggregation; else put more weight on flocking

    m_fAvgProximityNbrs /= 100.0f; // normalise - max range is 100cm.

    // m_fAvgProximityNbrs = m_fAvgProximityNbrs*m_fAvgProximityNbrs; // enable to have flocks moving faster, but with more distance between robots of aggregate

     //CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed * (0.75f * m_cFlockingVector.Normalize() + 0.25f * m_cAggregationVector.Normalize());
     CVector2 m_cHeadingVector =  m_sRobotData.MaxSpeed * ((1.0f - m_fAvgProximityNbrs) * m_cFlockingVector.Normalize() + m_fAvgProximityNbrs * m_cAggregationVector.Normalize());

     WheelSpeedsFromHeadingVector(m_cHeadingVector, fLeftWheelSpeed, fRightWheelSpeed);

     //fLeftWheelSpeed = (fLeftWheelSpeed + m_sSensoryData.f_LeftWheelSpeed)/2.0f;
     //fRightWheelSpeed = (fRightWheelSpeed + m_sSensoryData.f_RightWheelSpeed)/2.0f;
}

/******************************************************************************/
/******************************************************************************/

void CFlockingBehavior::PrintBehaviorIdentity()
{
    std::cout << "Flocking taking over";
}

/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::ObservedNeighbours::ObservedNeighbours(CFlockingBehavior &owner_class, Real TimeFirstObserved, unsigned ObservedRobotId):
    owner(owner_class), m_fTimeFirstObserved(TimeFirstObserved), m_unRobotId(ObservedRobotId)
{
    average_angularacceleration = (0.0f);

    /************************************************************************************/
    m_fEstimated_Velocity_ShortTimeWindow = CVector2(-10000.0f, -10000.0f); m_fEstimated_Velocity_MediumTimeWindow = CVector2(-10000.0f, -10000.0f);
    m_fEstimated_Velocity_LongTimeWindow = CVector2(-10000.0f, -10000.0f);

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    /************************************************************************************/

     u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

/* Use the copy constructor since your structure has raw pointers and push_back the structure instance to list will just copy the pointer value */
CFlockingBehavior::ObservedNeighbours::ObservedNeighbours(const ObservedNeighbours &ClassToCopy):
    owner(ClassToCopy.owner), m_fTimeFirstObserved(ClassToCopy.m_fTimeFirstObserved), m_unRobotId(ClassToCopy.m_unRobotId)
{
    average_angularacceleration = ClassToCopy.average_angularacceleration;

    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_fEstimated_Velocity_ShortTimeWindow = CVector2(-10000.0f, -10000.0f); m_fEstimated_Velocity_MediumTimeWindow = CVector2(-10000.0f, -10000.0f);
    m_fEstimated_Velocity_LongTimeWindow = CVector2(-10000.0f, -10000.0f);

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_uVelocityTimeWindowLength);
    /************************************************************************************/


    u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

CFlockingBehavior::ObservedNeighbours::~ObservedNeighbours()
{
    vec_RobPos_ShortRangeTimeWindow.clear();
    vec_RobPos_MediumRangeTimeWindow.clear();
    vec_RobPos_LongRangeTimeWindow.clear();
}

/******************************************************************************/
/******************************************************************************/

void CFlockingBehavior::SimulationStep()
{
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        /* this loop is quadratic in computational complexity. we can improve on this.*/

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

        /* add the robot u_ObservedRobotId to the list of observed robots*/
        if (!b_ObservedRobotFound && u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL)
            m_pcListObservedRobots.push_back(ObservedNeighbours((*this), m_sSensoryData.m_rTime, u_ObservedRobotId));
    }


    /*
    But when do we delete the observed robot from the list of observed robots, if it has not been observed for a long time.
    We don't delete it, but we can just refresh the priors perodically, as we would do for the observed robots
    */
    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    while (it_listobrob != m_pcListObservedRobots.end())
    {
        /*
         * Gets observation of average_angularacceleration and of distance travelled by robot in last 1s, 5s, and 10s
           Robots that are not observed in the current time-step are merely marked as such instead of clearing the entire window if even one observation is missed
        */
        it_listobrob->EstimateOdometry();

        ++it_listobrob;
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned CFlockingBehavior::GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index)
{
    if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER)
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

float CFlockingBehavior::GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index, unsigned observerd_robot_id)
{
    size_t index_start = 0;
    if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER)
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

void CFlockingBehavior::ObservedNeighbours::EstimateOdometry()
{
    Real observedRobotId_1_Range; CRadians observedRobotId_1_Bearing; Real observedRobotId_1_SelfBearingOrAngularAcceleration;

    bool b_DataAvailable = GetObservedRobotRangeBearing(observedRobotId_1_Range, observedRobotId_1_Bearing, observedRobotId_1_SelfBearingOrAngularAcceleration);
    // assert(b_DataAvailable); // WHAT IF THE ROBOT IS NOT OBSERVED AT THE CURRENT TIME-STEP


    //assert(observedRobotId_1_SelfBearingOrAngularAcceleration != ROBOTSELFACCELERATION_NOT_IN_SIGNAL);
    average_angularacceleration = observedRobotId_1_SelfBearingOrAngularAcceleration;


    /*
     * Computing angle rotated by robot in one tick
    */
    CRadians delta_orientation = CRadians(owner.m_sRobotData.seconds_per_iterations * ((-owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed)
                                                                                       / (owner.m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

    /*if (owner.m_sSensoryData.m_unRobotId == 0u)
        printf("%f\t",ToDegrees(delta_orientation).GetValue());*/


    Real step = owner.m_sSensoryData.m_rTime - m_fTimeFirstObserved;


    assert(step >= 0.0f);


    m_fEstimated_Velocity_ShortTimeWindow  = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_ShortRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Velocity_MediumTimeWindow = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_MediumRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Velocity_LongTimeWindow   = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_LongRangeTimeWindow, b_DataAvailable);
}

/******************************************************************************/
/******************************************************************************/

CVector2 CFlockingBehavior::ObservedNeighbours::TrackRobotDisplacement(Real step, Real observedRobotId_1_Range, CRadians observedRobotId_1_Bearing,
                                                                       CRadians delta_orientation, std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable)
{
    CVector2 displacement(-10000.0f, -10000.0f); /* if step is < displacement_vector.size, we return -10000 as not enough time has elapsed to make an observation */

    assert(step >= 0.0f);

    /*
     * Returns magnitude of displacement vector of the robot in the pre-specified time interval. If the robot is unobservable at the end of the pre-specified time interval, the function returns -1.
     */
    if(step < (Real)displacement_vector.size())
    {

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

        displacement_vector[(unsigned)(step)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(step)].b_DataAvailable)
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[(unsigned)(step)].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing),
                                                                   observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[(unsigned)(step)].Range_At_Start           = -1.0f;
            displacement_vector[(unsigned)(step)].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[(unsigned)(step)].Pos_At_Start.Set(-1.0f, -1.0f);
        }


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
                    ((owner.m_sSensoryData.f_LeftWheelSpeed + owner.m_sSensoryData.f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }


        size_t t = ((unsigned)(step)%displacement_vector.size());
        assert(t < displacement_vector.size());



        if(displacement_vector[(unsigned)(t)].b_DataAvailable && b_DataAvailable)
        {
            CVector2 tmp_pos           = CVector2(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
            CVector2 tmp_pos_rot       = tmp_pos.Rotate(displacement_vector[t].NetRotationSinceStart);
            CVector2 pos_rot_trans     = tmp_pos_rot + displacement_vector[t].NetTranslationSinceStart;

            CVector2 Pos_At_Start      = displacement_vector[t].Pos_At_Start;

            /*
             * Computing average displacement
             */
            displacement              = (pos_rot_trans - Pos_At_Start);
        }
        else
        {
            /*
                We don't have the range and bearing observations of the robot to position it at the end of the pre-specified time interval. So we can't compute the displacement.
            */
            displacement              = CVector2(-10000.0f, -10000.0f);
        }

        /*if(m_unRobotId == 16 && owner.m_sSensoryData.m_unRobotId ==19 && displacement_vector.size() == 100)
        {
            printf("displacement %f \n", displacement);
        }*/


        /* Preparing the start recorded data to be used at the end of the pre-specified time interval */
        displacement_vector[(unsigned)(t)].b_DataAvailable = b_DataAvailable;
        if (displacement_vector[(unsigned)(t)].b_DataAvailable)
        {
            displacement_vector[t].Range_At_Start           = observedRobotId_1_Range;
            displacement_vector[t].Bearing_At_Start         = observedRobotId_1_Bearing;
            displacement_vector[t].Pos_At_Start.Set(observedRobotId_1_Range * Cos(observedRobotId_1_Bearing), observedRobotId_1_Range * Sin(observedRobotId_1_Bearing));
        }
        else
        {
            displacement_vector[t].Range_At_Start           = -1.0f;
            displacement_vector[t].Bearing_At_Start.SetValue(-1.0f);
            displacement_vector[t].Pos_At_Start.Set(-1.0f, -1.0f);
        }
        displacement_vector[t].TimeSinceStart = 0.0f;
        displacement_vector[t].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[t].NetRotationSinceStart.SetValue(0.0f);
    }


    return displacement;
}

/******************************************************************************/
/******************************************************************************/

bool CFlockingBehavior::ObservedNeighbours::GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing,
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

            observedRobotId_1_SelfBearingORAngularAcceleration = owner.GetAnguAccelerFromRABPacket(owner.m_sSensoryData.m_RABSensorData, i, m_unRobotId);

            if (m_sRobotData.OBSERVATION_MODE_TYPE != 3)
            {
                std::cerr << "Observation modes can be of one of three types";
                exit(-1);
            }

            break;
        }
    }

    return observedRobotFound;
}

/******************************************************************************/
/******************************************************************************/
