#include "bayesianinferencefeaturevector.h"
#include "assert.h"
#include <iostream>
//#include <std.

/******************************************************************************/
/******************************************************************************/

#define MODELSTARTTIME 450.0 // used for your sliding window - computing features the old method with ComputeFeatureValues_Old()

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned CBayesianInferenceFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
double   CBayesianInferenceFeatureVector::FEATURE_RANGE             = 15.0; //cm // was 60 cm for the old features

/******************************************************************************/
/******************************************************************************/

int CBayesianInferenceFeatureVector::m_iShortTimeWindowLength  = 0u;
int CBayesianInferenceFeatureVector::m_iMediumTimeWindowLength = 0u;
int CBayesianInferenceFeatureVector::m_iLongTimeWindowLength   = 0u;


/******************************************************************************/
/******************************************************************************/
#define ROBOTID_NOT_IN_SIGNAL          999
#define ROBOTSELFBEARING_NOT_IN_SIGNAL 999.0f
#define ROBOTSELFACCELERATION_NOT_IN_SIGNAL 999.0f
/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::CBayesianInferenceFeatureVector()
{
    //m_pcListObservedRobots.clear();
}

/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::~CBayesianInferenceFeatureVector()
{}

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::SimulationStep(std::string &swarmbehav, std::vector<int> &beaconrobots_ids)
{
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        /* this loop is quadratic in computational complexity. we can improve on this.*/

        bool     b_ObservedRobotFound(false);
        bool     b_ObservedRobotIsBeacon(false);
        unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i);

        /* Ignore the homing beacon in fault detection */
        if( (swarmbehav.compare("SWARM_HOMING") == 0) && (u_ObservedRobotId == 0u))
        {
                b_ObservedRobotIsBeacon = true;
        }

        /* Ignore the foraging beacon(s) in fault detection */
        if((swarmbehav.compare("SWARM_FORAGING") == 0) && (std::find(beaconrobots_ids.begin(), beaconrobots_ids.end(), u_ObservedRobotId) != beaconrobots_ids.end()))
        {
                b_ObservedRobotIsBeacon = true;
        }

        //for (t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
        t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
        while(it_listobrob != m_pcListObservedRobots.end())
        {
            if(u_ObservedRobotId == it_listobrob->m_unRobotId)
            {
                b_ObservedRobotFound = true;

                /* If beacon robot already in m_pcListObservedRobots, it will be removed*/
                if(b_ObservedRobotIsBeacon)
                    it_listobrob = m_pcListObservedRobots.erase(it_listobrob);

                break; // each robot id is represented only once in the m_pcListObservedRobots list
            }
            ++it_listobrob;
        }

        /* add the robot u_ObservedRobotId to the list of observed robots*/
        if(!b_ObservedRobotIsBeacon)
            if (!b_ObservedRobotFound && u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL)
                m_pcListObservedRobots.push_back(BayesInference_ObservedRobots_FeatureVector((*this), m_sSensoryData.m_rTime, u_ObservedRobotId));
    }


    /*
    But when do we delete the observed robot from the list of observed robots, if it has not been observed for a long time.
    We don't delete it, but we can just refresh the priors perodically, as we would do for the observed robots
    */
    t_ListObservedRobots::iterator it_listobrob = m_pcListObservedRobots.begin();
    while (it_listobrob != m_pcListObservedRobots.end())
    {
        unsigned RefreshPriorsThreshold = 1000000u; /* For now we don't refresh it. In the future we might want to, specially since the distributions of the proportion of interactions that are positive (say in the presence of sensory input from a neighbouring robot) may not be stationary */
        if((it_listobrob->u_TimeSinceLastRefresh) > RefreshPriorsThreshold)
            it_listobrob->RefreshPriors();

        /*
         * Gets observation of average_angularacceleration and of distance travelled by robot in last 1s, 5s, and 10s
           Robots that are not observed in the current time-step are merely marked as such instead of clearing the entire window if even one observation is missed
        */
        it_listobrob->EstimateOdometry();



        /* One of our observations is the proportion of the last 10s when the robot had atleast one neighbour. For this we need to know when the robot was not observed in this 10 s window. */
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
        if (!b_ObservedRobotFound) /* the robot is not observed; used to track proporton of last 10s when observed robot has atleast one neighbour. this feature is not used any more to my knowledge */
        {
            it_listobrob->ClearNbrs();
        }


        ++it_listobrob;
    }


    unsigned count = 0;
    ObservedRobotIDs.clear(); ObservedRobotIDs_range.clear();
    ObservedRobotFVs.clear(); ObservedRobotFVs_Min_Number_Featureobservations.clear();
    ObservedRobotFVs_Number_Featureobservations_SM.clear(); ObservedRobotFVs_Number_Featureobservations_nSM.clear(); ObservedRobotFVs_Number_Featureobservations_M.clear();

    for (it_listobrob = m_pcListObservedRobots.begin(); it_listobrob != m_pcListObservedRobots.end(); ++it_listobrob)
    {
        // Only update the posterior distribution for robots you have new observations for
        for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
        {
            unsigned range = 99u;
            unsigned u_ObservedRobotId = GetIdFromRABPacket(m_sSensoryData.m_RABSensorData, i, &range);
            if(u_ObservedRobotId == (it_listobrob->m_unRobotId))
            {

                assert(u_ObservedRobotId != ROBOTID_NOT_IN_SIGNAL);
                it_listobrob->ComputeFeatureValues();

                /*
                 *  WHEN DO WE REPORT ON ID AND FV OF OF OBSERVED ROBOT
                 *  AFTER FIXED NUMBER OF OBSERVATIONS?
                 *  VARIANCE OF DISTRIBUTION LESS THAN THRESHOLD
                 *  BASED ON BAYESIAN CREDIBLE REGIONS?
                 */

                    ObservedRobotIDs.push_back(it_listobrob->m_unRobotId);
                    ObservedRobotFVs.push_back(it_listobrob->GetValue());
                    ObservedRobotFVs_Min_Number_Featureobservations.push_back((unsigned) (it_listobrob->min_number_featureobservations));

                    ObservedRobotIDs_range.push_back(range);
                    ObservedRobotFVs_Number_Featureobservations_SM.push_back((unsigned)  (it_listobrob->number_featureobservations_2));
                    ObservedRobotFVs_Number_Featureobservations_nSM.push_back((unsigned) (it_listobrob->number_featureobservations_3));
                    ObservedRobotFVs_Number_Featureobservations_M.push_back((unsigned)   (it_listobrob->number_featureobservations_4));

                    count++;

            }
        }
    }
}

/******************************************************************************/
/******************************************************************************/

unsigned CBayesianInferenceFeatureVector::GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings& rab_packet, size_t rab_packet_index, unsigned *range)
{
    if ((rab_packet[rab_packet_index].Data[0] == m_sRobotData.BEACON_SIGNAL_MARKER) ||
        (rab_packet[rab_packet_index].Data[0] == m_sRobotData.NEST_BEACON_SIGNAL_MARKER))
    {
        if (rab_packet[rab_packet_index].Data[1] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[1] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            if(range != NULL)
               *range = (unsigned)(rab_packet[rab_packet_index].Range);
            return rab_packet[rab_packet_index].Data[2];
        }
        else
        {
            if(range != NULL)
               *range = 0u;
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
    else
    {
        if (rab_packet[rab_packet_index].Data[0] == m_sRobotData.VOTER_PACKET_MARKER || rab_packet[rab_packet_index].Data[0] == m_sRobotData.SELF_INFO_PACKET_MARKER)
        {
            if(range != NULL)
               *range = (unsigned)(rab_packet[rab_packet_index].Range);
            return rab_packet[rab_packet_index].Data[1];
        }
        else
        {
            if(range != NULL)
               *range = 0u;
            return ROBOTID_NOT_IN_SIGNAL;
        }
    }
}

/******************************************************************************/
/******************************************************************************/

float CBayesianInferenceFeatureVector::GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id)
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

CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::BayesInference_ObservedRobots_FeatureVector(CBayesianInferenceFeatureVector &owner_class,
                                                                                                                          Real TimeFirstObserved, unsigned ObservedRobotId):
    owner(owner_class), m_fTimeFirstObserved(TimeFirstObserved), m_unRobotId(ObservedRobotId), m_unValue(0)
{
    average_angularacceleration = (0.0f);

    m_pfFeatureValues        = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];
    m_pfAllFeatureValues     = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_pfFeatureValues[i]         = 0.0;
    }

    f_likelihood_variance = 2.0f;

    /************************************************************************************/
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/

     /* Lets initialise the priors */
     InitialisePriors();

     u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

/* Use the copy constructor since your structure has raw pointers and push_back the structure instance to list will just copy the pointer value */
CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::BayesInference_ObservedRobots_FeatureVector(const BayesInference_ObservedRobots_FeatureVector &ClassToCopy):
    owner(ClassToCopy.owner), m_fTimeFirstObserved(ClassToCopy.m_fTimeFirstObserved), m_unRobotId(ClassToCopy.m_unRobotId), m_unValue(0)
{
    average_angularacceleration = ClassToCopy.average_angularacceleration;

    m_pfFeatureValues         = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];
    m_pfAllFeatureValues      = new Real[CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES];


    for(unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
    {
        m_pfFeatureValues[i]         = 0.0;
    }


    f_likelihood_variance = ClassToCopy.f_likelihood_variance;

    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(owner.m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(owner.m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(owner.m_iLongTimeWindowLength);
    /************************************************************************************/

    /* Lets initialise the priors */
    InitialisePriors();

    u_TimeSinceLastObserved = 0u; u_TimeSinceLastObserved_DistMeasure = 0u;
}

/******************************************************************************/
/******************************************************************************/

CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::~BayesInference_ObservedRobots_FeatureVector()
{
    delete m_pfFeatureValues;
    delete m_pfAllFeatureValues;

    vec_RobPos_ShortRangeTimeWindow.clear();
    vec_RobPos_MediumRangeTimeWindow.clear();
    vec_RobPos_LongRangeTimeWindow.clear();
}

/******************************************************************************/
/******************************************************************************/

void CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::ComputeFeatureValues()
{
    // We are updating the priors now. Increment the time since we last refreshed the priors.
    u_TimeSinceLastRefresh++;


    FEATURE_RANGE = 30.0f; // cm
    Real DistToNearestNbr, CoM_closerangenbrs = 0.0f, CoM_farrangenbrs = 0.0f;

    /*
     * Remember that CountNeighbors returns -1 if the robot was not observed at the current time-step
    */

    Real  f_unCloseRangeNbrCount = CountNeighbors(0.0, FEATURE_RANGE/2.0f, DistToNearestNbr, CoM_closerangenbrs);
    Real  f_unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE/2.0f, FEATURE_RANGE, DistToNearestNbr, CoM_farrangenbrs);


    /* we need to be able to observe the robot to make any inference */
    if (f_unCloseRangeNbrCount == -1.0f)
    {
        assert(f_unFarRangeNbrCount == -1.0f); // just to be sure that the neighbour was observed. if this assertion fails there is a bug in the code
        list_NbrsInCloseProxm.clear();
        list_NbrsInFarProxm.clear();
    }


    if (f_unCloseRangeNbrCount != -1.0f)
    {
        assert(f_unFarRangeNbrCount != -1.0f); // just to be sure that the neighbour was observed. if this assertion fails there is a bug in the code

        list_NbrsInCloseProxm.push_back((f_unCloseRangeNbrCount >= 1.0f ? 1.0f: 0.0f));
        if(list_NbrsInCloseProxm.size() > m_iLongTimeWindowLength)
            list_NbrsInCloseProxm.pop_front();

        list_NbrsInFarProxm.push_back((f_unFarRangeNbrCount >= 1.0f ? 1.0f: 0.0f));
        if(list_NbrsInFarProxm.size() > m_iLongTimeWindowLength)
            list_NbrsInFarProxm.pop_front();


        Real newobservation_nearnbrs, newobservation_farnbrs;

        newobservation_nearnbrs = std::accumulate(list_NbrsInCloseProxm.begin(), list_NbrsInCloseProxm.end(), 0.0f) / list_NbrsInCloseProxm.size();
        newobservation_farnbrs  = std::accumulate(list_NbrsInFarProxm.begin(), list_NbrsInFarProxm.end(), 0.0f) / list_NbrsInFarProxm.size();

        m_pfFeatureValues[0] = newobservation_nearnbrs >= 0.5f ? 1.0f : 0.0f;
        assert(FEATURE_RANGE/2.0f == 15.0f);
        m_pfFeatureValues[1] = newobservation_farnbrs  >= 0.5f ? 1.0f : 0.0f;


        assert((owner.m_sSensoryData.m_rTime - (Real)(u_TimeSinceLastObserved)) >= 0.0f); /*time since the robot was last observed */
    }


    /* we need to be able to observe the robot to make any inference */

    if (m_fEstimated_Dist_LongTimeWindow != -1.0f)
    {
        list_Dist_LongRangeTimeWindow.push_back(m_fEstimated_Dist_LongTimeWindow);
        if(list_Dist_LongRangeTimeWindow.size() >= 100u)
            list_Dist_LongRangeTimeWindow.pop_front();

        m_pfFeatureValues[5] = m_fEstimated_Dist_LongTimeWindow >= (0.15f*(m_iLongTimeWindowLength * m_sRobotData.MaxLinearSpeed)) ? 1.0f : 0.0f; // using noisy measurements

        assert((owner.m_sSensoryData.m_rTime - (Real)u_TimeSinceLastObserved_DistMeasure) >= 0.0f); /*time since the robot was last observed */

        u_TimeSinceLastObserved_DistMeasure = owner.m_sSensoryData.m_rTime;
    }



    Real COM_nbrs_tmp; Real Range_ObserverToObserved;
    bool neighbours_present = (CountNeighbors(0.0, FEATURE_RANGE, DistToNearestNbr, COM_nbrs_tmp, &Range_ObserverToObserved) > 0.0f) ? true : false;

    /* we need to have motor observations to make an inference */
    if(average_angularacceleration != ROBOTSELFACCELERATION_NOT_IN_SIGNAL && m_fEstimated_Dist_MediumTimeWindow != -1.0f)
    {
        Real f_MotorOutput      = fabs(average_angularacceleration) * (m_fEstimated_Dist_MediumTimeWindow / (m_iMediumTimeWindowLength * m_sRobotData.MaxLinearSpeed));

        /* we need to discretise the data. we assume that a motor interaction occurs if f_MotorOutput exceeds 10% of max motor output */
        unsigned un_MotorOutput = (f_MotorOutput >= 0.10f)? 1u : 0u;

        /*Update priors*/
        /* Motor interactions in the presence of sensors */
        if (neighbours_present)
        {
            sensmotPrior_Beta_a = sensmotPrior_Beta_a + un_MotorOutput;
            sensmotPrior_Beta_b = sensmotPrior_Beta_b + 1u - un_MotorOutput;

            number_featureobservations_2 += 1.0f;
        }

        /* Motor interactions in the absence of sensors */
        if (!neighbours_present)
        {
            nosensmotPrior_Beta_a = nosensmotPrior_Beta_a + un_MotorOutput;
            nosensmotPrior_Beta_b = nosensmotPrior_Beta_b + 1u - un_MotorOutput;

            number_featureobservations_3 += 1.0f;
        }

        irrespsensmotPrior_Beta_a = irrespsensmotPrior_Beta_a + un_MotorOutput;
        irrespsensmotPrior_Beta_b = irrespsensmotPrior_Beta_b + 1u - un_MotorOutput;

        number_featureobservations_4 += 1.0f;
    }


    /* used to establish consensus on FVs. if choice between many fv for robot id, use the fv with the highest min_number_featureobservations */
    min_number_featureobservations = std::min(std::min(number_featureobservations_2, number_featureobservations_3),
                                              number_featureobservations_4);


    max_posterior_variance = std::max((Real)-1.0,
                                       ((Real)(sensmotPrior_Beta_a * sensmotPrior_Beta_b)) /
                                       ((Real)((sensmotPrior_Beta_a + sensmotPrior_Beta_b)*
                                       (sensmotPrior_Beta_a + sensmotPrior_Beta_b)*
                                       (sensmotPrior_Beta_a + sensmotPrior_Beta_b + 1u))) );

    /*the normalised max_posterior variance [0, 1] is going to be very much larger than the variance of the Beta distribution, */
    max_posterior_variance = std::max(max_posterior_variance,
                                      ((Real)(nosensmotPrior_Beta_a * nosensmotPrior_Beta_b)) /
                                      ((Real)((nosensmotPrior_Beta_a + nosensmotPrior_Beta_b)*
                                       (nosensmotPrior_Beta_a + nosensmotPrior_Beta_b)*
                                       (nosensmotPrior_Beta_a + nosensmotPrior_Beta_b + 1u))) );

    /*the normalised max_posterior variance [0, 1] is going to be very much larger than the variance of the Beta distribution, */
    max_posterior_variance = std::max(max_posterior_variance,
                                      ((Real)(irrespsensmotPrior_Beta_a * irrespsensmotPrior_Beta_b)) /
                                      ((Real)((irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b)*
                                       (irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b)*
                                       (irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b + 1u))) );


    if(number_featureobservations_2 == 0)
        m_pfFeatureValues[2] = 0.0f; // default value if robot not observed because of no neighbours in close proximity, or no proprioceptively computed angular acceleration received, or distance moved by robot in last 5s could not be computed (because robot not observed 5s back)
    else
        m_pfFeatureValues[2] = (((Real)sensmotPrior_Beta_a)       / ((Real)(sensmotPrior_Beta_a       + sensmotPrior_Beta_b)))       > 0.05f ? 1.0f : 0.0f;

    if(number_featureobservations_3 == 0)
        m_pfFeatureValues[3] = 0.0f;  // default value if robot not observed because always neighbours in close proximity, or no proprioceptively computed angular acceleration received, or distance moved by robot in last 5s could not be computed (because robot not observed 5s back)
    else
        m_pfFeatureValues[3] = (((Real)nosensmotPrior_Beta_a)     / ((Real)(nosensmotPrior_Beta_a     + nosensmotPrior_Beta_b)))     > 0.05f ? 1.0f : 0.0f;

    if (number_featureobservations_4 == 0)
        m_pfFeatureValues[4] = 0.0f;  //default value if no proprioceptively computed angular acceleration received, or distance moved by robot in last 5s could not be computed (because robot not observed 5s back)
    else
        m_pfFeatureValues[4] = (((Real)irrespsensmotPrior_Beta_a) / ((Real)(irrespsensmotPrior_Beta_a + irrespsensmotPrior_Beta_b))) > 0.05f ? 1.0f : 0.0f;

    assert(CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES == 6);

    m_unValue = 0;
    for (unsigned int i = 0; i < CBayesianInferenceFeatureVector::NUMBER_OF_FEATURES; i++)
        m_unValue += (unsigned)m_pfFeatureValues[i] * (1 << i);
}

/******************************************************************************/
/******************************************************************************/

Real CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real& dist_nearest_nbr,
                                                                                                  Real& CoM_nbrs, Real *Range_ObserverToObserved)
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
        dist_nearest_nbr = 101u; // max range is 100 cm. if no neighbours were detected set yo 101u

    if (count_nbrs > 0.0f)
        CoM_nbrs = CoM_nbrs / count_nbrs;


    if(Range_ObserverToObserved !=NULL)
        *Range_ObserverToObserved = observedRobotId_1_Range;

    return count_nbrs;
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

void CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::EstimateOdometry()
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


    m_fEstimated_Dist_ShortTimeWindow  = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_ShortRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Dist_MediumTimeWindow = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_MediumRangeTimeWindow, b_DataAvailable);
    m_fEstimated_Dist_LongTimeWindow   = TrackRobotDisplacement(step, observedRobotId_1_Range, observedRobotId_1_Bearing, delta_orientation,
                                                                vec_RobPos_LongRangeTimeWindow, b_DataAvailable);
}

/******************************************************************************/
/******************************************************************************/

Real CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::TrackRobotDisplacement(Real step, Real observedRobotId_1_Range,
                                                                                                          CRadians observedRobotId_1_Bearing, CRadians delta_orientation,
                                                                                                          std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable)
{
    Real displacement = -1.0f; /* if step is < displacement_vector.size, we return -1 as not enough time has elapsed to make an observation */

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
            displacement              = (pos_rot_trans - Pos_At_Start).Length();
        }
        else
        {
            /*
                We don't have the range and bearing observations of the robot to position it at the end of the pre-specified time interval. So we can't compute the displacement.
            */
            displacement              = -1.0f;
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

bool CBayesianInferenceFeatureVector::BayesInference_ObservedRobots_FeatureVector::GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing,
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

