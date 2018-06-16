#include "propriofeaturevector.h"
#include "assert.h"
#include <iostream>

/******************************************************************************/
/******************************************************************************/

#define MODELSTARTTIME 300.0  //450.0 // used for your sliding window

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::NUMBER_OF_FEATURES        = 6;
unsigned int CProprioceptiveFeatureVector::MAX_NUMBER_OF_FEATURES    = 15;
unsigned int CProprioceptiveFeatureVector::NUMBER_OF_FEATURE_VECTORS = 0;
double       CProprioceptiveFeatureVector::FEATURE_RANGE             = 30.0; //cm  // was 60 cm for the old features

/* Length of time windows for observing neighbours of your neighbours and the distance they travel */
int CProprioceptiveFeatureVector::m_iShortTimeWindowLength  = 5u; //10u;
int CProprioceptiveFeatureVector::m_iMediumTimeWindowLength = 50u; //50u;
int CProprioceptiveFeatureVector::m_iLongTimeWindowLength   = 100u; //100u;

/******************************************************************************/
/******************************************************************************/

//CVector2 pos_ref(0.0f, 0.0f);


CProprioceptiveFeatureVector::CProprioceptiveFeatureVector()
{
    m_unValue  = 0;
    m_unLength = NUMBER_OF_FEATURES;

    NUMBER_OF_FEATURE_VECTORS = 1 << NUMBER_OF_FEATURES;

    m_pfFeatureValues         = new Real[m_unLength];
    m_piLastOccuranceEvent    = new int[m_unLength];
    m_piLastOccuranceNegEvent = new int[m_unLength];

    m_pfAllFeatureValues     = new Real[NUMBER_OF_FEATURES];


    m_iEventSelectionTimeWindow = MODELSTARTTIME;

    for(unsigned int i = 0; i < NUMBER_OF_FEATURES; i++)
    {
        m_piLastOccuranceEvent[i]    = 0;
        m_piLastOccuranceNegEvent[i] = 0;

        m_pfFeatureValues[i]         = 0.0;
    }


    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    m_unNbrsCurrQueueIndex = 0;

    m_unSumTimeStepsNbrsRange0to30 = 0;
    m_unSumTimeStepsNbrsRange30to60 = 0;

    m_punNbrsRange0to30AtTimeStep  = new unsigned int[m_iEventSelectionTimeWindow];
    m_punNbrsRange30to60AtTimeStep = new unsigned int[m_iEventSelectionTimeWindow];


    // keeping track of distance travelled by bot in last 100 time-steps
    m_iDistTravelledTimeWindow = 100;
    m_unCoordCurrQueueIndex    = 0;

    m_fSquaredDistTravelled = 0.0;
    m_fCumulativeDistTravelled = 0.0f;

    m_pvecCoordAtTimeStep = new CVector2[m_iDistTravelledTimeWindow];
    m_pfDistAtTimeStep    = new Real[m_iDistTravelledTimeWindow];



    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/

    m_unQueueIndex_ShortRangeTimeWindow = 0u; m_unQueueIndex_MediumRangeTimeWindow = 0u; m_unQueueIndex_LongRangeTimeWindow = 0u;
    m_unSumTimeStepsNbrs_ShortRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_MediumRangeTimeWindow = 0u; m_unSumTimeStepsNbrs_LongRangeTimeWindow = 0u;
    m_punNbrs_ShortRangeTimeWindow   = new unsigned int[m_iLongTimeWindowLength];
    m_punNbrs_MediumRangeTimeWindow  = new unsigned int[m_iLongTimeWindowLength];
    m_punNbrs_LongRangeTimeWindow    = new unsigned int[m_iLongTimeWindowLength];
    m_fEstimated_Dist_ShortTimeWindow = 0.0f; m_fEstimated_Dist_MediumTimeWindow = 0.0f; m_fEstimated_Dist_LongTimeWindow = 0.0f;

    vec_RobPos_ShortRangeTimeWindow.resize(m_iShortTimeWindowLength);
    vec_RobPos_MediumRangeTimeWindow.resize(m_iMediumTimeWindowLength);
    vec_RobPos_LongRangeTimeWindow.resize(m_iLongTimeWindowLength);
    /************************************************************************************/
}

/******************************************************************************/
/******************************************************************************/

CProprioceptiveFeatureVector::~CProprioceptiveFeatureVector()
{
    delete m_pfFeatureValues;

    delete m_piLastOccuranceEvent;
    delete m_piLastOccuranceNegEvent;

    delete m_punNbrsRange0to30AtTimeStep;
    delete m_punNbrsRange30to60AtTimeStep;

    delete m_pvecCoordAtTimeStep;
    delete m_pfDistAtTimeStep;


    delete m_punNbrs_ShortRangeTimeWindow;
    delete m_punNbrs_MediumRangeTimeWindow;
    delete m_punNbrs_LongRangeTimeWindow;
}

/******************************************************************************/
/******************************************************************************/

unsigned CProprioceptiveFeatureVector::GetValue() const
{
    return m_unValue;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::GetLength() const
{
    return m_unLength;
}

/******************************************************************************/
/******************************************************************************/

unsigned int CProprioceptiveFeatureVector::SimulationStep()
{
    // too small thresholded distance. robot moves very little in one control-cycle. better to intergrate distance over time and use threshold on that
    m_fVelocityThreshold            = 0.05  * (m_sRobotData.MaxLinearSpeed); // max speed is 0.5 cm per control cycle
    //m_fAccelerationThreshold      = 0.05  * (m_sRobotData.MaxLinearAcceleration); // max change in speed is \[PlusMinus]1 cm per control cycle per control cycle

    m_fAccelerationThreshold        = 2.0f; // max change in speed is \[PlusMinus]2.5 cm over 5 control cycles  per control cycle per control cycle


    // 0.032
    m_tAngularVelocityThreshold     = 0.15  * (m_sRobotData.MaxAngularSpeed); //Maximum angular speed is \[PlusMinus]21.621 degrees per control cycle
    m_tAngularAccelerationThreshold = 0.15  * (m_sRobotData.MaxAngularAcceleration); //Maximum angular acceleation is \[PlusMinus]43.242 degrees per control cycle per control cycle


    // the time window is in ticks and the maxlinearspeed is in cm / tick
    m_fSquaredDistThreshold = (0.05 * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow)) *
                              (0.05 * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow));


    m_fCumulativeDistThreshold = (0.05 * (m_sRobotData.MaxLinearSpeed*(Real)m_iDistTravelledTimeWindow));






    /*assert(m_iShortTimeWindowLength  == (unsigned)m_sRobotData.iterations_per_second);
    assert(m_iMediumTimeWindowLength == (unsigned)m_sRobotData.iterations_per_second * 5u);
    assert(m_iLongTimeWindowLength   == (unsigned)m_sRobotData.iterations_per_second * 10u);*/

    ComputeFeatureValues();
    m_unValue = 0;

    for (unsigned int i = 0; i < m_unLength; i++)
        m_unValue += (unsigned int)m_pfFeatureValues[i] * (1 << i);

     /*if (m_sSensoryData.m_unRobotId == 8u)
         std::cerr << " FV " << m_unValue << std::endl;*/
}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::ComputeFeatureValues()
{
    CProprioceptiveFeatureVector::FEATURE_RANGE = 30.0f;


    unsigned  unCloseRangeNbrCount = CountNeighbors(FEATURE_RANGE/2.0f);
    unsigned  unFarRangeNbrCount   = CountNeighbors(FEATURE_RANGE) - unCloseRangeNbrCount;

    bool neighbours_present = ((unCloseRangeNbrCount + unFarRangeNbrCount) > 0) ? true : false;


    int CurrentStepNumber = m_sSensoryData.m_rTime;

    // Feature (from LS to MS bits in FV)
    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps
    if(CurrentStepNumber >= m_iEventSelectionTimeWindow)
    {
        // decision based on the last X time-steps
        if(m_unSumTimeStepsNbrsRange0to30 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[0] = 1.0;
        else
            m_pfAllFeatureValues[0] = 0.0;

        if(m_unSumTimeStepsNbrsRange30to60 > (unsigned)(0.5*(double)m_iEventSelectionTimeWindow))
            m_pfAllFeatureValues[1] = 1.0;
        else
            m_pfAllFeatureValues[1] = 0.0;

        // removing the fist entry of the moving time window  from the sum
        m_unSumTimeStepsNbrsRange0to30  -=  m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex];
        m_unSumTimeStepsNbrsRange30to60 -=  m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex];
    }

    // adding new values into the queue
    if (unCloseRangeNbrCount > 0)
    {
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange0to30++;
    }
    else
        m_punNbrsRange0to30AtTimeStep[m_unNbrsCurrQueueIndex] = 0;

    if (unFarRangeNbrCount > 0)
    {
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 1;
        m_unSumTimeStepsNbrsRange30to60++;
    }
    else
        m_punNbrsRange30to60AtTimeStep[m_unNbrsCurrQueueIndex] = 0;


    m_unNbrsCurrQueueIndex = (m_unNbrsCurrQueueIndex + 1) % m_iEventSelectionTimeWindow;



    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration

    if(neighbours_present &&
            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(!neighbours_present &&
            (m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
             m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

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


    // Motors
    //5th: distance travelled by bot in past 100 time-steps. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    CVector2 vecAgentPos = m_sSensoryData.pos;

    m_fCumulativeDistTravelled += m_sSensoryData.dist;

    if(CurrentStepNumber >= m_iDistTravelledTimeWindow)
    {
        // distance travelled in last 100 time-steps
        m_fSquaredDistTravelled = (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX()) *
                (vecAgentPos.GetX() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetX())  +
                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY()) *
                (vecAgentPos.GetY() - m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex].GetY());


        //pos_ref = m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex];


        // removing the distance travelled in the first time-step of the moving time-window from the queue
        m_fCumulativeDistTravelled -= m_pfDistAtTimeStep[m_unCoordCurrQueueIndex];


        // decision based on distance travelled in the last 100 time-steps
        /*if(m_fCumulativeDistTravelled >= m_fCumulativeDistThreshold)
            m_pfAllFeatureValues[4] = 1.0;
        else
            m_pfAllFeatureValues[4] = 0.0;*/


        /*if(m_fSquaredDistTravelled >= m_fSquaredDistThreshold)
            m_pfAllFeatureValues[4] = 1.0;
        else
            m_pfAllFeatureValues[4] = 0.0;*/
    }

    m_fEstimated_Dist_LongTimeWindow   = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_LongRangeTimeWindow);
    m_pfAllFeatureValues[4] = (m_fEstimated_Dist_LongTimeWindow   >  sqrt(m_fSquaredDistThreshold)) ? 1.0f: 0.0f;



    //std::cout << " cumulative distance " << m_fCumulativeDistTravelled << std::endl;
    //std::cout << " LinearSpeed " <<  m_sSensoryData.LinearSpeed << " angular speed " << m_sSensoryData.AngularSpeed << std::endl;
    //std::cout << " LinearAcceleration " <<  m_sSensoryData.LinearAcceleration << " angular acceleration " << m_sSensoryData.AngularAcceleration << std::endl;

    // adding new coordinate values into the queue
    m_pvecCoordAtTimeStep[m_unCoordCurrQueueIndex] = vecAgentPos;

    // adding distance travelled at last time-step into queue
    m_pfDistAtTimeStep[m_unCoordCurrQueueIndex] = m_sSensoryData.dist;

    m_unCoordCurrQueueIndex = (m_unCoordCurrQueueIndex + 1) % m_iDistTravelledTimeWindow;


    if((m_sSensoryData.AngularAcceleration > m_tAngularAccelerationThreshold ||
        m_sSensoryData.AngularAcceleration < -m_tAngularAccelerationThreshold))
    {
         m_piLastOccuranceEvent[5] = CurrentStepNumber;
    }


    // 6th: set if the robot changed its angular speed (per control cycle) atleast once in the past time-window.
    if ((CurrentStepNumber - m_piLastOccuranceEvent[5]) <= m_iEventSelectionTimeWindow)
    {
        m_pfAllFeatureValues[5] = 1.0;
    }
    else
    {
        m_pfAllFeatureValues[5] = 0.0;
    }

    //6th: linear speed, higher than 5% of max. speed is accepted as feature=1 OR angular speed higher than 5% of max. angular speed is accepted as feature=1
    /*m_pfAllFeatureValues[5] = (m_sSensoryData.LinearSpeed  >= m_fVelocityThreshold) ? 1.0:0.0;*/


    // adding the selected features into the feature vector
    for(size_t i = 0; i <  NUMBER_OF_FEATURES; ++i)
        m_pfFeatureValues[i] = m_pfAllFeatureValues[i];
}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::ComputeFeatureValues_onlynbrsandactuators()
{
    Real f1, f2, f3, f4, f5, f6;
    unsigned  unNbrCount = CountNeighbors(FEATURE_RANGE);

    unsigned  unNbrCount1 = CountNeighbors(15.0f);
    unsigned  unNbrCount2 = CountNeighbors(30.0f);
    unsigned  unNbrCount3 = CountNeighbors(45.0f);

    /*
     * Time since the robot was first observed
     */
    Real CurrentStepNumber =  m_sSensoryData.m_rTime;

    unsigned num_nbrs_threshold = 1u; Real queue_length_threshold = 0.5f;
    /*f1 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount, num_nbrs_threshold,
                               m_iShortTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_ShortRangeTimeWindow, m_unQueueIndex_ShortRangeTimeWindow, m_punNbrs_ShortRangeTimeWindow);
    f2 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount, num_nbrs_threshold,
                               m_iMediumTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_MediumRangeTimeWindow, m_unQueueIndex_MediumRangeTimeWindow, m_punNbrs_MediumRangeTimeWindow);
    f3 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_LongRangeTimeWindow, m_unQueueIndex_LongRangeTimeWindow, m_punNbrs_LongRangeTimeWindow);*/

    queue_length_threshold = 0.25f;
    f1 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount1, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_ShortRangeTimeWindow, m_unQueueIndex_ShortRangeTimeWindow, m_punNbrs_ShortRangeTimeWindow);

    queue_length_threshold = 0.5f;
    f2 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount1, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_MediumRangeTimeWindow, m_unQueueIndex_MediumRangeTimeWindow, m_punNbrs_MediumRangeTimeWindow);

    queue_length_threshold = 0.75f;
    f3 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount1, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_LongRangeTimeWindow, m_unQueueIndex_LongRangeTimeWindow, m_punNbrs_LongRangeTimeWindow);



    m_fEstimated_Dist_ShortTimeWindow  = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_ShortRangeTimeWindow);
    m_fEstimated_Dist_MediumTimeWindow = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_MediumRangeTimeWindow);
    m_fEstimated_Dist_LongTimeWindow   = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_LongRangeTimeWindow);


    /*if (m_sSensoryData.m_unRobotId == 1u)
    {
            std::cout << " Dist - short time window " <<   m_fEstimated_Dist_ShortTimeWindow << std::endl;
            std::cout << " Dist - medium time window " <<  m_fEstimated_Dist_MediumTimeWindow << std::endl;
            std::cout << " Dist - long time window " <<    m_fEstimated_Dist_LongTimeWindow << std::endl;
    }*/


    Real disp_ShortWindow_Threshold  = 0.25f;
    Real disp_MediumWindow_Threshold = 0.25f;
    Real disp_LongWindow_Threshold   = 0.25f;
    // MaxLinearSpeed in cm / control cycle
//    f4 = (m_fEstimated_Dist_ShortTimeWindow  >  (disp_ShortWindow_Threshold  * ((Real)m_iShortTimeWindowLength)  * m_sRobotData.MaxLinearSpeed)) ? 1.0f: 0.0f;
//    f5 = (m_fEstimated_Dist_MediumTimeWindow >  (disp_MediumWindow_Threshold * ((Real)m_iMediumTimeWindowLength) * m_sRobotData.MaxLinearSpeed)) ? 1.0f: 0.0f;
//    f6 = (m_fEstimated_Dist_LongTimeWindow   >  (disp_LongWindow_Threshold   * ((Real)m_iLongTimeWindowLength)   * m_sRobotData.MaxLinearSpeed)) ? 1.0f: 0.0f;
    /*
     * Better to have a fixed absolute threshold instead of a fixed percentage as small percentages at large timewindows would be greatly affected by noise for the small time windows.
     */
    f4 = (m_fEstimated_Dist_ShortTimeWindow  >  (4.0f)) ? 1.0f: 0.0f;
    f5 = (m_fEstimated_Dist_MediumTimeWindow >  (4.0f)) ? 1.0f: 0.0f;
    f6 = (m_fEstimated_Dist_LongTimeWindow   >  (4.0f)) ? 1.0f: 0.0f;


    m_pfFeatureValues[0] = f1;
    m_pfFeatureValues[1] = f2;
    m_pfFeatureValues[2] = f3;
    m_pfFeatureValues[3] = f4;
    m_pfFeatureValues[4] = f5;
    m_pfFeatureValues[5] = f6;

    assert(CProprioceptiveFeatureVector::NUMBER_OF_FEATURES == 6);
}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::ComputeFeatureValues_NoAngularVelocityUsed()
{
    Real f1, f2, f5, f6;

    unsigned  unNbrCount1 = CountNeighbors(5.0f);
    unsigned  unNbrCount2 = CountNeighbors(10.0f);

    /*
     * Time since the robot was first observed
     */
    Real CurrentStepNumber =  m_sSensoryData.m_rTime;

    // Sensors
    //1st: set if bot has atleast one neighbor in range 0-30cm in the majority of of past X time-steps
    //2nd: set if bot has atleast one neighbor in range 30-60cm in the majority of of past X time-steps
    unsigned num_nbrs_threshold = 2u; Real queue_length_threshold;
    queue_length_threshold = 0.750f;
    f1 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount1, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_ShortRangeTimeWindow, m_unQueueIndex_ShortRangeTimeWindow, m_punNbrs_ShortRangeTimeWindow);

    queue_length_threshold = 0.750f;
    f2 = TrackNeighborsInQueue(CurrentStepNumber, unNbrCount2-unNbrCount1, num_nbrs_threshold,
                               m_iLongTimeWindowLength, queue_length_threshold,
                               m_unSumTimeStepsNbrs_MediumRangeTimeWindow, m_unQueueIndex_MediumRangeTimeWindow, m_punNbrs_MediumRangeTimeWindow);





    Real tmp = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_ShortRangeTimeWindow);
    m_sSensoryData.vec_linearspeeds[m_sSensoryData.vec_linearspeeds_index] = tmp;
    m_sSensoryData.LinearAcceleration = tmp - m_sSensoryData.vec_linearspeeds[(m_sSensoryData.vec_linearspeeds_index + 1u)%m_sSensoryData.vec_linearspeeds.size()];

    m_sSensoryData.vec_linearspeeds_index = (m_sSensoryData.vec_linearspeeds_index + 1) % m_sSensoryData.vec_linearspeeds.size();



    // Sensors-motor interactions
    // Set if the occurance of the following event, atleast once in time window X
    // 3rd: distance to nbrs 0-6 && change in angular acceleration
    // 4th: no neighbors detected  && change in angular acceleration



    bool neighbours_present = (unNbrCount2>0)?true:false;
    if(neighbours_present &&
            (m_sSensoryData.LinearAcceleration >=  1.5f ||
             m_sSensoryData.LinearAcceleration <= -1.5f))
    {
        m_piLastOccuranceEvent[2] = CurrentStepNumber;
    }

    if(!neighbours_present &&
            (m_sSensoryData.LinearAcceleration >=  1.5f ||
             m_sSensoryData.LinearAcceleration <= -1.5f))
    {
        m_piLastOccuranceEvent[3] = CurrentStepNumber;
    }

    for(unsigned int featureindex = 2; featureindex <=3; featureindex++)
    {
        if ((((int)CurrentStepNumber) - m_piLastOccuranceEvent[featureindex]) <= 300u)
        {
            m_pfAllFeatureValues[featureindex] = 1.0;
        }
        else
        {
            m_pfAllFeatureValues[featureindex] = 0.0;
        }
    }

    m_fEstimated_Dist_MediumTimeWindow = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_MediumRangeTimeWindow);
    m_fEstimated_Dist_LongTimeWindow   = TrackRobotDisplacement(CurrentStepNumber, vec_RobPos_LongRangeTimeWindow);


    /*if (m_sSensoryData.m_unRobotId == 8u)
    {
            std::cout << " LinearAcceleration " <<         m_sSensoryData.LinearAcceleration << std::endl;
            std::cout << " Dist - short time window " <<   tmp  << std::endl;
            std::cout << " Dist - medium time window " <<  m_fEstimated_Dist_MediumTimeWindow << std::endl;
            std::cout << " Dist - long time window " <<    m_fEstimated_Dist_LongTimeWindow << std::endl;
    }*/



    // Motors
    //5th: distance travelled by bot in past 5s and 10s. Higher than 5% of max-possible distance travelled is accepted as feature=1.
    Real disp_MediumWindow_Threshold = 0.20f;
    Real disp_LongWindow_Threshold   = 0.20f;

    // MaxLinearSpeed in cm / control cycle
    f5 = (m_fEstimated_Dist_MediumTimeWindow >  (disp_MediumWindow_Threshold * ((Real)m_iMediumTimeWindowLength) * m_sRobotData.MaxLinearSpeed)) ? 1.0f: 0.0f;
    f6 = (m_fEstimated_Dist_LongTimeWindow   >  (disp_LongWindow_Threshold   * ((Real)m_iLongTimeWindowLength)   * m_sRobotData.MaxLinearSpeed)) ? 1.0f: 0.0f;



    m_pfFeatureValues[0] = f1;
    m_pfFeatureValues[1] = f2;
    m_pfFeatureValues[2] = m_pfAllFeatureValues[2];
    m_pfFeatureValues[3] = m_pfAllFeatureValues[3];
    m_pfFeatureValues[4] = f5;
    m_pfFeatureValues[5] = f6;

    assert(CProprioceptiveFeatureVector::NUMBER_OF_FEATURES == 6);
}

/******************************************************************************/
/******************************************************************************/

unsigned CProprioceptiveFeatureVector::CountNeighbors(Real sensor_range)
{
    unsigned count_nbrs = 0;
    for(size_t i = 0; i <  m_sSensoryData.m_RABSensorData.size(); ++i)
    {
        if(m_sSensoryData.m_RABSensorData[i].Range <= sensor_range)
            count_nbrs++;
    }

    return count_nbrs;

}

/******************************************************************************/
/******************************************************************************/

Real CProprioceptiveFeatureVector::TrackNeighborsInQueue(Real step, unsigned current_num_nbrs, unsigned num_nbrs_threshold,
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

Real CProprioceptiveFeatureVector::TrackRobotDisplacement(Real step, std::vector<RobotRelativePosData>& displacement_vector)
{
    Real displacement = 0.0f;

    CRadians delta_orientation = CRadians(m_sRobotData.seconds_per_iterations * ((-m_sSensoryData.f_LeftWheelSpeed_prev + m_sSensoryData.f_RightWheelSpeed_prev)
                                                                                 / (m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

    /*
     * Computing average velocity
     */
    if(step < (Real)displacement_vector.size())
    {
        //vec_RobotRelativePosition[(unsigned)(m_sSensoryData.m_rTime - m_fTimeFirstObserved)]

        for (size_t t = 0 ; t < (unsigned)(step); ++t)
        {
            displacement_vector[t].TimeSinceStart++;

            CRadians prev_orientation = displacement_vector[t].NetRotationSinceStart;

            Real rX = m_sRobotData.seconds_per_iterations *
                    ((m_sSensoryData.f_LeftWheelSpeed_prev + m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = m_sRobotData.seconds_per_iterations *
                    ((m_sSensoryData.f_LeftWheelSpeed_prev + m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
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

            Real rX = m_sRobotData.seconds_per_iterations *
                    ((m_sSensoryData.f_LeftWheelSpeed_prev + m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Cos(prev_orientation + delta_orientation / (2.0f));
            Real rY = m_sRobotData.seconds_per_iterations *
                    ((m_sSensoryData.f_LeftWheelSpeed_prev + m_sSensoryData.f_RightWheelSpeed_prev) / 2.0f) * Sin(prev_orientation + delta_orientation / (2.0f));

            displacement_vector[t].NetTranslationSinceStart += CVector2(rX, rY);
            displacement_vector[t].NetRotationSinceStart    += delta_orientation;
        }

        size_t t = ((unsigned)(step)%displacement_vector.size());

        /*
         * Computing average displacement
         */
        displacement = (displacement_vector[t].NetTranslationSinceStart).Length();

        displacement_vector[t].TimeSinceStart = 0.0f;
        displacement_vector[t].NetTranslationSinceStart.Set(0.0f, 0.0f);
        displacement_vector[t].NetRotationSinceStart.SetValue(0.0f);
    }

    return displacement;
}

/******************************************************************************/
/******************************************************************************/

void CProprioceptiveFeatureVector::PrintFeatureDetails()
{
    int CurrentStepNumber = (int) m_sSensoryData.m_rTime;

    std::cout << "Step: " << CurrentStepNumber << " TimeSteps_NbrsInRange0to3:  " << m_unSumTimeStepsNbrsRange0to30 <<
                 " TimeSteps_NbrsInRange3to6: " << m_unSumTimeStepsNbrsRange30to60 << " SquaredDistTravelled:  " << m_fSquaredDistTravelled <<
                 " SquaredDistThreshold: " << m_fSquaredDistThreshold << " Linear speed: " << m_sSensoryData.LinearSpeed << " Angular speed: " << m_sSensoryData.AngularSpeed <<
                 " Linear acceleration: " << m_sSensoryData.LinearAcceleration << " Angular acceleration: " << m_sSensoryData.AngularAcceleration <<
                 std::endl;


    //        std::cout << "Step: " << CurrentStepNumber << " DistTravelled:  " << sqrt(m_fSquaredDistTravelled) <<
    //                     " X " << m_sSensoryData.pos.GetX() << " Y " << m_sSensoryData.pos.GetY() <<
    //                     " X-ref " << pos_ref.GetX() << " Y-ref " << pos_ref.GetY() <<
    //                     " Linear speed: " << m_sSensoryData.LinearSpeed << " Angular speed: " << m_sSensoryData.AngularSpeed <<
    //                     std::endl;
}

/******************************************************************************/
/******************************************************************************/

//std::string CProprioceptiveFeatureVector::ToString()
//{
//    char pchTemp[4096];

//    if(NUMBER_OF_FEATURES == 6U)
//        sprintf(pchTemp, "Values - "
//                "TS_nbrs:0to3: %f - "
//                "TS_nbrs:3to6: %f - "
//                "TW450_dist0to6_angacc: %1.1f - "
//                "TW450_dist6_angacc: %1.1f - "
//                "DistTW100: %1.1f - "
//                "speed: %1.1f - fv: %u",

//                m_pfFeatureValues[0],
//                m_pfFeatureValues[1],
//                m_pfFeatureValues[2],
//                m_pfFeatureValues[3],
//                m_pfFeatureValues[4],
//                m_pfFeatureValues[5],
//                m_unValue);



//    return string(pchTemp);
//}

/******************************************************************************/
/******************************************************************************/
