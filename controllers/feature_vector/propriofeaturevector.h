#ifndef PROPRIOCEPTIVE_FEATUREVECTOR_H_
#define PROPRIOCEPTIVE_FEATUREVECTOR_H_

/******************************************************************************/
/******************************************************************************/

#include <string>

/******************************************************************************/
/******************************************************************************/
#include <argos3/core/utility/math/vector2.h>

#include <argos3/plugins/robots/generic/control_interface/ci_proximity_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_light_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_ground_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CProprioceptiveFeatureVector
{
public:

    struct RobotData
    {
        Real     MaxLinearSpeed;
        Real     MaxAngularSpeed;

        Real     MaxLinearAcceleration;
        Real     MaxAngularAcceleration;

        Real     iterations_per_second, seconds_per_iterations;
        Real     INTERWHEEL_DISTANCE, HALF_INTERWHEEL_DISTANCE;
        Real     WHEEL_RADIUS;
    };

    struct SensoryData
    {
       unsigned m_unRobotId;

       Real m_rTime;
       Real f_LeftWheelSpeed, f_RightWheelSpeed;
       Real f_LeftWheelSpeed_prev, f_RightWheelSpeed_prev;
       std::vector<Real> m_ProximitySensorData;
       std::vector<Real> m_LightSensorData;
       std::vector<Real> m_GroundSensorData;
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       Real LinearSpeed, AngularSpeed, LinearAcceleration, AngularAcceleration;
       CVector2 pos; CRadians orientation;
       Real dist;

       std::vector<Real> vec_linearspeeds; unsigned vec_linearspeeds_index;

       SensoryData()
       {
           m_rTime = 0.0f;
           f_LeftWheelSpeed = 0.0f; f_RightWheelSpeed = 0.0f;  f_LeftWheelSpeed_prev = 0.0f; f_RightWheelSpeed_prev = 0.0f;
           LinearSpeed = 0.0f; AngularSpeed = 0.0f; LinearAcceleration = 0.0f; AngularAcceleration = 0.0f;
           pos  = CVector2(0.0, 0.0);
           dist = Real(0.0f);
           orientation.SetValue(0.0f);

           vec_linearspeeds.assign(15u, 0.0f); vec_linearspeeds_index = 0;
       }

       void SetSensoryData(unsigned RobId, Real time, std::vector<Real> proximity, std::vector<Real> light, std::vector<Real> ground,
                           CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_unRobotId = RobId;
           m_rTime = time;
           m_ProximitySensorData = proximity;
           m_LightSensorData = light;
           m_GroundSensorData = ground;
           m_RABSensorData = rab;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;

           EstimateCurrentSpeedAndAcceleration();
           //EstimateCurrentPosition();
       }

       void SetSensoryData(unsigned RobId, Real time, CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_unRobotId = RobId;
           m_rTime = time;
           m_RABSensorData = rab;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;

           EstimateCurrentSpeedAndAcceleration();
           //EstimateCurrentPosition();
       }

       void EstimateCurrentSpeedAndAcceleration()
       {
           Real prev_LinearSpeed = LinearSpeed;
           LinearSpeed = ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f) * m_sRobotData.seconds_per_iterations; // speed in cm per control-cycle
           LinearAcceleration = LinearSpeed - prev_LinearSpeed;

           Real prev_AngularSpeed = AngularSpeed;
           AngularSpeed = ((-f_LeftWheelSpeed + f_RightWheelSpeed) / (m_sRobotData.INTERWHEEL_DISTANCE*100.0f)) *  m_sRobotData.seconds_per_iterations; // angular speed in rad per control-cycle
           AngularAcceleration = AngularSpeed - prev_AngularSpeed;
       }

       void EstimateCurrentPosition()
       {
           CVector2 prev_pos         = pos;
           CRadians prev_orientation = orientation;

           CRadians delta_orientation = CRadians(m_sRobotData.seconds_per_iterations * ((-f_LeftWheelSpeed + f_RightWheelSpeed) / (m_sRobotData.INTERWHEEL_DISTANCE*100.0f)));

           orientation = prev_orientation + delta_orientation;

           Real rX = prev_pos.GetX() + m_sRobotData.seconds_per_iterations * ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f) * Cos(prev_orientation + delta_orientation/(2.0f));
           Real rY = prev_pos.GetY() + m_sRobotData.seconds_per_iterations * ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f) * Sin(prev_orientation + delta_orientation/(2.0f));
           pos.Set(rX, rY);

           dist = m_sRobotData.seconds_per_iterations * ((f_LeftWheelSpeed + f_RightWheelSpeed) / 2.0f);
       }

       Real GetNormalisedAngularAcceleration()
       {
           /*normalises to range [-1 to 1] */
           if ((AngularAcceleration / m_sRobotData.MaxAngularAcceleration) >= 0.0f)
               return std::min(AngularAcceleration / m_sRobotData.MaxAngularAcceleration,   1.0);
           else
                return std::max(AngularAcceleration / m_sRobotData.MaxAngularAcceleration, -1.0);
       }
    };

    struct RobotRelativePosData
    {
        CVector2 NetTranslationSinceStart;
        CRadians NetRotationSinceStart;

        Real     TimeSinceStart;
    };


    CProprioceptiveFeatureVector();
    virtual ~CProprioceptiveFeatureVector();

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int MAX_NUMBER_OF_FEATURES;
    static unsigned int NUMBER_OF_FEATURE_VECTORS;
    static double       FEATURE_RANGE;

    virtual unsigned GetValue() const;
    virtual unsigned int GetLength() const;

    void PrintFeatureDetails();

    virtual unsigned int SimulationStep();

    //virtual std::string ToString();

    static RobotData m_sRobotData;
    SensoryData m_sSensoryData;

protected:
    virtual void ComputeFeatureValues_onlynbrsandactuators();
    virtual void ComputeFeatureValues();
    virtual void ComputeFeatureValues_NoAngularVelocityUsed();

    virtual unsigned CountNeighbors(Real sensor_range);
    virtual Real TrackNeighborsInQueue(Real step, unsigned current_num_nbrs, unsigned num_nbrs_threshold,
                               unsigned queue_length, Real queue_length_threshold,
                               unsigned int& sum_nbrs, unsigned int& queue_index, unsigned int* queue_nbrs);
    virtual Real TrackRobotDisplacement(Real step, std::vector<RobotRelativePosData>& displacement_vector);

    unsigned  m_unValue;
    unsigned  m_unLength;

    Real*         m_pfFeatureValues;
    Real*         m_pfAllFeatureValues;

    int*           m_piLastOccuranceEvent;
    int*           m_piLastOccuranceNegEvent;

    int          m_iEventSelectionTimeWindow;


    Real       m_fVelocityThreshold;
    Real       m_fAccelerationThreshold;

    Real       m_tAngularVelocityThreshold;
    Real       m_tAngularAccelerationThreshold;



    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    unsigned int m_unNbrsCurrQueueIndex;

    unsigned int m_unSumTimeStepsNbrsRange0to30;
    unsigned int m_unSumTimeStepsNbrsRange30to60;

    unsigned int* m_punNbrsRange0to30AtTimeStep;
    unsigned int* m_punNbrsRange30to60AtTimeStep;



    // keeping track of distance travelled by bot in last 100 time-steps
    int              m_iDistTravelledTimeWindow;

    unsigned int     m_unCoordCurrQueueIndex;

    Real           m_fSquaredDistTravelled;
    Real           m_fSquaredDistThreshold;
    Real           m_fCumulativeDistTravelled, m_fCumulativeDistThreshold;

    argos::CVector2  *m_pvecCoordAtTimeStep;

    Real             *m_pfDistAtTimeStep;

    /************************************************************************************/
    /* Keeping track of neighbours at different time scales*/
    unsigned int  m_unSumTimeStepsNbrs_ShortRangeTimeWindow, m_unSumTimeStepsNbrs_MediumRangeTimeWindow, m_unSumTimeStepsNbrs_LongRangeTimeWindow;
    unsigned int  *m_punNbrs_ShortRangeTimeWindow, *m_punNbrs_MediumRangeTimeWindow, *m_punNbrs_LongRangeTimeWindow;
    unsigned int m_unQueueIndex_ShortRangeTimeWindow, m_unQueueIndex_MediumRangeTimeWindow, m_unQueueIndex_LongRangeTimeWindow;
    Real m_fEstimated_Dist_ShortTimeWindow, m_fEstimated_Dist_MediumTimeWindow, m_fEstimated_Dist_LongTimeWindow;

    // keeping track of nbrs in time windows of different lengths
    static int        m_iShortTimeWindowLength, m_iMediumTimeWindowLength, m_iLongTimeWindowLength;

    std::vector<RobotRelativePosData> vec_RobPos_ShortRangeTimeWindow, vec_RobPos_MediumRangeTimeWindow, vec_RobPos_LongRangeTimeWindow;
    /************************************************************************************/


};

/******************************************************************************/
/******************************************************************************/


#endif
