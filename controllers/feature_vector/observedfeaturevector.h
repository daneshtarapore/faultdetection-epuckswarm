#ifndef OBSERVED_FEATUREVECTOR_H_
#define OBSERVED_FEATUREVECTOR_H_


/******************************************************************************/
/******************************************************************************/
#include <string>
#include <list>
#include <numeric>
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

template <typename T> Real diff_angle(T estimated_heading, T prev_estimated_heading);

class CObservedFeatureVector
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

        size_t   BEACON_SIGNAL_MARKER, NEST_BEACON_SIGNAL_MARKER;

        size_t   SELF_INFO_PACKET_MARKER;
        size_t   SELF_INFO_PACKET_FOOTER_MARKER;

        size_t   RELAY_FVS_PACKET_MARKER;
        size_t   RELAY_FVS_PACKET_FOOTER_MARKER;

        size_t   VOTER_PACKET_MARKER;
        size_t   VOTER_PACKET_FOOTER_MARKER;

        size_t   DATA_BYTE_BOUND_MARKER;

        size_t   OBSERVATION_MODE_TYPE;
    };

    struct SensoryData
    {
       unsigned m_unRobotId;

       Real m_rTime;
       Real f_LeftWheelSpeed, f_RightWheelSpeed;
       Real f_LeftWheelSpeed_prev, f_RightWheelSpeed_prev;
       Real f_LeftWheelSpeed_prev_prev, f_RightWheelSpeed_prev_prev;

       std::vector<Real> m_ProximitySensorData;
       std::vector<Real> m_LightSensorData;
       std::vector<Real> m_GroundSensorData;
       CCI_RangeAndBearingSensor::TReadings  m_RABSensorData;

       /*Real LinearSpeed, AngularSpeed, LinearAcceleration, AngularAcceleration;
       CVector2 pos; CRadians orientation;
       Real dist;*/

       SensoryData()
       {
           m_rTime = 0.0f;
           f_LeftWheelSpeed = 0.0f; f_RightWheelSpeed = 0.0f; f_LeftWheelSpeed_prev = 0.0f; f_RightWheelSpeed_prev = 0.0f;
           /*LinearSpeed = 0.0f; AngularSpeed = 0.0f; LinearAcceleration = 0.0f; AngularAcceleration = 0.0f;
           pos  = CVector2(0.0, 0.0);
           dist = Real(0.0f);
           orientation.SetValue(0.0f);*/
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

           f_LeftWheelSpeed_prev_prev = f_LeftWheelSpeed_prev; f_RightWheelSpeed_prev_prev = f_RightWheelSpeed_prev;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
       }

       void SetSensoryData(unsigned RobId, Real time, CCI_RangeAndBearingSensor::TReadings  rab, Real LeftWheelSpeed, Real RightWheelSpeed)
       {
           m_unRobotId = RobId;

           m_rTime = time;
           m_RABSensorData = rab;

           f_LeftWheelSpeed_prev_prev = f_LeftWheelSpeed_prev; f_RightWheelSpeed_prev_prev = f_RightWheelSpeed_prev;
           f_LeftWheelSpeed_prev = f_LeftWheelSpeed; f_RightWheelSpeed_prev = f_RightWheelSpeed;
           f_LeftWheelSpeed = LeftWheelSpeed; f_RightWheelSpeed = RightWheelSpeed;
       }
    };

    struct RobotRelativePosData
    {
        Real     Range_At_Start;
        CRadians Bearing_At_Start;

        CVector2 Pos_At_Start, NetTranslationSinceStart;
        CRadians NetRotationSinceStart;

        Real     TimeSinceStart;
    };


    struct ObservedRobots_FeatureVector
    {
        ObservedRobots_FeatureVector(CObservedFeatureVector &owner, Real TimeFirstObserved, unsigned ObservedRobotId);
        ObservedRobots_FeatureVector(const ObservedRobots_FeatureVector& ClassToCopy);
        ~ObservedRobots_FeatureVector();

        unsigned GetValue() const
        {
            return m_unValue;
        }

        void ComputeFeatureValues_Old();
        void ComputeFeatureValues();
        Real CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real &dist_nearest_nbr, Real& CoM_nbrs);
        Real TrackNeighborsInQueue(Real step, unsigned current_num_nbrs, unsigned num_nbrs_threshold,
                                   unsigned queue_length, Real queue_length_threshold,
                                   unsigned int& sum_nbrs, unsigned int& queue_index, unsigned int* queue_nbrs);
        void EstimateOdometry();
        Real TrackRobotDisplacement(Real step, Real observed_range, CRadians observed_bearing, CRadians self_delta_orientation, std::vector<RobotRelativePosData>& displacement_vector);
        bool GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing, Real &observedRobotId_1_SelfBearingORAngularAcceleration);


        /*
         * ID of the observed Robot
         */
        unsigned  m_unRobotId;

        /*
         * Time the robot was first observed
         */
        Real      m_fTimeFirstObserved;


        /*
         * Value of feature vector of observed robot
         */
        unsigned  m_unValue;


        //Real EstimatedLinearSpeed, EstimatedAngularSpeed, EstimatedLinearAcceleration, EstimatedAngularAcceleration;
        //CRadians estimated_bearing, prev_bearing;
        CVector2 estimated_pos, prev_pos;
        Real estimated_dist, estimated_heading, prev_estimated_heading, prev_prev_estimated_heading;
        Real average_speed, prev_average_speed, average_acceleration, m_fMeanOfAverageSpeeds, *m_pfAverageSpeedAtTimeStep; // in cm / tick and cm /tick^2
        Real prev_prev_average_speed, prev_prev_prev_average_speed;
        Real average_angularspeed, prev_average_angularspeed, average_angularacceleration; // in rad / tick and rad / tick^2


        private:

            CObservedFeatureVector& owner;

            Real*        m_pfFeatureValues;
            Real*        m_pfAllFeatureValues;

            int*         m_piLastOccuranceEvent;
            int*         m_piLastOccuranceNegEvent;

            unsigned int m_unSumTimeStepsNbrsRange0to30;
            unsigned int m_unSumTimeStepsNbrsRange30to60;

            unsigned int* m_punNbrsRange0to30AtTimeStep;
            unsigned int* m_punNbrsRange30to60AtTimeStep;

            // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
            unsigned int m_unNbrsCurrQueueIndex;
            unsigned int m_unCoordCurrQueueIndex;

            /************************************************************************************/
            /* Keeping track of neighbours at different time scales*/
            unsigned int  m_unSumTimeStepsNbrs_ShortRangeTimeWindow, m_unSumTimeStepsNbrs_MediumRangeTimeWindow, m_unSumTimeStepsNbrs_LongRangeTimeWindow;
            unsigned int  *m_punNbrs_ShortRangeTimeWindow, *m_punNbrs_MediumRangeTimeWindow, *m_punNbrs_LongRangeTimeWindow;
            unsigned int m_unQueueIndex_ShortRangeTimeWindow, m_unQueueIndex_MediumRangeTimeWindow, m_unQueueIndex_LongRangeTimeWindow;
            Real m_fEstimated_Dist_ShortTimeWindow, m_fEstimated_Dist_MediumTimeWindow, m_fEstimated_Dist_LongTimeWindow;

            std::vector<RobotRelativePosData> vec_RobPos_ShortRangeTimeWindow, vec_RobPos_MediumRangeTimeWindow, vec_RobPos_LongRangeTimeWindow;
            std::list<Real>                   list_Dist_ShortRangeTimeWindow,  list_Dist_MediumRangeTimeWindow,  list_Dist_LongRangeTimeWindow;
            /************************************************************************************/


            Real             m_fSquaredDistTravelled, m_fCumulativeDistTravelled, *m_pfDistAtTimeStep;
            CVector2         *m_pvecCoordAtTimeStep;

            std::vector<RobotRelativePosData> vec_RobotRelativePosition;
    };


    CObservedFeatureVector();
    virtual ~CObservedFeatureVector();

    virtual unsigned SimulationStep();

    virtual unsigned GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index);
    virtual float    GetSelfBearingFromRABPacket(unsigned observer_robot_id, CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id);
    virtual float    GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id);


    static RobotData m_sRobotData;
    SensoryData m_sSensoryData;

    typedef std::list <ObservedRobots_FeatureVector> t_ListObservedRobots;
    t_ListObservedRobots m_pcListObservedRobots;

    std::vector<unsigned> ObservedRobotIDs, ObservedRobotFVs;




protected:

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int MAX_NUMBER_OF_FEATURES;
    static unsigned int NUMBER_OF_FEATURE_VECTORS;
    static double       FEATURE_RANGE;


    // keeping track of neighbors in last m_iEventSelectionTimeWindow time-steps
    static int        m_iEventSelectionTimeWindow;

    // keeping track of nbrs in time windows of different lengths
    static int        m_iShortTimeWindowLength, m_iMediumTimeWindowLength, m_iLongTimeWindowLength;

    // keeping track of distance travelled by bot in last 100 time-steps
    static int        m_iDistTravelledTimeWindow;

    static Real       m_fCumulativeDistThreshold;
    static Real       m_fVelocityThreshold, m_fAccelerationThreshold;
    /*static Real       m_tAngularVelocityThreshold, m_tAngularAccelerationThreshold;*/

};

/******************************************************************************/
/******************************************************************************/


#endif
