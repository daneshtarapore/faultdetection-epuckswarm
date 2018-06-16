#ifndef BAYESIANINFERENCE_FEATUREVECTOR_H_
#define BAYESIANINFERENCE_FEATUREVECTOR_H_


/******************************************************************************/
/******************************************************************************/
#include <string>
#include <list>
#include <numeric>
#include <algorithm>
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

class CBayesianInferenceFeatureVector
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

        void SetLengthOdometryTimeWindows()
        {
            /* Length of time windows for observing neighbours of your neighbours and the distance they travel */
            m_iShortTimeWindowLength  = (unsigned)iterations_per_second;
            m_iMediumTimeWindowLength = (unsigned)iterations_per_second * 5u;
            m_iLongTimeWindowLength   = (unsigned)iterations_per_second * 10u;
        }
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

        SensoryData()
        {
            m_rTime = 0.0f;
            f_LeftWheelSpeed = 0.0f; f_RightWheelSpeed = 0.0f; f_LeftWheelSpeed_prev = 0.0f; f_RightWheelSpeed_prev = 0.0f;
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

        bool     b_DataAvailable; // true if range and bearing data is available at the start of the pre-specified time interval (1s, 5s, 10s). False otherwise.
    };


    struct BayesInference_ObservedRobots_FeatureVector
    {
        BayesInference_ObservedRobots_FeatureVector(CBayesianInferenceFeatureVector &owner, Real TimeFirstObserved, unsigned ObservedRobotId);
        BayesInference_ObservedRobots_FeatureVector(const BayesInference_ObservedRobots_FeatureVector& ClassToCopy);
        ~BayesInference_ObservedRobots_FeatureVector();

        void ClearNbrs()
        {
            // clear the past observations of nbrs (upto 10s). Most likely because of an interruption in  observation
            list_NbrsInCloseProxm.clear(); list_NbrsInFarProxm.clear();
        }

        unsigned GetValue() const
        {
            return m_unValue;
        }

        void ComputeFeatureValues();
        Real CountNeighbors(Real lb_sensor_range, Real hb_sensor_range, Real &dist_nearest_nbr, Real &CoM_nbrs, Real* Range_ObserverToObserved=NULL);
        void EstimateOdometry();
        Real TrackRobotDisplacement(Real step, Real observed_range, CRadians observed_bearing, CRadians self_delta_orientation,
                                    std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable);
        bool GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing, Real &observedRobotId_1_SelfBearingORAngularAcceleration);


        void InitialisePriors()
        {

            /* Cleaning the slate */
            u_TimeSinceLastRefresh = 0u;


            /* Sensor motor interactions. Motor reactions in the presence of sensor input from nbrs, and in the absence of sensor input from nbrs. Associated with the belief  */
            sensmotPrior_Beta_a = 1u;   sensmotPrior_Beta_b = 19u; //1u
            nosensmotPrior_Beta_a = 1u; nosensmotPrior_Beta_b = 19u; //1u


            /* prior pertaining to actuation - i.e. distance travelled by robot in past 10s. Associated to the belief, is the robot moving large distances? */
            motPrior_Gaussian_mu = 25.0f; motPrior_Gaussian_variance = 50.0f; // robot moves 0.5 cm /tick, so in 100  ticks the max distance covered is 50cm.
            /* prior pertaining to motor reactions irrespective of sensor input. How does the robot react in a general sense */
            irrespsensmotPrior_Beta_a = 1u; irrespsensmotPrior_Beta_b = 19u; //1u

            /* prior pertaining to sensing - i.e. number of neighbours in close and far proximity. Associated to the belief, is the robot part of a small aggregate?, and is the robot part of a large aggregate?  */
            /*sensclosePrior_Gaussian_mu = 7.5f; sensclosePrior_Gaussian_variance = 15.0f; // sensed nbrs truly like in 15 cm range [0, 15].
            sensfarPrior_Gaussian_mu   = 15.0f; sensfarPrior_Gaussian_variance   = 15.0f; // sensed nbrs truly like in 15 cm range [15, 30].*/

            sensclosePrior_Gaussian_mu = 0.5f; sensclosePrior_Gaussian_variance = 1.0f; // nbr present (at least one) half the time of past 10 s
            sensfarPrior_Gaussian_mu   = 0.5f; sensfarPrior_Gaussian_variance   = 1.0f; // nbr present (at least one) half the time of past 10 s


            max_posterior_variance         = 50.0f;
            number_featureobservations_0  = 0.0f; number_featureobservations_1  = 0.0f; number_featureobservations_2  = 0.0f;
            number_featureobservations_3  = 0.0f; number_featureobservations_4  = 0.0f; number_featureobservations_5  = 0.0f, min_number_featureobservations = 0.0f;
        }

        void RefreshPriors()
        {

            /* Cleaning the slate */
            u_TimeSinceLastRefresh = 0u;


            /* Sensor motor interactions. Motor reactions in the presence of sensor input from nbrs, and in the absence of sensor input from nbrs. Associated with the belief  */
            sensmotPrior_Beta_a = 1u;   sensmotPrior_Beta_b = 19u; //1u
            nosensmotPrior_Beta_a = 1u; nosensmotPrior_Beta_b = 19u; //1u


            /* prior pertaining to actuation - i.e. distance travelled by robot in past 10s. Associated to the belief, is the robot moving large distances? */
            motPrior_Gaussian_mu = 25.0f; motPrior_Gaussian_variance = 50.0f; // robot moves 0.5 cm /tick, so in 100  ticks the max distance covered is 50cm.
            /* prior pertaining to motor reactions irrespective of sensor input. How does the robot react in a general sense */
            irrespsensmotPrior_Beta_a = 1u; irrespsensmotPrior_Beta_b = 19u; //1u


            /* prior pertaining to sensing - i.e. number of neighbours in close and far proximity. Associated to the belief, is the robot part of a small aggregate?, and is the robot part of a large aggregate?  */
            sensclosePrior_Gaussian_mu = 0.5f; sensclosePrior_Gaussian_variance = 1.0f; // nbr present (at least one) half the time of past 10 s
            sensfarPrior_Gaussian_mu   = 0.5f; sensfarPrior_Gaussian_variance   = 1.0f; // nbr present (at least one) half the time of past 10 s


            /* Only the priors associated with sensor-motor interactions need to be reset. The others use a Kalman filter, thus tracking the changes made */
            number_featureobservations_2  = 0.0f;
            number_featureobservations_3  = 0.0f; number_featureobservations_4  = 0.0f;
        }


        unsigned u_TimeSinceLastRefresh, u_TimeSinceLastObserved, u_TimeSinceLastObserved_DistMeasure;

        Real f_likelihood_variance;

        /* Sensor motor interactions. Motor reactions in the presence of sensor input from nbrs, and in the absence of sensor input from nbrs. Associated with the belief the robot is highly reactive to its neighbours. Or more precisely the fraction of times the robot reacts to its neighbours across its whole life-time - so say the robot is reacting to its neighbours about half its life-time. */
        unsigned sensmotPrior_Beta_a,   sensmotPrior_Beta_b;
        unsigned nosensmotPrior_Beta_a, nosensmotPrior_Beta_b;

        /* prior pertaining to actuation - i.e. distance travelled by robot in past 10s. Associated to the belief, is the robot moving large distances? */
        Real motPrior_Gaussian_mu, motPrior_Gaussian_variance;
        /* prior pertaining to motor reactions irrespective of sensor input. How does the robot react in a general sense */
        unsigned irrespsensmotPrior_Beta_a, irrespsensmotPrior_Beta_b;


        /* prior pertaining to sensing - i.e. number of neighbours in close and far proximity. Associated to the belief, is the robot part of a small aggregate?, and is the robot part of a large aggregate?  */
        Real sensclosePrior_Gaussian_mu, sensclosePrior_Gaussian_variance;
        Real sensfarPrior_Gaussian_mu, sensfarPrior_Gaussian_variance;


        Real max_posterior_variance, min_number_featureobservations;
        Real number_featureobservations_0, number_featureobservations_1, number_featureobservations_2;
        Real number_featureobservations_3, number_featureobservations_4, number_featureobservations_5;



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

        Real average_angularacceleration; // in rad / tick^2*/

    private:

        CBayesianInferenceFeatureVector& owner;

        Real*        m_pfFeatureValues;
        Real*        m_pfAllFeatureValues;


        /************************************************************************************/
        /* Keeping track of neighbours at different time scales*/
        Real m_fEstimated_Dist_ShortTimeWindow, m_fEstimated_Dist_MediumTimeWindow, m_fEstimated_Dist_LongTimeWindow;
        std::vector<RobotRelativePosData> vec_RobPos_ShortRangeTimeWindow, vec_RobPos_MediumRangeTimeWindow, vec_RobPos_LongRangeTimeWindow;

        std::list<Real> list_NbrsInCloseProxm, list_NbrsInFarProxm;
        std::list<Real> list_Dist_LongRangeTimeWindow;
        /************************************************************************************/
    };


    CBayesianInferenceFeatureVector();
    virtual ~CBayesianInferenceFeatureVector();

    virtual unsigned SimulationStep(std::string  &swarmbehav, std::vector<int> &beaconrobots_ids);

    virtual unsigned GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned *range = NULL);
    virtual float    GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id);


    static RobotData m_sRobotData;
    SensoryData m_sSensoryData;

    typedef std::list <BayesInference_ObservedRobots_FeatureVector> t_ListObservedRobots;
    t_ListObservedRobots m_pcListObservedRobots;

    std::vector<unsigned> ObservedRobotIDs; std::vector<Real> ObservedRobotIDs_range;
    std::vector<unsigned> ObservedRobotFVs, ObservedRobotFVs_Min_Number_Featureobservations;
    std::vector<unsigned> ObservedRobotFVs_Number_Featureobservations_SM, ObservedRobotFVs_Number_Featureobservations_nSM, ObservedRobotFVs_Number_Featureobservations_M;




protected:

    static unsigned int NUMBER_OF_FEATURES;
    static unsigned int MAX_NUMBER_OF_FEATURES;
    static double       FEATURE_RANGE;

    // keeping track of nbrs in time windows of different lengths
    static int        m_iShortTimeWindowLength, m_iMediumTimeWindowLength, m_iLongTimeWindowLength;
};

/******************************************************************************/
/******************************************************************************/


#endif
