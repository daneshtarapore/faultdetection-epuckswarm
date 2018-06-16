#ifndef FLOCKINGBEHAVIOR_H_
#define FLOCKINGBEHAVIOR_H_

/******************************************************************************/
/******************************************************************************/

#include "behavior.h"
#include <list>
#include <argos3/core/utility/math/vector2.h>

/******************************************************************************/
/******************************************************************************/

using namespace argos;

class CFlockingBehavior : public CBehavior
{
public:
    CFlockingBehavior(unsigned VelocityTimeWindowLength);

    virtual bool TakeControl();
    virtual void Action(Real &fLeftWheelSpeed, Real &fRightWheelSpeed);

    virtual void SimulationStep();

    virtual void PrintBehaviorIdentity();

    struct RobotRelativePosData
    {
        Real     Range_At_Start;
        CRadians Bearing_At_Start;

        CVector2 Pos_At_Start, NetTranslationSinceStart;
        CRadians NetRotationSinceStart;

        Real     TimeSinceStart;

        bool     b_DataAvailable; // true if range and bearing data is available at the start of the pre-specified time interval (1s, 5s, 10s). False otherwise.
    };

    struct ObservedNeighbours
    {
        ObservedNeighbours(CFlockingBehavior &owner, Real TimeFirstObserved, unsigned ObservedRobotId);
        ObservedNeighbours(const ObservedNeighbours &ClassToCopy);
        ~ObservedNeighbours();

        void EstimateOdometry();
        CVector2 TrackRobotDisplacement(Real step, Real observed_range, CRadians observed_bearing, CRadians self_delta_orientation,
                                        std::vector<RobotRelativePosData>& displacement_vector, bool b_DataAvailable);
        bool GetObservedRobotRangeBearing(Real& observedRobotId_1_Range, CRadians& observedRobotId_1_Bearing, Real &observedRobotId_1_SelfBearingORAngularAcceleration);

        CVector2 GetVelocity_ShortTimeWindow()
            {return m_fEstimated_Velocity_ShortTimeWindow;}
        CVector2 GetVelocity_MediumTimeWindow()
            {return m_fEstimated_Velocity_MediumTimeWindow;}
        CVector2 GetVelocity_LongTimeWindow()
            {return m_fEstimated_Velocity_LongTimeWindow;}


        unsigned u_TimeSinceLastObserved, u_TimeSinceLastObserved_DistMeasure;

        /*
         * ID of the observed Robot
         */
        unsigned  m_unRobotId;

        /*
         * Time the robot was first observed
         */
        Real      m_fTimeFirstObserved;


        Real average_angularacceleration; // in rad / tick^2*/

private:

        CFlockingBehavior& owner;


        /************************************************************************************/
        /* Keeping track of neighbours at different time scales*/
        CVector2 m_fEstimated_Velocity_ShortTimeWindow, m_fEstimated_Velocity_MediumTimeWindow, m_fEstimated_Velocity_LongTimeWindow;
        std::vector<RobotRelativePosData> vec_RobPos_ShortRangeTimeWindow, vec_RobPos_MediumRangeTimeWindow, vec_RobPos_LongRangeTimeWindow;

        /************************************************************************************/
    };


    virtual unsigned GetIdFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index);
    virtual float    GetAnguAccelerFromRABPacket(CCI_RangeAndBearingSensor::TReadings &rab_packet, size_t rab_packet_index, unsigned observerd_robot_id);


protected:
    unsigned       m_uVelocityTimeWindowLength;
    CVector2       m_cFlockingVector, m_cAggregationVector;
    Real           m_fAvgProximityNbrs;

    typedef std::list <ObservedNeighbours> t_ListObservedRobots;
    t_ListObservedRobots m_pcListObservedRobots;

};


/******************************************************************************/
/******************************************************************************/

#endif 
