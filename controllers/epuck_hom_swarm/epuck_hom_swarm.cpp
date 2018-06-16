

/* Include the controller definition */
#include "epuck_hom_swarm.h"

/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>

/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

/* Logging */
#include <argos3/core/utility/logging/argos_log.h>


/****************************************/
/****************************************/

CBehavior::SensoryData CBehavior::m_sSensoryData;
CBehavior::RobotData CBehavior::m_sRobotData;

CProprioceptiveFeatureVector::RobotData CProprioceptiveFeatureVector::m_sRobotData;

CObservedFeatureVector::RobotData CObservedFeatureVector::m_sRobotData;

CBayesianInferenceFeatureVector::RobotData CBayesianInferenceFeatureVector::m_sRobotData;

/****************************************/
/****************************************/

CEPuckHomSwarm::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_AGGREGATION),
    SBehavior_Trans(SWARM_NONE),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1"),
    time_between_robots_trans_behav(-1.0f)
{
    robot_ids_behav1.clear(); robot_ids_behav2.clear();
}


void CEPuckHomSwarm::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string errorbehav;
    std::string swarmbehav_trans, time_between_robots_trans_behav__str;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);

        GetNodeAttribute(t_node, "swarm_behavior_trans", swarmbehav_trans);
        GetNodeAttribute(t_node, "time_between_robots_trans_behav", time_between_robots_trans_behav__str);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_AGGREGATION") == 0)
        SBehavior = SWARM_AGGREGATION;
    else if (swarmbehav.compare("SWARM_DISPERSION") == 0)
        SBehavior = SWARM_DISPERSION;
    else if (swarmbehav.compare("SWARM_FLOCKING") == 0)
        SBehavior = SWARM_FLOCKING;
    else if (swarmbehav.compare("SWARM_HOMING") == 0)
        SBehavior = SWARM_HOMING;
    else if (swarmbehav.compare("SWARM_STOP") == 0)
        SBehavior = SWARM_STOP;
    else
    {
        std::cerr << "invalid swarm behavior";
        assert(-1);
    }

    if (errorbehav.compare("FAULT_NONE") == 0)
        FBehavior = FAULT_NONE;
    else if  (errorbehav.compare("FAULT_STRAIGHTLINE") == 0)
        FBehavior = FAULT_STRAIGHTLINE;
    else if  (errorbehav.compare("FAULT_RANDOMWALK") == 0)
        FBehavior = FAULT_RANDOMWALK;
    else if  (errorbehav.compare("FAULT_CIRCLE") == 0)
        FBehavior = FAULT_CIRCLE;
    else if  (errorbehav.compare("FAULT_STOP") == 0)
        FBehavior = FAULT_STOP;


    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMIN") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMIN;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETMAX") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETMAX;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETRANDOM") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETRANDOM;
    else if  (errorbehav.compare("FAULT_PROXIMITYSENSORS_SETOFFSET") == 0)
        FBehavior = FAULT_PROXIMITYSENSORS_SETOFFSET;


    else if  (errorbehav.compare("FAULT_RABSENSOR_SETOFFSET") == 0)
        FBehavior = FAULT_RABSENSOR_SETOFFSET;
    else if  (errorbehav.compare("FAULT_RABSENSOR_MISSINGRECEIVERS") == 0)
        FBehavior = FAULT_RABSENSOR_MISSINGRECEIVERS;


    else if  (errorbehav.compare("FAULT_ACTUATOR_LWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_LWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_RWHEEL_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_RWHEEL_SETZERO;
    else if  (errorbehav.compare("FAULT_ACTUATOR_BWHEELS_SETZERO") == 0)
        FBehavior = FAULT_ACTUATOR_BWHEELS_SETZERO;

    else if  (errorbehav.compare("FAULT_SOFTWARE") == 0)
        FBehavior = FAULT_SOFTWARE;

    else if  (errorbehav.compare("FAULT_POWER_FAILURE") == 0)
        FBehavior = FAULT_POWER_FAILURE;

    else
    {
        std::cerr << "invalid fault behavior";
        assert(-1);
    }


    /* Transition to behavior */
    if (swarmbehav_trans.compare("SWARM_AGGREGATION") == 0)
        SBehavior_Trans = SWARM_AGGREGATION;
    else if (swarmbehav_trans.compare("SWARM_DISPERSION") == 0)
        SBehavior_Trans = SWARM_DISPERSION;
    else if (swarmbehav_trans.compare("SWARM_FLOCKING") == 0)
        SBehavior_Trans = SWARM_FLOCKING;
    else if (swarmbehav_trans.compare("SWARM_HOMING") == 0)
        SBehavior_Trans = SWARM_HOMING;
    else if (swarmbehav_trans.compare("SWARM_STOP") == 0)
        SBehavior_Trans = SWARM_STOP;
    else if (swarmbehav_trans.compare("") == 0)
        SBehavior_Trans = SWARM_NONE;
    else
    {
        std::cerr << "invalid swarm transition behavior";
        assert(-1);
    }

    time_between_robots_trans_behav = strtold(time_between_robots_trans_behav__str.c_str(),NULL);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::SWheelTurningParams::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);

    //std::cout << " MaxSpeed " << MaxSpeed << std::endl;
}

/****************************************/
/****************************************/

CEPuckHomSwarm::CEPuckHomSwarm() :
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcRNG(CRandom::CreateRNG("argos")),
    m_pcRNG_FVs(CRandom::CreateRNG("argos")),
    b_damagedrobot(false),
    u_num_consequtivecollisions(0)
{
#ifndef DESYNC_ROB_CLOCK
    m_fRobotTimerAtStart = 0.0f;
#else
    // desync clocks by +/-5 s - Gaussian dist
    m_fRobotTimerAtStart = m_pcRNG->Gaussian(25.0f, 50.0f); // mean 50 ticsk, std dev. 10 ticks (50 ticks = 5 sec)
    if(m_fRobotTimerAtStart < 0.0f)
        m_fRobotTimerAtStart = 0.0f;
    else if(m_fRobotTimerAtStart > 100.0f)
        m_fRobotTimerAtStart = 100.0f;
    else
        m_fRobotTimerAtStart = (unsigned) m_fRobotTimerAtStart;
#endif

    m_fInternalRobotTimer = m_fRobotTimerAtStart;
    listFVsSensed.clear();
    listMapFVsToRobotIds.clear();
    listMapFVsToRobotIds_relay.clear();
    listConsensusInfoOnRobotIds.clear();
    listVoteInformationRobots.clear();

    b_CRM_Run = false;
    m_fCRM_RUN_TIMESTAMP = 0.0f;
    m_uRobotFV = 9999; // for debugging urposes
}

/****************************************/
/****************************************/

CEPuckHomSwarm::~CEPuckHomSwarm()
{
    // delete all behaviors

    // delete list of feature-vectors

    // delete crm model's data containers
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Init(TConfigurationNode& t_node)
{
    try
    {
        /*
       * Initialize sensors/actuators
       */

        m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
        m_pcWheelsEncoder = GetSensor  <CCI_DifferentialSteeringSensor  >("differential_steering");
        m_pcLEDs      = GetActuator<CCI_LEDsActuator                >("leds"                 );
        m_pcRABA      = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
        m_pcRABS      = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );
        m_pcProximity = GetSensor  <CCI_ProximitySensor        >("proximity"    );

        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck hom_swarm controller for robot \"" << GetId() << "\"", ex);


    Reset();

    m_sRobotDetails.SetKinematicDetails(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);

    CopyRobotDetails(m_sRobotDetails);


    m_pFlockingBehavior = new CFlockingBehavior(m_sRobotDetails.iterations_per_second * 1.0f); // 5.0f

    if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;

    // robotid set to 0 for now
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::CopyRobotDetails(RobotDetails& robdetails)
{
    //std::cout << " robdetails.MaxLinearSpeed " << robdetails.MaxLinearSpeed << std::endl;
    //std::cout << " robdetails.iterations_per_second " << robdetails.iterations_per_second << std::endl;

    CBehavior::m_sRobotData.MaxSpeed                    = robdetails.MaxLinearSpeed * robdetails.iterations_per_second; // max speed in cm/s to control behavior
    CBehavior::m_sRobotData.iterations_per_second       = robdetails.iterations_per_second;
    CBehavior::m_sRobotData.seconds_per_iterations      = 1.0f / robdetails.iterations_per_second;
    CBehavior::m_sRobotData.HALF_INTERWHEEL_DISTANCE    = robdetails.HALF_INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.INTERWHEEL_DISTANCE         = robdetails.INTERWHEEL_DISTANCE;
    CBehavior::m_sRobotData.WHEEL_RADIUS                = robdetails.WHEEL_RADIUS;

    CBehavior::m_sRobotData.m_cNoTurnOnAngleThreshold   = robdetails.m_cNoTurnOnAngleThreshold;
    CBehavior::m_sRobotData.m_cSoftTurnOnAngleThreshold = robdetails.m_cSoftTurnOnAngleThreshold;

    CBehavior::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBehavior::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBehavior::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBehavior::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBehavior::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBehavior::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBehavior::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBehavior::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CProprioceptiveFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CObservedFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CObservedFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CObservedFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CObservedFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CObservedFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;



    CObservedFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CObservedFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CObservedFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CObservedFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CObservedFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CObservedFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;




    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearSpeed           = robdetails.MaxLinearSpeed; //cm/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxLinearAcceleration    = robdetails.MaxLinearAcceleration; //cm/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.HALF_INTERWHEEL_DISTANCE = robdetails.HALF_INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.INTERWHEEL_DISTANCE      = robdetails.INTERWHEEL_DISTANCE; // m
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularSpeed          = robdetails.MaxAngularSpeed; // rad/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.MaxAngularAcceleration   = robdetails.MaxAngularAcceleration; // rad/controlcycle/controlcycle
    CBayesianInferenceFeatureVector::m_sRobotData.iterations_per_second    = robdetails.iterations_per_second;
    CBayesianInferenceFeatureVector::m_sRobotData.seconds_per_iterations   = robdetails.seconds_per_iterations;
    CBayesianInferenceFeatureVector::m_sRobotData.WHEEL_RADIUS             = robdetails.WHEEL_RADIUS;

    CBayesianInferenceFeatureVector::m_sRobotData.SetLengthOdometryTimeWindows();

    CBayesianInferenceFeatureVector::m_sRobotData.BEACON_SIGNAL_MARKER           = BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER      = NEST_BEACON_SIGNAL;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_MARKER        = SELF_INFO_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.SELF_INFO_PACKET_FOOTER_MARKER = SELF_INFO_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_MARKER        = RELAY_FVS_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.RELAY_FVS_PACKET_FOOTER_MARKER = RELAY_FVS_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_MARKER            = VOTER_PACKET;
    CBayesianInferenceFeatureVector::m_sRobotData.VOTER_PACKET_FOOTER_MARKER     = VOTER_PACKET_FOOTER;
    CBayesianInferenceFeatureVector::m_sRobotData.DATA_BYTE_BOUND_MARKER         = DATA_BYTE_BOUND;
    CBayesianInferenceFeatureVector::m_sRobotData.OBSERVATION_MODE_TYPE          = FV_MODE;
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::SumFVDist(t_listFVsSensed& FVsSensed)
{
    unsigned robotcount = 0;
    for (t_listFVsSensed::iterator it = FVsSensed.begin(); it != FVsSensed.end(); ++it)
        robotcount += it->fRobots;

    return robotcount;
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::ControlStep()
{
    //std::cout << RobotIdStrToInt() << " " << m_fInternalRobotTimer << std::endl;

    if(m_fInternalRobotTimer == m_fRobotTimerAtStart)
    {
        /*init function of loopfunction object (used to count number of e-pucks) is called after calling init of individual robot objects, and not before; so the u_num_epucks is used here*/
        assert(m_sExpRun.u_num_epucks <= 100u);
        for(unsigned id = 0; id < m_sExpRun.u_num_epucks; ++id)
            m_sExpRun.robot_ids_behav1.push_back(id);
    }


    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle
    m_uEPuckRABDataIndex = 0;

    m_fInternalRobotTimer+=1.0f;

    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }

    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_AGGREGATION ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_DISPERSION  ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_FLOCKING    ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_HOMING      ||
            m_sExpRun.SBehavior == ExperimentToRun::SWARM_STOP)
        RunHomogeneousSwarmExperiment();


    if(!b_damagedrobot || b_RunningGeneralFaults || m_sExpRun.FBehavior == ExperimentToRun::FAULT_NONE)
        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
                                                 GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    else
    {
        if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMIN)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMAX)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETRANDOM)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_MISSINGRECEIVERS)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_SOFTWARE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_POWER_FAILURE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    }

    /*For flocking behavior - to compute relative velocity*/
    CBehavior::m_sSensoryData.SetWheelSpeedsFromEncoders(m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

    /*The robot has to continually track the velocity of its neighbours - since this is done over a period of time. It can't wait until the flocking behavior is activated to start tracking neighbours*/
    m_pFlockingBehavior->SimulationStep();


    Real leftSpeed = 0.0, rightSpeed = 0.0;
    bool bControlTaken = false;
    for (TBehaviorVectorIterator i = m_vecBehaviors.begin(); i != m_vecBehaviors.end(); i++)
    {
        if (!bControlTaken)
        {
            bControlTaken = (*i)->TakeControl();
            if (bControlTaken)
            {
                (*i)->Action(leftSpeed, rightSpeed);
            }
        } else
            (*i)->Suppress();
    }

    /*If robot is contantly colliding against a wall, half the speed at which the wheels rotate - to make the robot movement closer to reality.
     * We use the IR sensors to detect this scenario and we can do this even when there is a sensor fault as in reality the speed would reduce
     * on its own when the robot is stuck to a wall*/

    if(m_pcProximity->GetReadings()[0] > 0.4f ||
            m_pcProximity->GetReadings()[1] > 0.4f ||
            m_pcProximity->GetReadings()[2] > 0.4f ||
            m_pcProximity->GetReadings()[3] > 0.4f ||
            m_pcProximity->GetReadings()[4] > 0.4f ||
            m_pcProximity->GetReadings()[5] > 0.4f ||
            m_pcProximity->GetReadings()[6] > 0.4f ||
            m_pcProximity->GetReadings()[7] > 0.4f)
        u_num_consequtivecollisions++;
    else
        u_num_consequtivecollisions = 0u;


    // if the robot is colliding with the wall other robot for more than 1s, we reduce its speed by half
    /* this will be harder to detect when we add noise on the IR sensors. Be wary of that. So using the noiseless variant of the IR sensors for this detection*/
    if((Real)u_num_consequtivecollisions > (m_sRobotDetails.iterations_per_second * 1.0f))
    {
        leftSpeed = leftSpeed/2.0f;
        rightSpeed = rightSpeed/2.0f;
    }



    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        leftSpeed  = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        rightSpeed = 0.0f;

    if(b_damagedrobot && m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
    {
        leftSpeed = 0.0f;
        rightSpeed = 0.0f;
    }

    m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed); // in cm/s

    //std::cout << "LS:  " << leftSpeed << " RS:  " << rightSpeed << std::endl;

    CCI_RangeAndBearingSensor::TReadings rabsensor_readings = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

    m_uRobotId = RobotIdStrToInt();
    SenseCommunicateDetect(RobotIdStrToInt(), m_pcRABA, m_uEPuckRABDataIndex, m_pcWheelsEncoder,
                           m_fInternalRobotTimer, rabsensor_readings,
                           listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed, listVoteInformationRobots, listConsensusInfoOnRobotIds,
                           m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                           b_CRM_Run, m_fCRM_RUN_TIMESTAMP, crminAgent, m_pcRNG_FVs, m_uRobotFV, m_sExpRun.swarmbehav, beaconrobots_ids);
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunGeneralFaults()
{
    m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
        CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
        m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);  // 0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE)
    {
        CCircleBehavior* pcCircleBehavior = new CCircleBehavior();
        m_vecBehaviors.push_back(pcCircleBehavior);
    }

    else //m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP
    {}
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::RunHomogeneousSwarmExperiment()
{
    m_vecBehaviors.clear();

    float start_firsttrans_sec = 500.0f;

    if(m_sExpRun.SBehavior_Trans != ExperimentToRun::SWARM_NONE)// && m_sExpRun.time_between_robots_trans_behav > 0.0f)
    {
        if(m_fInternalRobotTimer / m_sRobotDetails.iterations_per_second < start_firsttrans_sec)
        {
            m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;
        }
        else
        {
            std::list<unsigned>::iterator findIter = std::find(m_sExpRun.robot_ids_behav1.begin(), m_sExpRun.robot_ids_behav1.end(), RobotIdStrToInt());
            if ( m_sExpRun.robot_ids_behav1.end() == findIter ) // not in list1
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior_Trans;
            else
                m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;
        }
    }
    else
        m_sExpRun.SBehavior_Current = m_sExpRun.SBehavior;


    //if(RobotIdStrToInt()>0 || (RobotIdStrToInt()==0 && m_fInternalRobotTimer <= 2500.0f))
    if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_AGGREGATION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CAggregateBehavior* pcAggregateBehavior = new CAggregateBehavior(100.0f); //range threshold in cm //60.0
        m_vecBehaviors.push_back(pcAggregateBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::GREEN);
    }

    //else if((RobotIdStrToInt()==0 && m_fInternalRobotTimer > 2500.0f))
    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_DISPERSION)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);

        //m_pcLEDs->SetAllColors(CColor::RED);
    }

    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_FLOCKING)
    {
        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        m_vecBehaviors.push_back(m_pFlockingBehavior);

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_HOMING)
    {
        if(this->GetId().compare("ep0") == 0)
        {
            // ep0 is the beacon robot
            /* Sends out data 'BEACON_SIGNAL' with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
            // BEACON_SIGNAL is way above the DATA_BYTE_BOUND

            m_pcRABA->SetData(0, BEACON_SIGNAL);
            m_uEPuckRABDataIndex++;
            //m_pcLEDs->SetAllColors(CColor::YELLOW);
        }
        else
        {
            CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));    // 0.1f reflects a distance of about 4.5cm
            m_vecBehaviors.push_back(pcDisperseBehavior);

            Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
            CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
            m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

            CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f); //0.05f
            m_vecBehaviors.push_back(pcRandomWalkBehavior);
        }

        // Homing disabled as the beacon signal data will interfere with the FV data
        //exit(-1);
    }
    else if(m_sExpRun.SBehavior_Current == ExperimentToRun::SWARM_STOP)
    {
    }
}

/****************************************/
/****************************************/

void CEPuckHomSwarm::Reset()
{
    /* Set LED color */
    //m_pcLEDs->SetAllColors(CColor::WHITE);
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

unsigned CEPuckHomSwarm::RobotIdStrToInt()
{
    std::string id = GetId();
    id.erase(0, 2); // remove the first two characters

    std::string::size_type sz;   // alias of size_t
    unsigned u_id = std::stoi(id, &sz);
    return u_id;


    if(id.compare("ep0")==0)
        return 0;

    else if(id.compare("ep1")==0)
        return 1;

    else if(id.compare("ep2")==0)
        return 2;

    else if(id.compare("ep3")==0)
        return 3;

    else if(id.compare("ep4")==0)
        return 4;

    else if(id.compare("ep5")==0)
        return 5;

    else if(id.compare("ep6")==0)
        return 6;

    else if(id.compare("ep7")==0)
        return 7;

    else if(id.compare("ep8")==0)
        return 8;

    else if(id.compare("ep9")==0)
        return 9;

    else if(id.compare("ep10")==0)
        return 10;

    else if(id.compare("ep11")==0)
        return 11;

    else if(id.compare("ep12")==0)
        return 12;

    else if(id.compare("ep13")==0)
        return 13;

    else if(id.compare("ep14")==0)
        return 14;

    else if(id.compare("ep15")==0)
        return 15;

    else if(id.compare("ep16")==0)
        return 16;

    else if(id.compare("ep17")==0)
        return 17;

    else if(id.compare("ep18")==0)
        return 18;

    else if(id.compare("ep19")==0)
        return 19;

    else
        LOGERR << "We can't be here, there's a bug!" << std::endl;
}

/****************************************/
/****************************************/

/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as
 * second argument.
 * The string is then usable in the XML configuration file to refer to
 * this controller.
 * When ARGoS reads that string in the XML file, it knows which controller
 * class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CEPuckHomSwarm, "epuck_homswarm_controller")
