/* Include the controller definition */
#include "epuck_foraging.h"
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

CEPuckForaging::ExperimentToRun::ExperimentToRun() :
    SBehavior(SWARM_FORAGING),
    FBehavior(FAULT_NONE),
    id_FaultyRobotInSwarm("-1") {}


void CEPuckForaging::ExperimentToRun::Init(TConfigurationNode& t_node)
{
    std::string errorbehav;

    try
    {
        GetNodeAttribute(t_node, "swarm_behavior", swarmbehav);
        GetNodeAttribute(t_node, "fault_behavior", errorbehav);
        GetNodeAttribute(t_node, "id_faulty_robot", id_FaultyRobotInSwarm);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing type of experiment to run, and fault to simulate.", ex);

    if (swarmbehav.compare("SWARM_FORAGING") == 0)
        SBehavior = SWARM_FORAGING;
    else
    {
        std::cerr << "invalid swarm behavior";
        exit(-1);
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
}

/****************************************/
/****************************************/

CEPuckForaging::SFoodData::SFoodData() :
    HasFoodItem(false),
    TotalFoodItems(0)
{
}

void CEPuckForaging::SFoodData::Reset()
{
    HasFoodItem = false;
    TotalFoodItems = 0;
}

/****************************************/
/****************************************/

void CEPuckForaging::SWheelTurningParams::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "max_speed", MaxSpeed);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller wheel turning parameters.", ex);
}

/****************************************/
/****************************************/

CEPuckForaging::SStateData::SStateData() :
    ProbRange(0.0f, 1.0f)
{}

void CEPuckForaging::SStateData::Init(TConfigurationNode& t_node)
{
    try
    {
        GetNodeAttribute(t_node, "initial_rest_to_explore_prob", InitialRestToExploreProb);
        GetNodeAttribute(t_node, "minimum_resting_time", MinimumRestingTime);
        GetNodeAttribute(t_node, "minimum_unsuccessful_explore_time", MinimumUnsuccessfulExploreTime);
        GetNodeAttribute(t_node, "minimum_search_for_place_in_nest_time", MinimumSearchForPlaceInNestTime);
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing controller state parameters.", ex);
}

void CEPuckForaging::SStateData::Reset()
{
    State = STATE_RESTING;
    InNest = true;
    OnFood = false;
    TimeExploringUnsuccessfully = 0;
    /* Initially the robot is resting, and by setting RestingTime to
      MinimumRestingTime we force the robots to make a decision at the
      experiment start. If instead we set RestingTime to zero, we would
      have to wait till RestingTime reaches MinimumRestingTime before
      something happens, which is just a waste of time. */
    TimeRested = MinimumRestingTime;
    TimeSearchingForPlaceInNest = 0;
}

/****************************************/
/****************************************/

CEPuckForaging::CEPuckForaging() :
    m_fInternalRobotTimer(0.0f),
    m_pcWheels(NULL),
    m_pcLEDs(NULL),
    m_pcRABA(NULL),
    m_pcRABS(NULL),
    m_pcProximity(NULL),
    m_pcLight(NULL),
    m_pcGround(NULL),
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


    TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
    TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;

    TIME_STATE_RESTING1 = 0u; TIME_STATE_EXPLORING1 = 0u; TIME_STATE_BEACON1 = 0u;
    TIME_STATE_RESTING_AT_FOOD1 = 0u;  TIME_STATE_RETURN_TO_NEST1 = 0u;

    TIME_STATE_RESTING2 = 0u; TIME_STATE_EXPLORING2 = 0u; TIME_STATE_BEACON2 = 0u;
    TIME_STATE_RESTING_AT_FOOD2 = 0u;  TIME_STATE_RETURN_TO_NEST2 = 0u;
}

/****************************************/
/****************************************/

void CEPuckForaging::Init(TConfigurationNode& t_node)
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
        m_pcLight     = GetSensor  <CCI_LightSensor          >("light"        );
        m_pcGround    = GetSensor  <CCI_GroundSensor                >("ground" );
        /*
       * Parse XML parameters
       */
        /* Experiment to run */
        m_sExpRun.Init(GetNode(t_node, "experiment_run"));
        /* Wheel turning */
        m_sWheelTurningParams.Init(GetNode(t_node, "wheel_turning"));
        /* Controller state */
        m_sStateData.Init(GetNode(t_node, "state"));
    }
    catch(CARGoSException& ex)
            THROW_ARGOSEXCEPTION_NESTED("Error initializing the e-puck foraging controller for robot \"" << GetId() << "\"", ex);

    /*
    * Initialize other stuff
    */
    Reset();

    m_sRobotDetails.SetKinematicDetails(m_sWheelTurningParams.MaxSpeed, m_sWheelTurningParams.MaxSpeed);

    CopyRobotDetails(m_sRobotDetails);

    if(this->GetId().compare("ep"+m_sExpRun.id_FaultyRobotInSwarm) == 0)
        b_damagedrobot = true;

    // robotid set to 0 for now
    crminAgent = new CRMinRobotAgentOptimised(RobotIdStrToInt(), CProprioceptiveFeatureVector::NUMBER_OF_FEATURES);
}


/****************************************/
/****************************************/

void CEPuckForaging::CopyRobotDetails(RobotDetails& robdetails)
{
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
    CBayesianInferenceFeatureVector::m_sRobotData.NEST_BEACON_SIGNAL_MARKER           = NEST_BEACON_SIGNAL;
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

unsigned CEPuckForaging::SumFVDist(t_listFVsSensed& FVsSensed)
{
    unsigned robotcount = 0;
    for (t_listFVsSensed::iterator it = FVsSensed.begin(); it != FVsSensed.end(); ++it)
        robotcount += it->fRobots;

    return robotcount;
}

/****************************************/
/****************************************/

void CEPuckForaging::ControlStep()
{
    m_pcRABA->ClearData(); // clear the channel at the start of each control cycle
    m_uEPuckRABDataIndex = 0;

    m_fInternalRobotTimer += 1.0f;

    if(m_fInternalRobotTimer == 10000.0f)
    {
        TIME_STATE_RESTING1         = TIME_STATE_RESTING;
        TIME_STATE_EXPLORING1       = TIME_STATE_EXPLORING;
        TIME_STATE_BEACON1          = TIME_STATE_BEACON;
        TIME_STATE_RESTING_AT_FOOD1 = TIME_STATE_RESTING_AT_FOOD;
        TIME_STATE_RETURN_TO_NEST1  = TIME_STATE_RETURN_TO_NEST;

        TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
        TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;
    }
    else if(m_fInternalRobotTimer == 20000.0f)
    {
        TIME_STATE_RESTING2         = TIME_STATE_RESTING;
        TIME_STATE_EXPLORING2       = TIME_STATE_EXPLORING;
        TIME_STATE_BEACON2          = TIME_STATE_BEACON;
        TIME_STATE_RESTING_AT_FOOD2 = TIME_STATE_RESTING_AT_FOOD;
        TIME_STATE_RETURN_TO_NEST2  = TIME_STATE_RETURN_TO_NEST;

        TIME_STATE_RESTING = 0u; TIME_STATE_EXPLORING = 0u; TIME_STATE_BEACON = 0u;
        TIME_STATE_RESTING_AT_FOOD = 0u;  TIME_STATE_RETURN_TO_NEST = 0u;
    }

    bool b_RunningGeneralFaults(false);
    if(b_damagedrobot && (m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_CIRCLE ||
                          m_sExpRun.FBehavior == ExperimentToRun::FAULT_STOP))
    {
        b_RunningGeneralFaults = true;
        RunGeneralFaults();
    }


    else if(m_sExpRun.SBehavior == ExperimentToRun::SWARM_FORAGING)
        RunForagingExperiment();


    if(!b_damagedrobot || b_RunningGeneralFaults || m_sExpRun.FBehavior == ExperimentToRun::FAULT_NONE)
        CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    else
    {
        //m_pcLEDs->SetAllColors(CColor::RED);

        if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMIN)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(), GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETMAX)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETRANDOM)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_PROXIMITYSENSORS_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_SETOFFSET)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_RABSENSOR_MISSINGRECEIVERS)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_LWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_RWHEEL_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }
        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_ACTUATOR_BWHEELS_SETZERO)
        {
            // does not affect the sensors - they stay the same
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
        }


        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_SOFTWARE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));

        else if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_POWER_FAILURE)
            CBehavior::m_sSensoryData.SetSensoryData(m_pcRNG, m_fInternalRobotTimer, GetIRSensorReadings(b_damagedrobot, m_sExpRun.FBehavior), m_pcLight->GetReadings(), m_pcGround->GetReadings(),GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior));
    }

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

    /*If robot is contantly colliding against a wall, half the speed at which the wheels rotate - to make the robot movement closer to reality. We use the IR sensors to detect this scenario and we can do this even when there is a sensor fault as in reality the speed would reduce on its own when the robot is stuck to a wall*/
    /*Using the noiseless variant of the IR sensors for this detection*/

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

void CEPuckForaging::RunGeneralFaults()
{
    //m_pcLEDs->SetAllColors(CColor::RED);

    m_vecBehaviors.clear();
    if(m_sExpRun.FBehavior == ExperimentToRun::FAULT_STRAIGHTLINE)
    {
         CRandomWalkBehavior* pcStraightLineBehavior = new CRandomWalkBehavior(0.0f);
         m_vecBehaviors.push_back(pcStraightLineBehavior);
    }
    else if (m_sExpRun.FBehavior == ExperimentToRun::FAULT_RANDOMWALK)
    {
        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
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

void CEPuckForaging::Reset()
{
    /* Reset robot state */
    m_sStateData.Reset();
    /* Reset food data */
    m_sFoodData.Reset();
    /* Set LED color */
    //m_pcLEDs->SetAllColors(CColor::RED);
    /* Clear up the last exploration result */
    m_eLastExplorationResult = LAST_EXPLORATION_NONE;
    m_pcRABA->ClearData();
}

/****************************************/
/****************************************/

void CEPuckForaging::UpdateState()
{
    /* Reset state flags */
    m_sStateData.InNest = false;
    /* Read stuff from the ground sensor */
    const std::vector<Real> tGroundReads = m_pcGround->GetReadings();
    /*
    * You can say whether you are in the nest by checking the ground sensor
    * placed close to the wheel motors. It returns a value between 0 and 1.
    * It is 1 when the robot is on a white area, it is 0 when the robot
    * is on a black area and it is around 0.5 when the robot is on a gray
    * area.
    * The e-puck has 3 sensors in a straight line in the front of the robot.
    */
    if(tGroundReads[0] > 0.25f &&
            tGroundReads[0] < 0.75f &&
            tGroundReads[1] > 0.25f &&
            tGroundReads[1] < 0.75f &&
            tGroundReads[2] > 0.25f &&
            tGroundReads[2] < 0.75f)
        m_sStateData.InNest = true;


    m_sStateData.OnFood = false;
    if(tGroundReads[0] <= 0.1f &&
            tGroundReads[1] <= 0.1f &&
            tGroundReads[2] <= 0.1f)
        m_sStateData.OnFood = true;

    //std::cout << " tGroundReads[0] " << tGroundReads[0] << " tGroundReads[1] " << tGroundReads[1] << " tGroundReads[2] " << tGroundReads[2] << std::endl << std::endl;
}

/****************************************/
/****************************************/

void CEPuckForaging::RunForagingExperiment()
{
    switch(m_sStateData.State) /* Deciding state transitions */
    {
    case SStateData::STATE_RESTING:
    {
        //std::cout << "SStateData::STATE_RESTING " << std::endl;
        RestAtNest();
        TIME_STATE_RESTING++;
        break;
    }
    case SStateData::STATE_EXPLORING:
    {
        //std::cout << "SStateData::STATE_EXPLORING " << std::endl;
        Explore(); // have we transitioned from explore?
        TIME_STATE_EXPLORING++;
        break;
    }
    case SStateData::STATE_BEACON:
    {
        //std::cout << "SStateData::STATE_BEACON " << std::endl;
        BecomeABeacon();
        TIME_STATE_BEACON++;
        break;
    }
    case SStateData::STATE_RESTING_AT_FOOD:
    {
        //std::cout << "SStateData::STATE_RESTING_AT_FOOD " << std::endl;
        RestAtFood();
        TIME_STATE_RESTING_AT_FOOD++;
        break;
    }
    case SStateData::STATE_RETURN_TO_NEST:
    {
        //std::cout << "SStateData::STATE_RETURN_TO_NEST " << std::endl;
        ReturnToNest();
        TIME_STATE_RETURN_TO_NEST++;
        break;
    }
    default:
    {
        LOGERR << "We can't be here, there's a bug!" << std::endl;
    }
    }

    //m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;

    if(m_sStateData.State == SStateData::STATE_RESTING)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_RESTING_AT_FOOD)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_BEACON)
        m_vecBehaviors.clear(); // nothing to execute

    else if(m_sStateData.State == SStateData::STATE_EXPLORING)
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
        CHomingToFoodBeaconBehavior* pcHomingToFoodBeaconBehavior = new CHomingToFoodBeaconBehavior(BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
        m_vecBehaviors.push_back(pcHomingToFoodBeaconBehavior);

        /*CAntiPhototaxisBehavior* pcAntiPhototaxisBehavior = new CAntiPhototaxisBehavior();
        m_vecBehaviors.push_back(pcAntiPhototaxisBehavior);*/

        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }

    else if(m_sStateData.State == SStateData::STATE_RETURN_TO_NEST) // Perform exploration
    {
        m_vecBehaviors.clear();

        CDisperseBehavior* pcDisperseBehavior = new CDisperseBehavior(0.1f, ToRadians(CDegrees(5.0f)));
        m_vecBehaviors.push_back(pcDisperseBehavior);

        //! Oscillation of take control between CPhototaxisBehavior and Disperse can cause robots to remain stuck at food source in RETURN_TO_NEST STATE.
        //! But this happends very rarely
        //CPhototaxisBehavior* pCPhototaxisBehavior = new CPhototaxisBehavior();
        //m_vecBehaviors.push_back(pCPhototaxisBehavior);


        /*Real MAX_BEACON_SIGNAL_RANGE = 1.0f; //1m
        CHomingToFoodBeaconBehavior* pcHomingToNestBeaconBehavior = new CHomingToFoodBeaconBehavior(NEST_BEACON_SIGNAL, MAX_BEACON_SIGNAL_RANGE);
        m_vecBehaviors.push_back(pcHomingToNestBeaconBehavior);*/


        CRandomWalkBehavior* pcRandomWalkBehavior = new CRandomWalkBehavior(0.0017f);
        m_vecBehaviors.push_back(pcRandomWalkBehavior);
    }
}


/****************************************/
/****************************************/

void CEPuckForaging::RestAtNest()
{
    /* If we have stayed here enough, probabilistically switch to
    * 'exploring' */
    if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
            m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.InitialRestToExploreProb)
    {
        //m_pcLEDs->SetAllColors(CColor::GREEN);
        m_sStateData.State = SStateData::STATE_EXPLORING;
        m_sStateData.TimeRested = 0;
    }
    else
    {
        ++m_sStateData.TimeRested;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::RestAtFood()
{
    /* If we have stayed here enough, probabilistically switch to
    * 'ReturnToNest' */
    if(m_sStateData.TimeRested > m_sStateData.MinimumRestingTime &&
            m_pcRNG->Uniform(m_sStateData.ProbRange) < m_sStateData.InitialRestToExploreProb) //InitialRestToExploreProb used here as well.
    {
        //m_pcLEDs->SetAllColors(CColor::BLUE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
        m_sStateData.TimeRested = 0;
    }
    else
    {
        ++m_sStateData.TimeRested;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::BecomeABeacon()
{
    UpdateState();

    if(m_sStateData.OnFood)
    {
        /* Send out data with RABS that you are a beacon. Neighbouring robots will use this data to home in on your position */
        m_pcRABA->SetData(0, BEACON_SIGNAL);
        m_uEPuckRABDataIndex++;
    }
    else
        m_sStateData.State = SStateData::STATE_EXPLORING;
}

/****************************************/
/****************************************/

void CEPuckForaging::Explore()
{
    /* We switch to 'become a beacon' in one situation:
    * 1. if we are on a food item and there is no beacon nearby
    */


    /* We switch to 'return to nest' in two situations:
    * 1. if we have a food item
    * 2. if we have not found a food item for some time;
    *    in this case, the switch is probabilistic
    */

    bool bBecomeBeacon(false);
    UpdateState();
    if(m_sStateData.OnFood)
    {
        bBecomeBeacon = true;

        // Check if you are to be a beacon
        const CCI_RangeAndBearingSensor::TReadings& tPackets = m_pcRABS->GetReadings();
        for(size_t i = 0; i < tPackets.size(); ++i)
        {
            /*if(tPackets[i].Data[0] == BEACON_SIGNAL)
                std::cout << "Packet index " << i << " packet range to beacon " << tPackets[i].Range << "cm" << std::endl;*/
            /* tPackets[i].Range is in cm and bearing is normalized [-pi pi] */
            if(tPackets[i].Data[0] == BEACON_SIGNAL && tPackets[i].Range < 25.0f) // Each food spot has radius of 0.2m // a slightly higher threshold is chosen to be safe
            {
                // Note: If a foraging robot blocks the IR rays from the RAB actuator of a beacon robot, more than one beacons may be formed at the foraging site
                bBecomeBeacon = false;
                break;
            }
        }
        if(bBecomeBeacon == false)
            m_sFoodData.HasFoodItem = true;
    }


    bool bReturnToNest(false);
    bool bRestAtFood(false);

    /*
    * Test the first condition: have we found a food item?
    * NOTE: the food data is updated by the loop functions, so
    * here we just need to read it
    * UPDATE: Not any more. Now it is check in the robot's control loop
    */
    if(m_sFoodData.HasFoodItem)
    {
        m_eLastExplorationResult = LAST_EXPLORATION_SUCCESSFUL;
        /* Switch to 'return to nest' */
        bRestAtFood = true;
    }
    else if(bBecomeBeacon)
    {
        //m_eLastExplorationResult = BEACON_ESTABLISHED;
    }

    /* Test the second condition: we switch to 'return to
    * nest' if we have been wandering for a lot of time and found nothing */
    else if(m_sStateData.TimeExploringUnsuccessfully > m_sStateData.MinimumUnsuccessfulExploreTime)
    {
        /* Store the result of the expedition */
        m_eLastExplorationResult = LAST_EXPLORATION_UNSUCCESSFUL;
        /* Switch to 'return to nest' */
        bReturnToNest = true;
    }


    if(bReturnToNest) /* So, do we return to the nest - ie, the last exploration has been unsuccessful */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::BLUE);
        m_sStateData.State = SStateData::STATE_RETURN_TO_NEST;
    }
    else if(bBecomeBeacon) /*Do we become a beacon */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::YELLOW);
        m_sStateData.State = SStateData::STATE_BEACON;
    }
    else if(bRestAtFood) /* So, do we rest at the food source  */
    {
        /* Yes, we do! */
        m_sStateData.TimeExploringUnsuccessfully = 0;
        m_sStateData.TimeSearchingForPlaceInNest = 0;
        //m_pcLEDs->SetAllColors(CColor::BLACK);
        m_sStateData.State = SStateData::STATE_RESTING_AT_FOOD;
    }
    else
    {
        /* No, perform the actual exploration */
        ++m_sStateData.TimeExploringUnsuccessfully;
    }
}

/****************************************/
/****************************************/

void CEPuckForaging::ReturnToNest()
{
    /* As soon as you get to the nest, switch to 'resting' */
    UpdateState();
    /* Are we in the nest? */
    if(m_sStateData.InNest)
    {
        if(m_sFoodData.HasFoodItem)
        {
            /* Drop the food item */
            m_sFoodData.HasFoodItem = false;
            ++m_sFoodData.TotalFoodItems;
        }

        /* Have we looked for a place long enough? */
        if(m_sStateData.TimeSearchingForPlaceInNest > m_sStateData.MinimumSearchForPlaceInNestTime)
        {
            // m_pcLEDs->SetAllColors(CColor::YELLOW); if the robot became a nest beacon

            /* switch to state 'resting' */
            //m_pcLEDs->SetAllColors(CColor::RED);
            m_sStateData.State = SStateData::STATE_RESTING;
            m_sStateData.TimeSearchingForPlaceInNest = 0;
            m_eLastExplorationResult = LAST_EXPLORATION_NONE;
            return;
        }
        else
        {
            /* No, keep looking */
            ++m_sStateData.TimeSearchingForPlaceInNest;
        }
    }
    else
    {
        /* Still outside the nest */
        m_sStateData.TimeSearchingForPlaceInNest = 0;
    }
    //    /* Keep going */
    //    bool bCollision;
    //    SetWheelSpeedsFromVector(
    //                m_sWheelTurningParams.MaxSpeed * DiffusionVector(bCollision) +
    //                m_sWheelTurningParams.MaxSpeed * CalculateVectorToLight());
}

/****************************************/
/****************************************/

unsigned CEPuckForaging::RobotIdStrToInt()
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
REGISTER_CONTROLLER(CEPuckForaging, "epuck_foraging_controller")
