
#include "sensingandcommunication.h"

/****************************************/
/****************************************/

unsigned m_uRABDataIndex;

/****************************************/
/****************************************/

void Sense(Real m_fProbForget,
           unsigned RobotId, CCI_RangeAndBearingActuator *pcRABA,
           Real m_fInternalRobotTimer, CCI_RangeAndBearingSensor::TReadings &rabsensor_readings,
           t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
           t_listVoteInformationRobots&  listVoteInformationRobots, t_listConsensusInfoOnRobotIds& listConsensusInfoOnRobotIds,
           CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CObservedFeatureVector &m_cObservationFeatureVector,
           CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector, CRandom::CRNG* m_pcRNG_FVs, std::string &swarmbehav, std::vector<int> &beaconrobots_ids)
{
#if FV_MODE == PROPRIOCEPT_MODE

#ifdef ConsensusOnMapOfIDtoFV
    exit(-1);
#endif

    const CCI_RangeAndBearingSensor::TReadingsdings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);

    /* Listen for feature vectors from neighbours */
    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */
    bool read_status = ReadFromCommunicationChannel_IdFv(m_fInternalRobotTimer, listMapFVsToRobotIds_relay, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/


    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS); /*remove entries older than 10s */

    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
#endif

#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

#ifdef ConsensusOnMapOfIDtoFV
    exit(-1);
#endif

    listMapFVsToRobotIds_relay.clear();
    for (size_t i = 0; i < m_cObservationFeatureVector.ObservedRobotIDs.size(); ++i)
    {
        unsigned robotId = m_cObservationFeatureVector.ObservedRobotIDs[i];
        unsigned fv      = m_cObservationFeatureVector.ObservedRobotFVs[i];

        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));

        //if(robotId == 15)
        //  std::cerr << "Observer: " << m_uRobotId << " ObservedId " << robotId << " ObservedFV " << fv << std::endl;

        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
    }

    const CCI_RangeAndBearingSensor::TReadingsdings& tmp = GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
    bool read_status = ReadFromCommunicationChannel_RelayedFv(m_fInternalRobotTimer, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

    //remove entries older than 10s
    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CBehavior::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);
    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed
#endif

#if FV_MODE == BAYESIANINFERENCE_MODE
    listMapFVsToRobotIds_relay.clear();
    for (size_t i = 0; i < m_cBayesianInferredFeatureVector.ObservedRobotIDs.size(); ++i)
    {
        /* Ignore the homing beacon in fault detection */
        if(swarmbehav.compare("SWARM_HOMING") == 0)
            if(m_cBayesianInferredFeatureVector.ObservedRobotIDs[i] == 0u)
            {
                continue;
            }

        /* Ignore the foraging beacon(s) in fault detection */
        if(swarmbehav.compare("SWARM_FORAGING") == 0)
            if(std::find(beaconrobots_ids.begin(), beaconrobots_ids.end(), m_cBayesianInferredFeatureVector.ObservedRobotIDs[i]) != beaconrobots_ids.end())
            {
                continue;
            }



        unsigned robotId = m_cBayesianInferredFeatureVector.ObservedRobotIDs[i];
        unsigned fv      = m_cBayesianInferredFeatureVector.ObservedRobotFVs[i];


        unsigned num_obs_sm  = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_SM[i];
        unsigned num_obs_nsm = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_nSM[i];
        unsigned num_obs_m   = m_cBayesianInferredFeatureVector.ObservedRobotFVs_Number_Featureobservations_M[i];

        listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv,
                                                                          m_cBayesianInferredFeatureVector.ObservedRobotIDs_range[i],
                                                                          num_obs_sm, num_obs_nsm, num_obs_m));

#ifndef ConsensusOnMapOfIDtoFV
        UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer,
                             m_cBayesianInferredFeatureVector.ObservedRobotIDs_range[i],
                             num_obs_sm, num_obs_nsm, num_obs_m);
#else
        UpdateFvToRobotIdMap(listMapFVsToRobotIds, RobotId, fv, robotId, m_fInternalRobotTimer,
                             m_cBayesianInferredFeatureVector.ObservedRobotIDs_range[i],
                             num_obs_sm, num_obs_nsm, num_obs_m);
#endif
    }

    const CCI_RangeAndBearingSensor::TReadings& tmp = rabsensor_readings; //GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior);
    bool read_status = ReadFromCommunicationChannel_RelayedFv(m_fInternalRobotTimer, listMapFVsToRobotIds, tmp); /* returns true if successfully read id and fvs from at least one neighbour*/

    //remove entries older than 10s
    TrimFvToRobotIdMap(listMapFVsToRobotIds, m_fInternalRobotTimer, CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second * CRM_RESULTS_VALIDFOR_SECONDS);

#ifdef ConsensusOnMapOfIDtoFV
    SelectBestFVFromAllObservedFVs(listMapFVsToRobotIds, CProprioceptiveFeatureVector::NUMBER_OF_FEATURES, m_pcRNG_FVs, RobotId);
#endif

    UpdaterFvDistribution(listFVsSensed, listMapFVsToRobotIds, m_pcRNG_FVs, m_fProbForget); // update listFVsSensed

#endif
}

/****************************************/
/****************************************/

void SenseCommunicateDetect(unsigned RobotId, CCI_RangeAndBearingActuator *m_pcRABA, unsigned& DataIndex,  CCI_DifferentialSteeringSensor* m_pcWheelsEncoder,
         Real m_fInternalRobotTimer, CCI_RangeAndBearingSensor::TReadings &rabsensor_readings,
         t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
         t_listVoteInformationRobots&  listVoteInformationRobots, t_listConsensusInfoOnRobotIds& listConsensusInfoOnRobotIds,
         CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
         CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
         bool b_CRM_Run, Real m_fCRM_RUN_TIMESTAMP, CRMinRobotAgentOptimised* crminAgent, CRandom::CRNG* m_pcRNG_FVs, unsigned& m_uRobotFV,
         std::string  &swarmbehav, std::vector<int> &beaconrobots_ids)
{
    unsigned m_uRobotId;

    m_uRABDataIndex = DataIndex;


    /****************************************/
#if FV_MODE == PROPRIOCEPT_MODE

    /* Estimate feature-vectors - proprioceptively */

    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
    m_cProprioceptiveFeatureVector.SimulationStep();

    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue(); // to debug
    m_uRobotId = RobotId; //RobotIdStrToInt();

    /* Communicate your id and proprioceptively computed FV to whoever is in range, using the RAB sensor*/
    /* Also relay the id and fvs of neighbours, received by you in the previous control cycle */
    //if ((unsigned)m_fInternalRobotTimer%2u == 0)

    //printf(" SendFVsToNeighbours() \n\n\n");
    //SendFVsToNeighbours();
    SendFVsToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                        RobotId, listMapFVsToRobotIds_relay);

    /* Listen for robot ids + feature vectors from neighbours and then assimilate them  */
    //printf(" Sense(PROBABILITY_FORGET_FV); \n\n\n");
    Sense(PROBABILITY_FORGET_FV, RobotId, m_pcRABA,
          m_fInternalRobotTimer, rabsensor_readings,
          listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
          listVoteInformationRobots, listConsensusInfoOnRobotIds,
          m_cProprioceptiveFeatureVector, m_cObservationFeatureVector,
          m_cBayesianInferredFeatureVector, m_pcRNG_FVs, swarmbehav, beaconrobots_ids);

#endif
    /****************************************/



    /****************************************/
#if FV_MODE == OBSERVATION_MODE || FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE

    /* Estimating FVs proprioceptively - to be used for the simplifying fault detection and to compute angular acceleration for the COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE */
    /*m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
                                                                 leftSpeed, rightSpeed);*/
    /*encoders give you the speed at the previous tick not current tick */
    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);


    m_cProprioceptiveFeatureVector.SimulationStep();
    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

    /* Estimate feature-vectors - via observation */
    m_uRobotId = RobotId; //RobotIdStrToInt();


    /*m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotIdStrToInt(), m_fInternalRobotTimer, GetRABSensorReadings(b_damagedrobot, m_sExpRun.FBehavior),
                                                              leftSpeed, rightSpeed);*/
    /*encoders give you the speed at the previous tick not current tick */
    m_cObservationFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                              m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);
    m_cObservationFeatureVector.SimulationStep();

    Sense(PROBABILITY_FORGET_FV, RobotId, m_pcRABA,
          m_fInternalRobotTimer, rabsensor_readings,
          listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
          listVoteInformationRobots, listConsensusInfoOnRobotIds,
          m_cProprioceptiveFeatureVector, m_cObservationFeatureVector,
          m_cBayesianInferredFeatureVector, m_pcRNG_FVs, swarmbehav, beaconrobots_ids);


    //if ( ((unsigned)m_fInternalRobotTimer%2u == 0) || (m_fInternalRobotTimer <= MODELSTARTTIME))
    /*
     * Send the robot id and the bearing at which it observes its different neighbours. Also relay the observed FVs
     */

    SendIdSelfBearingAndObsFVsToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector, RobotId,
                                           rabsensor_readings, listMapFVsToRobotIds_relay);
#endif
    /****************************************/

    /****************************************/
#if FV_MODE == BAYESIANINFERENCE_MODE

    /* Estimating FVs proprioceptively - to be used for computing angular acceleration*/
    /*encoders give you the speed at the previous tick not current tick */
    m_cProprioceptiveFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                 m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

    m_cProprioceptiveFeatureVector.SimulationStep();
    m_uRobotFV = m_cProprioceptiveFeatureVector.GetValue();

    /* Estimate feature-vectors - via observation */
    m_uRobotId = RobotId; //RobotIdStrToInt();


    /*encoders give you the speed at the previous tick not current tick */
    m_cBayesianInferredFeatureVector.m_sSensoryData.SetSensoryData(RobotId, m_fInternalRobotTimer, rabsensor_readings,
                                                                   m_pcWheelsEncoder->GetReading().VelocityLeftWheel, m_pcWheelsEncoder->GetReading().VelocityRightWheel);

     m_cBayesianInferredFeatureVector.SimulationStep(swarmbehav, beaconrobots_ids);


    Sense(PROBABILITY_FORGET_FV, RobotId, m_pcRABA,
          m_fInternalRobotTimer, rabsensor_readings,
          listMapFVsToRobotIds, listMapFVsToRobotIds_relay, listFVsSensed,
          listVoteInformationRobots, listConsensusInfoOnRobotIds,
          m_cProprioceptiveFeatureVector, m_cObservationFeatureVector,
          m_cBayesianInferredFeatureVector, m_pcRNG_FVs, swarmbehav, beaconrobots_ids);

    SendIdSelfBearingAndObsFVsToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector, RobotId,
                                           rabsensor_readings, listMapFVsToRobotIds_relay);
#endif
    /****************************************/


    /****************************************/

    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) == 0u)
        // to avoid consensus already in the medium to establish itself in the next step. when the robot clocks are not in sync, this period would have to be longer than just 2 iterations
    {
        listConsensusInfoOnRobotIds.clear();
        listVoteInformationRobots.clear();
    }
    else if(m_fInternalRobotTimer > MODELSTARTTIME)
        /* else because you don't want to receive consensus already in the medium from before the buffer was cleared*/
    {
        Real m_fVoteCompilationStartTime = (Real)(((unsigned)m_fInternalRobotTimer)/
                                             ((unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second))) *
                                             (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second);

#ifndef DISABLE_SWARMCOALITION
#ifndef FILTER_BEFORE_VOTE
        /* Listen for voting packets and consensus packets from neighbours*/
        ReceiveVotesAndConsensus(listVoteInformationRobots, listMapFVsToRobotIds, listConsensusInfoOnRobotIds, rabsensor_readings);
        EstablishConsensus(m_fInternalRobotTimer, listVoteInformationRobots, listConsensusInfoOnRobotIds);
#else

        if(((m_fInternalRobotTimer - m_fVoteCompilationStartTime)/ (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >
           (1.0f - 1.0f/VOTCON_RESULTS_VALIDFOR_SECONDS)) //0.9f)
        {
            /* Listen for voting packets and consensus packets from neighbours*/
            ReceiveVotesAndConsensus(listVoteInformationRobots, listMapFVsToRobotIds, listConsensusInfoOnRobotIds, rabsensor_readings);
            EstablishConsensus(m_fInternalRobotTimer, listVoteInformationRobots, listConsensusInfoOnRobotIds);
        }
#endif
#endif
    }


    Real TimeSinceCRM = (m_fInternalRobotTimer - m_fCRM_RUN_TIMESTAMP) * CProprioceptiveFeatureVector::m_sRobotData.seconds_per_iterations; // in seconds
    if (b_CRM_Run && (TimeSinceCRM > CRM_RESULTS_VALIDFOR_SECONDS)) /* the results of the CRM are no longer valid */
        b_CRM_Run = false;

    if((m_fInternalRobotTimer > MODELSTARTTIME) && (listFVsSensed.size() > 0))    // the robot has atleast had one FV entry in its distribution. if not the CRM will crash.
    {
        crminAgent->SimulationStepUpdatePosition(m_fInternalRobotTimer, &listFVsSensed);
        b_CRM_Run = true;
        m_fCRM_RUN_TIMESTAMP = m_fInternalRobotTimer;
    }

    if(b_CRM_Run) // a failsafe to make sure you don't use outdated CRM results
    {
        // the CRM results on FVs in listFVsSensed is not outdated
        for(t_listFVsSensed::iterator it_fv = listFVsSensed.begin(); it_fv != listFVsSensed.end(); ++it_fv)
        {
#ifndef FILTER_BEFORE_VOTE
            UpdateVoterRegistry(listVoteInformationRobots,
                                listMapFVsToRobotIds,
                                listConsensusInfoOnRobotIds,
                                RobotId, it_fv->uFV, it_fv->uMostWantedState);
#else
            Real m_fVoteCompilationStartTime = (Real)(((unsigned)m_fInternalRobotTimer)/
                                                 ((unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second))) *
                                          (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second);


            if(((m_fInternalRobotTimer - m_fVoteCompilationStartTime)/ (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >
                    (1.0f - 1.0f/VOTCON_RESULTS_VALIDFOR_SECONDS))//0.9f)
            {
                UpdateVoterRegistry(listVoteInformationRobots,
                                    listMapFVsToRobotIds,
                                    listConsensusInfoOnRobotIds,
                                    RobotId, it_fv->uFV, it_fv->uMostWantedState, true);
            }
            else
            {
                IntegrateAttackTolerateDecisions(listMapFVsToRobotIds, it_fv->uFV, it_fv->uMostWantedState);
            }

#endif
        }
    }

    if(((unsigned)m_fInternalRobotTimer % (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) != 0u) /* dont send CRM results if buffer is cleared*/
    {
        if ((m_fInternalRobotTimer > MODELSTARTTIME)) // && (unsigned)m_fInternalRobotTimer%2u == 1)
        {
#ifndef FILTER_BEFORE_VOTE
            SendCRMResultsAndConsensusToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                                                   RobotId, listMapFVsToRobotIds, listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Run); // only send CRM results if they are valid
#else
            Real m_fVoteCompilationStartTime = (Real)(((unsigned)m_fInternalRobotTimer)/
                                                      ((unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second))) *
                                               (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second);


            if(((m_fInternalRobotTimer - m_fVoteCompilationStartTime)/ (VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >
                    (1.0f - 1.0f/VOTCON_RESULTS_VALIDFOR_SECONDS)) //0.9f)
            {
                for (t_listFVsSensed::iterator it_fvdist = listFVsSensed.begin(); it_fvdist != listFVsSensed.end(); ++it_fvdist)
                {
                    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
                    {
                        if(it_map->uFV == it_fvdist->uFV)
                        {
                                if (it_map->f_TimesAttacked / (it_map->f_TimesAttacked + it_map->f_TimesTolerated) > 0.5f)
                                {
                                    it_fvdist->uMostWantedState = 1;
                                }
                                else
                                {
                                    it_fvdist->uMostWantedState = 2;
                                }
                                break;
                        }
                    }
                }
                SendCRMResultsAndConsensusToNeighbours(m_pcRABA, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                                                       RobotId, listMapFVsToRobotIds,
                                                       listFVsSensed, listConsensusInfoOnRobotIds, b_CRM_Run); // only send CRM results if they are valid
            }
#endif
        }
    }
}

/****************************************/
/****************************************/

void SendIdSelfBearingAndObsFVsToNeighbours(CCI_RangeAndBearingActuator *m_pcRABA, CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                            CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                            unsigned RobotId, const CCI_RangeAndBearingSensor::TReadings &tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
{
    /*Communicate your id to neighbours, so they know who they are observing*/
    /*Also communicate the bearing at which you observed the neighbours */
    /*Also communicate the FVs you have observed */

    WriteToCommunicationChannel(m_pcRABA,  m_uRABDataIndex, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                                RobotId, tPackets, IdToFVsMap_torelay);
}

/****************************************/
/****************************************/

void SendFVsToNeighbours(CCI_RangeAndBearingActuator* m_pcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                         CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                         unsigned RobotId, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay)
{
    /*Communicate your id and FV, and relay the id and fvs of neighbours, received by you in the previous control cycle*/
    WriteToCommunicationChannel(m_pcRABA, m_uRABDataIndex, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                                RobotId, m_cProprioceptiveFeatureVector.GetValue(), listMapFVsToRobotIds_relay);
}

/****************************************/
/****************************************/

void SendCRMResultsAndConsensusToNeighbours(CCI_RangeAndBearingActuator *m_pcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                            CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                            unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist,
                                            t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid)
{
    /* Commmunicate your CRM results to your neighbours
     * Also broadcast consensus information - add any new consensus info to your local list and send it out again */

    WriteToCommunicationChannel(m_pcRABA, m_uRABDataIndex, m_cProprioceptiveFeatureVector, m_cObservationFeatureVector, m_cBayesianInferredFeatureVector,
                                VoterId, MapFVsToRobotIds,
                                CRMResultsOnFVDist, ConsensusLst, b_CRM_Results_Valid);
}

/****************************************/
/****************************************/

void WriteToCommunicationChannel(CCI_RangeAndBearingActuator *m_pcRABA, unsigned& databyte_index,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings &tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay)
{
    // size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
    m_pcRABA->SetData(databyte_index++, SelfId);

#if FV_MODE == COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE || FV_MODE == BAYESIANINFERENCE_MODE

    // angular acceleration [-1, +1] to [0, DATA_BYTE_BOUND]
    unsigned un_angularacceleration = (unsigned)(((m_cProprioceptiveFeatureVector.m_sSensoryData.GetNormalisedAngularAcceleration() + 1.0) / 2.0) * DATA_BYTE_BOUND);
    m_pcRABA->SetData(databyte_index++, un_angularacceleration);

#elif FV_MODE == OBSERVATION_MODE
    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0; unsigned robotId, un_bearing; CRadians bearing;

        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
            byte_index = 1;
        else
            byte_index = 0;

        byte_index++; // the message header type. this is always followed by the robot id.

        robotId = tPackets[i].Data[byte_index];
        bearing = tPackets[i].HorizontalBearing;
        un_bearing = (unsigned)(ToDegrees(bearing).UnsignedNormalize().GetValue() * DATA_BYTE_BOUND / 360.0f);

        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadingsdings& tPackets) ";
            exit(-1);
        }
        m_pcRABA->SetData(databyte_index++, robotId);

        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, const CCI_RangeAndBearingSensor::TReadingsdings& tPackets) ";
            exit(-1);
        }
        m_pcRABA->SetData(databyte_index++, un_bearing);
    }
#endif

    /*if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
      m_pcRABA->SetData(databyte_index, END_BUFFER);*/

    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
    for (t_listMapFVsToRobotIds::iterator it = IdToFVsMap_torelay.begin(); it != IdToFVsMap_torelay.end(); ++it)
    {
        m_pcRABA->SetData(databyte_index++, it->uRobotId);
        m_pcRABA->SetData(databyte_index++, it->uFV);

        // additional information on quality of information uRange, uNumobs_sm, uNumobs_nsm, uNumobs_m;
        m_pcRABA->SetData(databyte_index++, (it->uRange > 100)?100:it->uRange);

        m_pcRABA->SetData(databyte_index++, it->uNumobs_sm  > (unsigned)DATA_BYTE_BOUND?(unsigned)DATA_BYTE_BOUND:it->uNumobs_sm);
        m_pcRABA->SetData(databyte_index++, it->uNumobs_nsm > (unsigned)DATA_BYTE_BOUND?(unsigned)DATA_BYTE_BOUND:it->uNumobs_nsm);
        m_pcRABA->SetData(databyte_index++, it->uNumobs_m   > (unsigned)DATA_BYTE_BOUND?(unsigned)DATA_BYTE_BOUND:it->uNumobs_m);

    }


    if(databyte_index >= m_pcRABA->GetSize()-1)
    {
        std::cerr << " buffer_full " << " WriteToCommunicationChannel(CCI_RangeAndBearingActuatorpcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,        CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,unsigned SelfId, const CCI_RangeAndBearingSensor::TReadingsdings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay) ";
        exit(-1);
    }
    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
}

/****************************************/
/****************************************/

void WriteToCommunicationChannel(CCI_RangeAndBearingActuator* m_pcRABA, unsigned& databyte_index, CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay)
{
    //size_t databyte_index;

//    if ((m_pcRABA->GetData(0) == BEACON_SIGNAL) || (m_pcRABA->GetData(0) == NEST_BEACON_SIGNAL))
//        // sending out a becon signal at data-byte 0; send the other information on data-bytes 1 onwards
//        databyte_index = 1;
//    else
//        databyte_index = 0;


    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET);
    m_pcRABA->SetData(databyte_index++, SelfId);
    m_pcRABA->SetData(databyte_index++, SelfFV);
    m_pcRABA->SetData(databyte_index++, SELF_INFO_PACKET_FOOTER);


    m_pcRABA->SetData(databyte_index++, RELAY_FVS_PACKET);
    bool buffer_full(false);
    for(t_listMapFVsToRobotIds::iterator itd = IdToFVsMap_torelay.begin(); itd != IdToFVsMap_torelay.end(); ++itd)
    {
        m_pcRABA->SetData(databyte_index++, itd->uRobotId);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            /*buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;*/
            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
            exit(-1);
        }

        m_pcRABA->SetData(databyte_index++, itd->uFV);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            /*buffer_full = true;
            m_pcRABA->SetData(databyte_index, END_BUFFER);
            break;*/
            std::cerr << " buffer_full " << " WriteToCommunicationChannel(unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay) " << std::endl;
            exit(-1);
        }
    }

    m_pcRABA->SetData(databyte_index, RELAY_FVS_PACKET_FOOTER);
}

/****************************************/
/****************************************/

void WriteToCommunicationChannel(CCI_RangeAndBearingActuator *m_pcRABA, unsigned& databyte_index, CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds,
                                 t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid)
{
    //size_t databyte_index;

    bool end_buffer_found(false);
//    for (size_t tmp_index = 0; tmp_index < m_pcRABA->GetSize(); ++tmp_index)
//    {
//        if (m_pcRABA->GetData(tmp_index) == RELAY_FVS_PACKET_FOOTER)
//        {
//            end_buffer_found = true;
//            databyte_index = tmp_index + 1;
//            break;
//        }
//    }

    if(databyte_index < m_pcRABA->GetSize())
        end_buffer_found = true;


    if (end_buffer_found == false)
    {
        std::cerr << " RELAY_FVS_PACKET_FOOTER not found  " << " WriteToCommunicationChannel(unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist, t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid) " << std::endl;
        exit(-1);
    }


    if(databyte_index == (m_pcRABA->GetSize()-1))
    {
        std::cerr << " buffer full. no place to write voter packet header type " << std::endl;
        exit(-1);
    }

    m_pcRABA->SetData(databyte_index++, VOTER_PACKET);
    if(databyte_index == (m_pcRABA->GetSize()-1))
    {
        std::cerr << " buffer full. no place to write voter id " << std::endl;
        exit(-1);
    }
    m_pcRABA->SetData(databyte_index++, VoterId);


    if(CRMResultsOnFVDist.size() == 0 && ConsensusLst.size() == 0) // nothing to be written
    {
        if(databyte_index == (m_pcRABA->GetSize()))
        {
            std::cerr << " buffer full. no place to write end buffer " << std::endl;
            exit(-1);
        }
        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
        return;
    }

    bool buffer_full(false);
    /*
     * Write the consensus list to the channel comprising < .... <robot id, its consensus state> ... >
     */
    for (t_listConsensusInfoOnRobotIds::iterator it_cons = ConsensusLst.begin(); it_cons != ConsensusLst.end(); ++it_cons)
    {
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }

        m_pcRABA->SetData(databyte_index++, it_cons->uRobotId);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }

        m_pcRABA->SetData(databyte_index++, (it_cons->consensus_state==1)?ATTACK_CONSENSUS:TOLERATE_CONSENSUS);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }
    }

    if(buffer_full)
    {
        std::cerr << " Written consensus. But buffer full now. No longer able to write the vote packet. complain by exiting" << std::endl;
        std::cerr << " CRMResultsOnFVDist.size() " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
        exit(-1);
    }

    if(!b_CRM_Results_Valid) /* the crm results on the FVs in CRMResultsOnFVDist is no longer valid */
    {
        if(databyte_index == (m_pcRABA->GetSize()))
        {
            std::cerr << " buffer full. no place to write end buffer " << std::endl;
            exit(-1);
        }
        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
        return;
    }

    /*
     * Write the results of CRM to the channel comprising < .... <fv, attack/tolerate state> ... >
     * We dont write the results of all the FVs in the listFVsSensed as they may be some very old FVs no longer present in the swarm.
     * Update: Your FV-ID map may be old too. Other robots may have a better map. Don't curtail information from them.
     */
    buffer_full = false;
    for (t_listFVsSensed::iterator it_fvdist = CRMResultsOnFVDist.begin(); it_fvdist != CRMResultsOnFVDist.end(); ++it_fvdist)
    {
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }

#ifndef VOTESONROBOTID
        m_pcRABA->SetData(databyte_index++, it_fvdist->uFV);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }

        m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
        if(databyte_index == m_pcRABA->GetSize()-1)
        {
            buffer_full = true;
            m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
            break;
        }
#else
        for (t_listMapFVsToRobotIds::iterator it_map = MapFVsToRobotIds.begin(); it_map != MapFVsToRobotIds.end(); ++it_map)
        {
            if(it_map->uFV ==  it_fvdist->uFV)
            {
                m_pcRABA->SetData(databyte_index++, it_map->uRobotId);
                if(databyte_index == m_pcRABA->GetSize()-1)
                {
                    buffer_full = true;
                    m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
                    break;
                }

                m_pcRABA->SetData(databyte_index++, (it_fvdist->uMostWantedState==1)?ATTACK_VOTE:TOLERATE_VOTE);
                if(databyte_index == m_pcRABA->GetSize()-1)
                {
                    buffer_full = true;
                    m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
                    break;
                }
            }
        }

        if(buffer_full)
            break;
#endif
    }

    if(buffer_full)
    {
        std::cerr << " No longer able to write the vote packet. complain by exiting" << std::endl;
        std::cerr << " CRMResultsOnFVDist.size() or VotedOnRobotIDs " << CRMResultsOnFVDist.size() << " ConsensusLst.size() " << ConsensusLst.size() << std::endl;
        exit(-1);
    }

    if(databyte_index != m_pcRABA->GetSize()-1) // END_BUFFER has not yet been placed
        m_pcRABA->SetData(databyte_index, VOTER_PACKET_FOOTER);
}

/****************************************/
/****************************************/

bool func_SortPacketsOnRange (const CCI_RangeAndBearingSensor::SPacket i, const CCI_RangeAndBearingSensor::SPacket j) { return (i.Range < j.Range); }

/****************************************/
/****************************************/


bool ReadFromCommunicationChannel_IdFv(Real m_fInternalRobotTimer, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listMapFVsToRobotIds& listMapFVsToRobotIds,
                                       const CCI_RangeAndBearingSensor::TReadings& tPackets)
{
    /* Read the id and proprioceptively computed FV of your neighbours. Read from communication channel and stored in listMapFVsToRobotIds */
    /* Mark these read FVs as to be relayed. Stored separately in listMapFVsToRobotIds_relay */
    /* Also read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

    bool read_successful(false); // successfully read id and fvs from at least one neighbour

    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations

    // Adding the most recent observations into listMapFVsToRobotIds and marked for relay in listMapFVsToRobotIds_relay
    listMapFVsToRobotIds_relay.clear();
    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0; unsigned robotId, fv;

        if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
            byte_index = 1;
        else
            byte_index = 0;


        if(tPackets[i].Data[byte_index++] == SELF_INFO_PACKET) // this neighbour is not sending me its FVs
        {
            robotId = tPackets[i].Data[byte_index++];
            fv      = tPackets[i].Data[byte_index++];
            read_successful = true;
            byte_index++; // SELF_INFO_PACKET_FOOTER

            listMapFVsToRobotIds_relay.push_back(DetailedInformationFVsSensed(robotId, m_fInternalRobotTimer, fv));
            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer);
        }


        if(tPackets[i].Data[byte_index++] == RELAY_FVS_PACKET) // this neighbour is not sending me its FVs
            for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
            {
                if(tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
                    break;

                robotId = tPackets[i].Data[byteindex1];

                if(tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
                    break;

                fv      = tPackets[i].Data[byteindex1+1];

                UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1); // old information
            }

    }

    return read_successful;
}

/****************************************/
/****************************************/

bool ReadFromCommunicationChannel_RelayedFv(Real m_fInternalRobotTimer, t_listMapFVsToRobotIds &listMapFVsToRobotIds, const CCI_RangeAndBearingSensor::TReadings &tPackets)
{
    /* Only read the the id and fvs relayed by your neighbours, timstamped to behavior at the previous control cycle. Read from communication channel and stored in listMapFVsToRobotIds */

    bool read_successful(false); // successfully read id and fvs from at least one neighbour

    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0;  unsigned robotId, fv, observerId = 999u;
        unsigned range = DATA_BYTE_BOUND+1, numobs_sm = DATA_BYTE_BOUND+1, numobs_nsm = DATA_BYTE_BOUND+1, numobs_m = DATA_BYTE_BOUND+1;

        bool SELF_INFO_PACKET_FOUND(false);
        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
        {
            if(tPackets[i].Data[byte_index] == SELF_INFO_PACKET)
            {
                SELF_INFO_PACKET_FOUND = true;
                byte_index++;
                break;
            }
        }

        if(SELF_INFO_PACKET_FOUND == true)
            observerId = tPackets[i].Data[byte_index];


        bool RELAY_FVS_PACKET_FOUND(false);
        for(byte_index = 0; byte_index < tPackets[i].Data.Size(); ++byte_index)
        {
            if(tPackets[i].Data[byte_index] == RELAY_FVS_PACKET)
            {
                RELAY_FVS_PACKET_FOUND = true;
                byte_index++;
                break;
            }
        }

        if(RELAY_FVS_PACKET_FOUND == false)
            continue;


        //for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=6)
        {
            if (tPackets[i].Data[byteindex1] == RELAY_FVS_PACKET_FOOTER)
                break;

            robotId = tPackets[i].Data[byteindex1];

            if (tPackets[i].Data[byteindex1+1] == RELAY_FVS_PACKET_FOOTER)
                break;

            fv           = tPackets[i].Data[byteindex1+1];

            if (tPackets[i].Data[byteindex1+2] == RELAY_FVS_PACKET_FOOTER)
                break;

            range        = tPackets[i].Data[byteindex1+2];

            if (tPackets[i].Data[byteindex1+3] == RELAY_FVS_PACKET_FOOTER)
                break;

            numobs_sm    = tPackets[i].Data[byteindex1+3];


            if (tPackets[i].Data[byteindex1+4] == RELAY_FVS_PACKET_FOOTER)
                break;

            numobs_nsm   = tPackets[i].Data[byteindex1+4];


            if (tPackets[i].Data[byteindex1+5] == RELAY_FVS_PACKET_FOOTER)
                break;

            numobs_m     = tPackets[i].Data[byteindex1+5];



            /*if(m_uRobotId == robotId)
            {
                std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
            }*/

            /*if(robotId == 16) // && observerId == 93 && ((m_fInternalRobotTimer > 780 && m_fInternalRobotTimer < 790)))
            {
                    std::cerr << "Robot " << observerId << " observing " << robotId << " fv " << fv << std::endl;
            }*/



            read_successful = true;


#ifndef ConsensusOnMapOfIDtoFV
            //if(listMapFVsToRobotIds.size() < 10) /* restrict number of entries to 10 */
            UpdateFvToRobotIdMap(listMapFVsToRobotIds, fv, robotId, m_fInternalRobotTimer-1, range, numobs_sm, numobs_nsm, numobs_m); // old information
#else
            if(observerId == 999u)
            {
                printf("\n observerId was not in packet");
                exit(-1);
            }

            UpdateFvToRobotIdMap(listMapFVsToRobotIds, observerId, fv, robotId, m_fInternalRobotTimer-1, range, numobs_sm, numobs_nsm, numobs_m);
#endif
        }
    }

    return read_successful;
}

/****************************************/
/****************************************/

bool ReadFromCommunicationChannel_VotCon(t_listVoteInformationRobots   &listVoteInformationRobots,
                                         t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                                         const CCI_RangeAndBearingSensor::TReadings &tPackets)
{

    /* Listen to votes and consensus from neighbours */
    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
     *
     * If a consensus is received,
     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
     *
    */

    bool read_successful(false); // successfully read votes / consensus from at least one neighbour


    //std::sort(tPackets.begin(), tPackets.end(), func_SortPacketsOnRange); // used if you want to read only the 10 nearest observations


    for(size_t i = 0; i < tPackets.size(); ++i)
    {
        size_t byte_index = 0;
        unsigned votertId, fv_or_id, attack_tolerate_vote, ConsensusOnRobotId, ConsensusState; unsigned tmp1, tmp2;

        /*if((tPackets[i].Data[0] == BEACON_SIGNAL) || (tPackets[i].Data[0] == NEST_BEACON_SIGNAL)) // data from a beacon  - get the next two bytes
            byte_index = 1;
        else
        byte_index = 0;*/

        bool voter_packet_found(false);
        for (size_t tmp_index = 0; tmp_index < tPackets[i].Data.Size(); ++tmp_index)
        {
            if (tPackets[i].Data[tmp_index] == VOTER_PACKET)
            {
                voter_packet_found = true;
                byte_index = tmp_index + 1;
                break;
            }
        }

        if(voter_packet_found == false) // this neighbour is not sending me any votes or consensus
            continue;


        votertId = tPackets[i].Data[byte_index++];

        for(unsigned byteindex1 = byte_index; byteindex1 < tPackets[i].Data.Size(); byteindex1+=2)
        {
            //printf("\npacket index %d; byteindex1=%d \n",i,byteindex1);

            if(tPackets[i].Data[byteindex1] == VOTER_PACKET_FOOTER)
                break;

            tmp1 = tPackets[i].Data[byteindex1];

            if(tPackets[i].Data[byteindex1+1] == VOTER_PACKET_FOOTER)
                break;

            tmp2 = tPackets[i].Data[byteindex1+1];


            if(tmp2 == ATTACK_VOTE || tmp2 == TOLERATE_VOTE)
            {
                fv_or_id             = tmp1;
                attack_tolerate_vote = (tmp2==ATTACK_VOTE)?1u:2u;

#ifndef VOTESONROBOTID
                UpdateVoterRegistry(listVoteInformationRobots,
                                    listMapFVsToRobotIds,
                                    listConsensusInfoOnRobotIds,
                                    votertId, fv_or_id, attack_tolerate_vote);
#else
                UpdateVoterRegistry(listVoteInformationRobots,
                                    listConsensusInfoOnRobotIds,
                                    votertId, fv_or_id, attack_tolerate_vote);
#endif

            }
            else
            {
                assert(tmp2 == ATTACK_CONSENSUS || tmp2 == TOLERATE_CONSENSUS);
                ConsensusOnRobotId = tmp1;
                ConsensusState     = (tmp2==ATTACK_CONSENSUS)?1u:2u;

                bool b_ConsensusAlreadyEstablishedOnRobot(false);
                for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
                {
                    if(it_cons->uRobotId == ConsensusOnRobotId)
                    {
                        b_ConsensusAlreadyEstablishedOnRobot = true;
                        if (ConsensusState != it_cons->consensus_state)
                        {
                            // Difference in consensus state. there is a disparity in our consensus ????
                            // We can correct state, assuming the lowest id (between my id and voter id) is correct?
                        }
                        break; // robot ids are unique in the consensus list
                    }
                }

                if(!b_ConsensusAlreadyEstablishedOnRobot)
                    listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(ConsensusOnRobotId, ConsensusState));
            }

            read_successful = true;
        }

    }

    return read_successful;
}

/****************************************/
/****************************************/

void ReceiveVotesAndConsensus(t_listVoteInformationRobots &listVoteInformationRobots, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                              t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, const CCI_RangeAndBearingSensor::TReadings &tPackets)
{
    /* Listen to votes and consensus from neighbours */
    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
     *
     * If a consensus is received,
     *                      1. update listConsensusInfoOnRobotIds to include id and ATTACK_CONSENSUS / TOLERATE_CONSENSUS
     *                      2. if listConsensusInfoOnRobotIds already has id with a different CONSENSUS from the received message - problem with discrepancy in consensus???? Could except consensus from lowest voter id as true???
     *
    */

    bool read_status = ReadFromCommunicationChannel_VotCon(listVoteInformationRobots, listMapFVsToRobotIds, listConsensusInfoOnRobotIds, tPackets); /* returns true if successfully read votes or consensus from at least one neighbour*/
}

/****************************************/
/****************************************/

void EstablishConsensus(Real m_fInternalRobotTimer, t_listVoteInformationRobots  &listVoteInformationRobots, t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds)
{
    /* For each robot id in listVoteInformationRobots that is not in listConsensusInfoOnRobotIds
     * If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) (not anymore)
     * Establish temporary consensus on robot id by adding it to listConsensusInfoOnRobotIds
     */

    for (t_listVoteInformationRobots::iterator it_vot = listVoteInformationRobots.begin(); it_vot != listVoteInformationRobots.end(); ++it_vot)
    {
        unsigned VotedOnRobotId = it_vot->uRobotId;
        bool b_ConsensusReachedOnId(false);

        for (t_listConsensusInfoOnRobotIds::iterator it_cons = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
        {
            if (it_cons->uRobotId == VotedOnRobotId)
            {
                b_ConsensusReachedOnId = true;
                break; /* the robot id in consensus list are unique */
            }

        }

        if (b_ConsensusReachedOnId)
            continue; /* consensus already reached for VotedOnRobotId, lets go to the next robot in the listVoteInformationRobots list */
        else
        {
            /*bool b_OneSecondToVotConReset = (((unsigned)m_fInternalRobotTimer %
                                              (unsigned)(VOTCON_RESULTS_VALIDFOR_SECONDS * CProprioceptiveFeatureVector::m_sRobotData.iterations_per_second)) >= 9u);*/
            bool b_OneSecondToVotConReset(false);


            /* If #votes-registered > CONSENSUS_THRESHOLD, or if we are close to the expiry time of current VOTCON_RESULTS_VALIDFOR_SECONDS window (1s remaining) after which the consensus and vote vectors will be cleared (not anymore) */
            if ((it_vot->uVoterIds.size() >= CONSENSUS_THRESHOLD) || b_OneSecondToVotConReset) /* at least one vote will be registered. establish consensus on that */
            {
                /*if(RobotIdStrToInt() == 19 && VotedOnRobotId==15 && (m_fInternalRobotTimer >= 700 && m_fInternalRobotTimer <= 705))
                {
                    for(std::list<unsigned>::iterator tmp = it_vot->uVoterIds.begin(); tmp != it_vot->uVoterIds.end(); ++tmp)
                    {
                        std::cout << "In voter list at m_fInternalRobotTimer " << m_fInternalRobotTimer << " voter ids" <<  (*tmp) << std::endl;
                    }
                }*/

                it_vot->fTimeConsensusReached = m_fInternalRobotTimer;
                listConsensusInfoOnRobotIds.push_back(ConsensusInformationRobots(it_vot->uRobotId,
                                                                                 (it_vot->attackvote_count > it_vot->toleratevote_count)?1u:2u)); /* if equal votes, we tolerate robot*/
            }
        }
    }
}

/****************************************/
/****************************************/
