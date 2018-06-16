#ifndef SENSINGANDCOMMUNICATION_H_
#define SENSINGANDCOMMUNICATION_H_

#include <iostream>
#include <vector>
#include <algorithm>    // std::sort

/****************************************/
/****************************************/
/* ARGoS headers */

/* Definition of the CCI_Controller class. */

/* Definition of the range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>

/* Definition of the range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>

/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>

/* Definition of the differential steering wheel encoder */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_sensor.h>

/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>

/****************************************/
/****************************************/

/* Definition of functions to estimate feature-vectors - proprioceptively, or by observation */

#include "propriofeaturevector.h"
#include "observedfeaturevector.h"
#include "bayesianinferencefeaturevector.h"

/****************************************/
/****************************************/
/* Definition of functions to assimilate the different feature-vectors and perform abnormality detection */

#include "featurevectorsinrobotagent.h"

/****************************************/
/****************************************/

#include "crminrobotagent_optimised.h"

/****************************************/
/****************************************/


#define DATA_BYTE_BOUND 240.0f
#define BEACON_SIGNAL 241
#define NEST_BEACON_SIGNAL 242


#define SELF_INFO_PACKET 243 /* used to encompass info of self, be that the proprioceptively computed FVs, the bearings at which neighbours are observed, or proprioceptively computed angular acceleration.*/
#define SELF_INFO_PACKET_FOOTER 244

#define RELAY_FVS_PACKET 245
#define RELAY_FVS_PACKET_FOOTER 246

#define VOTER_PACKET 247
#define ATTACK_VOTE 248
#define TOLERATE_VOTE 249
#define ATTACK_CONSENSUS 250
#define TOLERATE_CONSENSUS 251
#define VOTER_PACKET_FOOTER 252


#define PROPRIOCEPT_MODE 0
#define OBSERVATION_MODE 1
#define COMBINED_PROPRIOCEPTIVE_OBSERVATION_MODE 2
#define BAYESIANINFERENCE_MODE 3
#define FV_MODE BAYESIANINFERENCE_MODE

/****************************************/
/****************************************/

/*
 * Probability to forget FV in distribution
 */
#define PROBABILITY_FORGET_FV 1.0f //0.001f // We don't need a large history because FVs are being relayed every time-step. That combined with the BI of FVs (small observation window) means that robots will have sufficient FVs of neighbours to make a decision. Also it is difficult to assume that robot behavior has not changed in the last 100s.Will have a CRM_RESULTS_VALIDFOR_SECONDS history instead.

/*
 * Consensus threshold on FVs.
 */
#define CONSENSUS_THRESHOLD 5u /* odd number so that we don't have a tie in attackers and tolerators - but this is just a threshold. number of voters may be more than threshold */

/*
 * The results of the CRM are valid for atmost 10s in the absence of any FVs to run the CRM
 */
#define CRM_RESULTS_VALIDFOR_SECONDS 10.0f // in seconds // assume the robot behaviors have not changed in the last 10s // can we remove this.

/*
 * The vote counts and consensus are valid for atmost VOTCON_RESULTS_VALIDFOR_SECONDS before being refreshed
 * (in seconds)
 * default value is 10.0f, we tried values of 5.0f 15.0f and 20.0f
 */
#define VOTCON_RESULTS_VALIDFOR_SECONDS 10.0f

/****************************************/
/****************************************/


/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;


/* Index to last data entry in RAB actuator data.
*/
extern unsigned m_uRABDataIndex;

/*
 *
 */
void Sense(Real m_fProbForget,
           unsigned RobotId, CCI_RangeAndBearingActuator* pcRABA,
           Real m_fInternalRobotTimer, CCI_RangeAndBearingSensor::TReadings &rabsensor_readings,
           t_listMapFVsToRobotIds& listMapFVsToRobotIds, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listFVsSensed& listFVsSensed,
           t_listVoteInformationRobots&  listVoteInformationRobots, t_listConsensusInfoOnRobotIds& listConsensusInfoOnRobotIds,
           CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector, CObservedFeatureVector &m_cObservationFeatureVector,
           CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector, CRandom::CRNG* m_pcRNG_FVs, std::string &swarmbehav, std::vector<int> &beaconrobots_ids);


/*
 *
 */
void SenseCommunicateDetect(unsigned RobotId, CCI_RangeAndBearingActuator* m_pcRABA, unsigned& DataIndex, CCI_DifferentialSteeringSensor* m_pcWheelsEncoder,
         Real m_fInternalRobotTimer, CCI_RangeAndBearingSensor::TReadings &rabsensor_readings,
         t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listMapFVsToRobotIds& MapFVsToRobotIds_Relay, t_listFVsSensed& CRMResultsOnFVDist,
         t_listVoteInformationRobots&  VoteInformationRobots, t_listConsensusInfoOnRobotIds& ConsensusLst,
         CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
         CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
         bool b_CRM_Run, Real m_fCRM_RUN_TIMESTAMP, CRMinRobotAgentOptimised* crminAgent, CRandom::CRNG* m_pcRNG_FVs, unsigned& m_uRobotFV, std::string &swarmbehav,
         std::vector<int> &beaconrobots_ids);


/*
 *
 */
void SendIdSelfBearingAndObsFVsToNeighbours(CCI_RangeAndBearingActuator* m_pcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                            CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector, unsigned RobotId,
                                            const CCI_RangeAndBearingSensor::TReadings& tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay);

/*
 *
 */
void SendFVsToNeighbours(CCI_RangeAndBearingActuator* m_pcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                         CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                         unsigned RobotId, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay);


/*
 *
 */
void SendCRMResultsAndConsensusToNeighbours(CCI_RangeAndBearingActuator* m_pcRABA,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                            CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                            unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist,
                                            t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid);

/*
 *
 */
void WriteToCommunicationChannel(CCI_RangeAndBearingActuator* m_pcRABA, unsigned &databyte_index, CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned SelfId, const CCI_RangeAndBearingSensor::TReadings &tPackets, t_listMapFVsToRobotIds &IdToFVsMap_torelay);


/*
 *
 */
void WriteToCommunicationChannel(CCI_RangeAndBearingActuator* m_pcRABA, unsigned &databyte_index,  CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned SelfId, unsigned SelfFV, t_listMapFVsToRobotIds& IdToFVsMap_torelay);


/*
 *
 */
void WriteToCommunicationChannel(CCI_RangeAndBearingActuator* m_pcRABA,  unsigned &databyte_index, CProprioceptiveFeatureVector &m_cProprioceptiveFeatureVector,
                                 CObservedFeatureVector &m_cObservationFeatureVector, CBayesianInferenceFeatureVector &m_cBayesianInferredFeatureVector,
                                 unsigned VoterId, t_listMapFVsToRobotIds& MapFVsToRobotIds, t_listFVsSensed& CRMResultsOnFVDist,
                                 t_listConsensusInfoOnRobotIds& ConsensusLst, bool b_CRM_Results_Valid);


/*
 *
 */
bool ReadFromCommunicationChannel_IdFv(Real m_fInternalRobotTimer, t_listMapFVsToRobotIds& listMapFVsToRobotIds_relay, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                                       const CCI_RangeAndBearingSensor::TReadings& tPackets);


/*
 *
 */
bool ReadFromCommunicationChannel_RelayedFv(Real m_fInternalRobotTimer, t_listMapFVsToRobotIds &listMapFVsToRobotIds, const CCI_RangeAndBearingSensor::TReadings& tPackets);


/*
 *
 */
bool ReadFromCommunicationChannel_VotCon(t_listVoteInformationRobots &listVoteInformationRobots, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, const CCI_RangeAndBearingSensor::TReadings& tPackets);


/*
 *
 */
void ReceiveVotesAndConsensus(t_listVoteInformationRobots &listVoteInformationRobots, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                              t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, const CCI_RangeAndBearingSensor::TReadings& tPackets);

/*
 *
 */
void EstablishConsensus(Real m_fInternalRobotTimer, t_listVoteInformationRobots   &listVoteInformationRobots, t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds);

#endif
