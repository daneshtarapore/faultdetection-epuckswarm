#include <list>
#include <assert.h>

/******************************************************************************/
/******************************************************************************/

#include "featurevectorsinrobotagent.h"

/******************************************************************************/
/******************************************************************************/


void UpdaterFvDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                           CRandom::CRNG* m_pcRNG, Real m_fProbForget)
{

    t_listFVsSensed::iterator it_dist, it_history;

    // forget old FVs in distribution with probability m_fProbForgetFV
    it_dist = listFVsSensed.begin();
    while(it_dist != listFVsSensed.end())
    {
        if(m_pcRNG->Uniform(CRange<Real>(0.0f, 1.0f)) <= m_fProbForget)
        {
            it_dist = listFVsSensed.erase(it_dist);
            continue;
        }
        ++it_dist;
    }

    t_listFVsSensed tmp_list = t_listFVsSensed(listFVsSensed.begin(), listFVsSensed.end());

    listFVsSensed.clear();

    // updated listFVsSensed distribution with the most recent number of robots for different FVs.
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        if(it_map->uFV == 999u) // a stron enough consensus has not been reached to determine the one or more value of features of this feature vector.
            continue;

        double increment = 1.0f;

        bool b_EntryInserted(false);

        // check if fv is in listFVsSensed
        // if so, update the value it holds by increment
        // if not insert it (while keeping list sorted based on fv) and initialize its value by increment
        for (it_dist = listFVsSensed.begin(); it_dist != listFVsSensed.end(); ++it_dist)
        {
            if(it_dist->uFV == it_map->uFV)
            {
                // if fv is already present
                it_dist->fRobots += increment;
                b_EntryInserted = true;
                break;
            }

            if(it_dist->uFV > it_map->uFV)
            {   // we assume the list is kept sorted.
                // if fv is absent
                listFVsSensed.insert(it_dist, StructFVsSensed(it_map->uFV, increment));
                b_EntryInserted = true;
                break;
            }
        }

        if(b_EntryInserted)
            continue;

        // when the list is empty or item is to be inserted in the end
        listFVsSensed.push_back(StructFVsSensed(it_map->uFV, increment));
    }

    // integrate into the current list listFVsSensed the history from tmp_list
    // if FV is the same in the current list, and in the history, - the history for that FV is ignored.
    // else the FV is integrated into the current list
    it_dist = listFVsSensed.begin(); it_history = tmp_list.begin();
    while(it_history != tmp_list.end() && it_dist != listFVsSensed.end())
    {
        if(it_history->uFV == it_dist->uFV)
        {
            ++it_history; ++it_dist;
            continue;
        }

        if(it_history->uFV < it_dist->uFV)
        {
                listFVsSensed.insert(it_dist, StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
                ++it_history;
        }
        else
             ++it_dist;
   }

    while(it_history != tmp_list.end())
    {
            listFVsSensed.push_back(StructFVsSensed(it_history->uFV, it_history->fRobots, it_history->uMostWantedState));
            ++it_history;
    }
}

/******************************************************************************/
/******************************************************************************/

void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                          unsigned int fv, unsigned robotId, double timesensed,
                          unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
{
    t_listMapFVsToRobotIds::iterator itd;

    // check if robotId is in listMapFVsToRobotIds
    // if so, than update its fv if timesensed is more recent.
    // if not, than add its fv to the list

    bool robot_present(false);
    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
    {
        if((*itd).uRobotId == robotId)
        {
            robot_present = true;

            // if the robot id is already present, add if the new information is more recent
            if(timesensed > (*itd).fTimeSensed)
            {
                (*itd).fTimeSensed = timesensed;
                (*itd).uRobotId = robotId;
                (*itd).uFV = fv;

                (*itd).uRange = range;
                (*itd).uNumobs_sm = numobs_sm;
                (*itd).uNumobs_nsm = numobs_nsm;
                (*itd).uNumobs_m = numobs_m;
            }
            return;
        }
    }

    if(!robot_present) // if the list is empty or the robot with given id is not present in the list
        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(robotId, timesensed, fv, range, numobs_sm, numobs_nsm, numobs_m));
}

/******************************************************************************/
/******************************************************************************/

//void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
//                          unsigned int ObserverRobotId, unsigned int fv, unsigned ObservedRobotId, double timesensed)
//{
//    t_listMapFVsToRobotIds::iterator itd;

//    // check if ObservedRobotId is in listMapFVsToRobotIds
//    /*
//     * if so, then check if ObserverRobotId is part of the entry.
//     *        if so,  than update its fv if timesensed is more recent.
//     *        if not, than add the ObserverRobotId, timesensed and fv to the entry
//     *
//     *  if not, then add ObservedRobotId, and also add ObserverRobotId, timesensed and fv to the entry
//     */

//    bool robot_present(false);
//    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
//    {
//        if((*itd).uRobotId == ObservedRobotId)
//        {
//            robot_present = true;

//            bool observer_robot_present(false);
//            //for(std::vector<unsigned>::iterator itd1 = (*itd).vec_ObserverRobotIds.begin(); itd1 != (*itd).vec_ObserverRobotIds.end(); ++itd1)
//            for(size_t vec_index = 0; vec_index < (*itd).vec_ObserverRobotIds.size(); ++vec_index)
//            {
//                if (ObserverRobotId == (*itd).vec_ObserverRobotIds[vec_index])
//                {
//                    observer_robot_present = true;

//                    if(timesensed > (*itd).vec_TimeObserved[vec_index])
//                    {
//                        (*itd).vec_TimeObserved[vec_index]     = timesensed;
//                        (*itd).vec_ObservedRobotFVs[vec_index] = fv;
//                    }
//                    return;
//                }
//            }

//            if(!observer_robot_present)
//            {
//                (*itd).AddNewInformationFVsSensed(ObserverRobotId, timesensed, fv);
//                return;
//            }
//        }
//    }

//    if(!robot_present) // if the list is empty or the robot with given id is not present in the list. we set the fv to 999u, a value that will never be reached. so if consensus is not reached on the fv, we know about it from this value.
//        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(ObserverRobotId, ObservedRobotId, timesensed, 999u));
//}

/******************************************************************************/
/******************************************************************************/

void UpdateFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
                          unsigned int ObserverRobotId, unsigned int fv, unsigned ObservedRobotId, double timesensed,
                          unsigned int range, unsigned int numobs_sm, unsigned int numobs_nsm, unsigned int numobs_m)
{
    t_listMapFVsToRobotIds::iterator itd;

    // check if ObservedRobotId is in listMapFVsToRobotIds
    /*
     * if so, then check if ObserverRobotId is part of the entry.
     *        if so,  than update its fv if timesensed is more recent.
     *        if not, than add the ObserverRobotId, timesensed and fv to the entry
     *
     *  if not, then add ObservedRobotId, and also add ObserverRobotId, timesensed and fv to the entry
     */

    bool robot_present(false);
    for (itd = listMapFVsToRobotIds.begin(); itd != listMapFVsToRobotIds.end(); ++itd)
    {
        if((*itd).uRobotId == ObservedRobotId)
        {
            robot_present = true;

            bool observer_robot_present(false);
            //for(std::vector<unsigned>::iterator itd1 = (*itd).vec_ObserverRobotIds.begin(); itd1 != (*itd).vec_ObserverRobotIds.end(); ++itd1)
            for(size_t vec_index = 0; vec_index < (*itd).vec_ObserverRobotIds.size(); ++vec_index)
            {
                if (ObserverRobotId == (*itd).vec_ObserverRobotIds[vec_index])
                {
                    observer_robot_present = true;

                    if(timesensed > (*itd).vec_TimeObserved[vec_index])
                    {
                        (*itd).vec_TimeObserved[vec_index]     = timesensed;
                        (*itd).vec_ObservedRobotFVs[vec_index] = fv;

                        (*itd).vec_ObserverRobotRange[vec_index] = range;
                        (*itd).vec_ObserverRobotNumObservations_SM[vec_index] = numobs_sm;
                        (*itd).vec_ObserverRobotNumObservations_nSM[vec_index] = numobs_nsm;
                        (*itd).vec_ObserverRobotNumObservations_M[vec_index] = numobs_m;
                    }
                    return;
                }
            }

            if(!observer_robot_present)
            {
                (*itd).AddNewInformationFVsSensed(ObserverRobotId, timesensed, fv, range, numobs_sm, numobs_nsm, numobs_m);
                return;
            }
        }
    }

    if(!robot_present) // if the list is empty or the robot with given id is not present in the list. we set the fv to 999u, a value that will never be reached. so if consensus is not reached on the fv, we know about it from this value.
        listMapFVsToRobotIds.push_back(DetailedInformationFVsSensed(ObserverRobotId, ObservedRobotId, timesensed, 999u, range, numobs_sm, numobs_nsm, numobs_m));
}

/******************************************************************************/
/******************************************************************************/

bool sortfventries (const DetailedInformationFVsSensed& i, const DetailedInformationFVsSensed& j) { return (i.fTimeSensed < j.fTimeSensed); }

void TrimFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds, Real f_CurrentRobotTime, Real f_FvToId_MaintenanceTime)
{
#ifndef ConsensusOnMapOfIDtoFV
    // when do we delete entries?
    // if an entry is older than f_FvToId_MaintenanceTime, we delete it.
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end()) // REMEMBER A FOR LOOP HERE WILL CAUSE RUNTIME ERRORS IF THE LAST ENTRY IN POPULATION IS DELETED.
    {
         if (itd->fTimeSensed < (f_CurrentRobotTime - f_FvToId_MaintenanceTime))
         {
             itd = listMapFVsToRobotIds.erase(itd);
             continue;
         }

         ++itd;
    }
#else


    // when do we delete entries?
    // if all subentries in vec_ObserverRobotIds are older than f_FvToId_MaintenanceTime, we delete it.
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end()) // REMEMBER A FOR LOOP HERE WILL CAUSE RUNTIME ERRORS IF THE LAST ENTRY IN POPULATION IS DELETED.
    {
        std::vector<unsigned>::iterator it_observer     = itd->vec_ObserverRobotIds.begin();
        std::vector<Real>::iterator     it_timeobserved = itd->vec_TimeObserved.begin();
        std::vector<unsigned>::iterator it_observerdfv  = itd->vec_ObservedRobotFVs.begin();

        std::vector<unsigned>::iterator it_observer_range       = itd->vec_ObserverRobotRange.begin();
        std::vector<unsigned>::iterator it_observer_numobs_sm   = itd->vec_ObserverRobotNumObservations_SM.begin();
        std::vector<unsigned>::iterator it_observer_numobs_nsm  = itd->vec_ObserverRobotNumObservations_nSM.begin();
        std::vector<unsigned>::iterator it_observer_numobs_m    = itd->vec_ObserverRobotNumObservations_M.begin();



        while(it_timeobserved != itd->vec_TimeObserved.end())
        {
            if ((*it_timeobserved) < (f_CurrentRobotTime - f_FvToId_MaintenanceTime))
            {
                it_observer     = itd->vec_ObserverRobotIds.erase(it_observer);
                it_timeobserved = itd->vec_TimeObserved.erase(it_timeobserved);
                it_observerdfv  = itd->vec_ObservedRobotFVs.erase(it_observerdfv);

                it_observer_range       = itd->vec_ObserverRobotRange.erase(it_observer_range);
                it_observer_numobs_sm   = itd->vec_ObserverRobotNumObservations_SM.erase(it_observer_numobs_sm);
                it_observer_numobs_nsm  = itd->vec_ObserverRobotNumObservations_nSM.erase(it_observer_numobs_nsm);
                it_observer_numobs_m    = itd->vec_ObserverRobotNumObservations_M.erase(it_observer_numobs_m);

                continue;
            }
            ++it_observer; ++it_timeobserved; ++it_observerdfv;
            ++it_observer_range; ++it_observer_numobs_sm; ++it_observer_numobs_nsm; ++it_observer_numobs_m;
        }


        if (itd->vec_ObserverRobotIds.size() == 0u)
        {
            assert(itd->vec_TimeObserved.size()     == 0u);
            assert(itd->vec_ObservedRobotFVs.size() == 0u);

            assert(itd->vec_ObserverRobotRange.size() == 0u);
            assert(itd->vec_ObserverRobotNumObservations_SM.size() == 0u);
            assert(itd->vec_ObserverRobotNumObservations_nSM.size() == 0u);
            assert(itd->vec_ObserverRobotNumObservations_M.size() == 0u);

            itd = listMapFVsToRobotIds.erase(itd);
            continue;
        }

         ++itd;
    }
#endif
}

/******************************************************************************/
/******************************************************************************/

void PrintFvToRobotIdMap(t_listMapFVsToRobotIds &listMapFVsToRobotIds)
{
    std::cout << "Number of entries in detailed map is " << listMapFVsToRobotIds.size() << std::endl;

    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        std::cout << "Observed robot id " << itd->uRobotId << " has fv " << itd->uFV << ", first time sensed is " << itd->fTimeSensed
                  << "range " << itd->uRange << " num s-m obs " << itd->uNumobs_sm << ", num s-nm obs " << itd->uNumobs_nsm
                  << ", num m obs " << itd->uNumobs_m
                  << std::endl;
        ++itd;
    }
}

/******************************************************************************/
/******************************************************************************/

void PrintFvToRobotIdMap(unsigned u_MapOfRobotId, t_listMapFVsToRobotIds &listMapFVsToRobotIds, unsigned u_ObservedRobotId)
{
    std::cout << "Number of entries in map of " << u_MapOfRobotId << " is " << listMapFVsToRobotIds.size() << std::endl;

    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        if(u_ObservedRobotId != 99999u && itd->uRobotId != u_ObservedRobotId)
        //if(itd->uRobotId != 99999u && itd->uRobotId != u_ObservedRobotId)
        {
            ++itd;
            continue;
        }

        std::cout << "Observed robot id " << itd->uRobotId << " has " << itd->vec_ObserverRobotIds.size() <<  " sub-entries " << std::endl;

        for(size_t index = 0; index < itd->vec_ObserverRobotIds.size(); ++index)
        {
            std::cout << "   Sub-entry " << index <<
                         " ObserverId " << itd->vec_ObserverRobotIds[index] <<
                         " ObservedFV " << itd->vec_ObservedRobotFVs[index] << " Time observed " << itd->vec_TimeObserved[index]
                         << "range " << itd->vec_ObserverRobotRange[index] << " num s-m obs " << itd->vec_ObserverRobotNumObservations_SM[index]
                         << ", num s-nm obs " << itd->vec_ObserverRobotNumObservations_nSM[index]
                         << ", num m obs " << itd->vec_ObserverRobotNumObservations_M[index]
                         << std::endl;

        }

        std::cout << "Observed robot fv (best selected) " << itd->uFV << std::endl << std::endl;
        ++itd;
    }
}

/******************************************************************************/
/******************************************************************************/

#ifdef ConsensusOnMapOfIDtoFV

void SelectBestFVFromAllObservedFVs(t_listMapFVsToRobotIds &listMapFVsToRobotIds, unsigned u_NumFeatures, CRandom::CRNG* m_pcRNG_FVs, unsigned uSelfRobotId)
{
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        itd->SelectBestFVFromAllObservedFVs(u_NumFeatures, m_pcRNG_FVs, uSelfRobotId);
        ++itd;
    }
}
#endif

/******************************************************************************/
/******************************************************************************/

void IntegrateAttackTolerateDecisions(t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                                      unsigned fv,
                                      unsigned attack_tolerate_prevote)
{
    t_listMapFVsToRobotIds::iterator itd = listMapFVsToRobotIds.begin();
    while(itd != listMapFVsToRobotIds.end())
    {
        if(itd->uFV == fv)
        {
            if (attack_tolerate_prevote == 1)
                itd->f_TimesAttacked+= 1.0f;
            else
                itd->f_TimesTolerated+= 1.0f;
        }
        ++itd;
    }
}

/******************************************************************************/
/******************************************************************************/

void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id,
                         unsigned votedon_id,
                         unsigned attack_tolerate_vote)
{

    /* Read the voter id:
     * Followed by <id, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. if votedon_id is in listConsensusInfoOnRobotIds, ignore vote for this robot id.
     *                      2. if voter id has voted on votedon_id before, ignore vote.
    */


    /* map the fv to the robot id  (one-many function) */
    unsigned mappedRobotId = votedon_id;

    /* if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. */
    bool b_ConsensusReachedOnId(false);
    for(t_listConsensusInfoOnRobotIds::iterator it_cons  = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
    {
        if(it_cons->uRobotId == mappedRobotId)
        {
            b_ConsensusReachedOnId = (true);
            break;
        }
    }

    if(b_ConsensusReachedOnId)
        return;

    /* consensus has not yet been reached for mappedRobotId. lets vote*/
    bool b_MappedRobotVotedOnBefore(false);
    for(t_listVoteInformationRobots::iterator it_vote = listVoteInformationRobots.begin(); it_vote != listVoteInformationRobots.end(); ++it_vote)
    {
        /* robot is on the list of robots being voted on */
        if(it_vote->uRobotId == mappedRobotId)
        {
            b_MappedRobotVotedOnBefore = (true);

            /* if voter id has voted on mapped robot id before, ignore vote */
            bool b_VoterIdHasVotedBefore(false);
            for(std::list<unsigned>::iterator it_voterids = it_vote->uVoterIds.begin(); it_voterids != it_vote->uVoterIds.end(); ++it_voterids)
            {
                if((*it_voterids) == voter_id) /* yes voterid has voted before on mappedrobotid - ignore this vote */
                {
                    b_VoterIdHasVotedBefore = (true);
                    break;
                }
            }

            if(!b_VoterIdHasVotedBefore) /* no voterid has not voted before on mappedrobotid - ignore this vote */
            {
                it_vote->uVoterIds.push_back(voter_id);

                if (attack_tolerate_vote == 1)
                    it_vote->attackvote_count++;
                else
                    it_vote->toleratevote_count++;
            }

            break; /* robots on the list of robots being voted on, are unique */
        }
    }

    /* if mappedRobotId robot has not been voted on before, lets vote */
    if(!b_MappedRobotVotedOnBefore)
        listVoteInformationRobots.push_back(VoteInformationRobots(mappedRobotId, voter_id, attack_tolerate_vote));
}

/******************************************************************************/
/******************************************************************************/

void UpdateVoterRegistry(t_listVoteInformationRobots   &listVoteInformationRobots,
                         t_listMapFVsToRobotIds   &listMapFVsToRobotIds,
                         t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds,
                         unsigned voter_id,
                         unsigned fv,
                         unsigned attack_tolerate_vote, bool filterownersvote) // filterownersvote set if filter enabled and robot is casting vote based on its own CRM decisions as opposed to casting proxy votes (nbrs send their crm decision on fv. robot maps these decisions to corresponding robot ids) from neighbours.
{

    /* Read the voter id:
     * Followed by <fv, ATTACK_VOTE / TOLERATE_VOTE > or <id, ATTACK_CONSENSUS / TOLERATE_CONSENSUS >
     * If a vote is received,
     *                      1. map the fv to the robot id (if none existed - ignore vote???)
     *                      2. if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search.
     *                      3. if voter id has voted on mapped robot id before, ignore vote. But remember, the voted fv may map to other robot ids so continue search.
    */

    bool b_IdForFvFound(false);
    for(t_listMapFVsToRobotIds::iterator it_map = listMapFVsToRobotIds.begin(); it_map != listMapFVsToRobotIds.end(); ++it_map)
    {
        if(it_map->uFV == fv)
        {
            if(filterownersvote)
            {
                if (it_map->f_TimesAttacked / (it_map->f_TimesAttacked + it_map->f_TimesTolerated) > 0.5f)
                    attack_tolerate_vote = 1;
                else
                    attack_tolerate_vote = 2;
            }

            b_IdForFvFound = (true);

            /* map the fv to the robot id  (one-many function) */
            unsigned mappedRobotId = it_map->uRobotId;

            /* if mapped robot id is in listConsensusInfoOnRobotIds, ignore vote for this robot id. But remember, the voted fv may map to other robot ids so continue search. */
            bool b_ConsensusReachedOnId(false);
            for(t_listConsensusInfoOnRobotIds::iterator it_cons  = listConsensusInfoOnRobotIds.begin(); it_cons != listConsensusInfoOnRobotIds.end(); ++it_cons)
            {
                if(it_cons->uRobotId == mappedRobotId)
                {
                    b_ConsensusReachedOnId = (true);
                    break;
                }
            }

            if(b_ConsensusReachedOnId) /* continue search for other robot ids. the fv-id function is one-many*/
                continue;

            /* consensus has not yet been reached for mappedRobotId. lets vote*/
            bool b_MappedRobotVotedOnBefore(false);
            for(t_listVoteInformationRobots::iterator it_vote = listVoteInformationRobots.begin(); it_vote != listVoteInformationRobots.end(); ++it_vote)
            {
                /* robot is on the list of robots being voted on */
                if(it_vote->uRobotId == mappedRobotId)
                {
                    b_MappedRobotVotedOnBefore = (true);

                    /* if voter id has voted on mapped robot id before, ignore vote */
                    bool b_VoterIdHasVotedBefore(false);
                    for(std::list<unsigned>::iterator it_voterids = it_vote->uVoterIds.begin(); it_voterids != it_vote->uVoterIds.end(); ++it_voterids)
                    {
                        if((*it_voterids) == voter_id) /* yes voterid has voted before on mappedrobotid - ignore this vote */
                        {
                            b_VoterIdHasVotedBefore = (true);
                            break;
                        }
                    }

                    if(!b_VoterIdHasVotedBefore) /* no voterid has not voted before on mappedrobotid - ignore this vote */
                    {
                        it_vote->uVoterIds.push_back(voter_id);

                        if (attack_tolerate_vote == 1)
                            it_vote->attackvote_count++;
                        else
                            it_vote->toleratevote_count++;
                    }

                    break; /* robots on the list of robots being voted on, are unique */
                }

            }

            /* if mappedRobotId robot has not been voted on before, lets vote */
            if(!b_MappedRobotVotedOnBefore)
                listVoteInformationRobots.push_back(VoteInformationRobots(mappedRobotId, voter_id, attack_tolerate_vote));

        }
    }

    if(!b_IdForFvFound)
    {
        /* no robot id was found in the map, for the voted on fv;
         *                              options: 1. ignore vote???
         *                                       2. cast vote on ids with fvs close to the voted on fv?
          */
    }

}

/******************************************************************************/
/******************************************************************************/

void PrintVoterRegistry(unsigned u_VoterRegistryOfRobotId, t_listVoteInformationRobots &listVoteInformationRobots, unsigned u_VotedOnRobotId)
{
    std::cout << "Number of entries in vote registry of " << u_VoterRegistryOfRobotId << " is " << listVoteInformationRobots.size() << std::endl;

    t_listVoteInformationRobots::iterator itd = listVoteInformationRobots.begin();
    while(itd != listVoteInformationRobots.end())
    {
        if(u_VotedOnRobotId != 99999u && itd->uRobotId != u_VotedOnRobotId)
        //if(itd->uRobotId != 99999u && itd->uRobotId != u_VotedOnRobotId)
        {
            ++itd;
            continue;
        }

        std::cout << "Voted on robot id " << itd->uRobotId << " has received " << itd->uVoterIds.size() <<  " votes " << std::endl;
        size_t index = 0;
        for(std::list <unsigned>::iterator itd1 = itd->uVoterIds.begin(); itd1 != itd->uVoterIds.end(); ++itd1)
        {
            std::cout << "   Sub-entry " << index <<
                         " VotedId " << (*itd1) << std::endl;
             ++index;

        }

        std::cout << "Numbers of attack voters " << itd->attackvote_count << "; tolerate voters " << itd->toleratevote_count << ". Time consensus reached " << itd->fTimeConsensusReached << std::endl << std::endl;
        ++itd;
    }
}

/******************************************************************************/
/******************************************************************************/

void PrintConsensusRegistry(unsigned u_ConsensusRegistryOfRobotId, t_listConsensusInfoOnRobotIds &listConsensusInfoOnRobotIds, unsigned u_ConsensusOnRobotId)
{
    std::cout << "Number of entries in consensus registry of " << u_ConsensusRegistryOfRobotId << " is " << listConsensusInfoOnRobotIds.size() << std::endl;

    t_listConsensusInfoOnRobotIds::iterator itd = listConsensusInfoOnRobotIds.begin();
    while(itd != listConsensusInfoOnRobotIds.end())
    {
        if(u_ConsensusOnRobotId != 99999u && itd->uRobotId != u_ConsensusOnRobotId)
        {
            ++itd;
            continue;
        }

        std::cout << "Consensus state on robot id " << itd->uRobotId << " is " << itd->consensus_state << std::endl;
        ++itd;
    }

    std::cout << std::endl << std::endl;
}

/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/

//void UpdateFeatureVectorDistribution(t_listMapFVsToRobotIds &listMapFVsToRobotIds,
//                                     unsigned int fv, unsigned robotId, double timesensed)
//{
//    UpdateFVDetails(listMapFVsToRobotIds, fv, robotId, timesensed);
//}

/******************************************************************************/
/******************************************************************************/


//void UpdateFeatureVectorDistribution(t_listFVsSensed &listFVsSensed, t_listMapFVsToRobotIds &listMapFVsToRobotIds,
//                                     unsigned int fv, unsigned robotId, double timesensed)
//{

//    double increment = 1.0; // increment by one robot
//    UpdaterDistribution(listFVsSensed, fv, increment);
//    UpdateFVDetails(listMapFVsToRobotIds, fv, robotId, timesensed);
//}
