#include "homswarm_loop_functions.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <controllers/epuck_hom_swarm/epuck_hom_swarm.h>



/****************************************/
/****************************************/

CHomSwarmLoopFunctions::CHomSwarmLoopFunctions() :
   m_pcFloor(NULL),
   m_pcRNG(CRandom::CreateRNG("argos"))
{
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Init(TConfigurationNode& t_node)
{
   try
    {
      TConfigurationNode& tParams = GetNode(t_node, "params");
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();

      /* Get the output file name from XML */
      GetNodeAttribute(tParams, "output", m_strOutput);
      /* Open the file, erasing its contents */
      m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
      //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;

       GetNodeAttribute(tParams, "arenalength", fArenaLength);

   }
   catch(CARGoSException& ex)
      THROW_ARGOSEXCEPTION_NESTED("Error parsing homswarm loop function.", ex);


    // counting the number of robots in the swarm
    u_num_epucks = 0u;
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        u_num_epucks++;
    }


    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());
        cController.GetExperimentType().SetNumEPuckRobotsInSwarm(u_num_epucks);
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Reset()
{  /* Close the file */
   m_cOutput.close();
   /* Open the file, erasing its contents */
   m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
   //m_cOutput << "# clock\trobot_id\trobot_fv\tobserving_robots\tattack\ttolerate\tundecided" << std::endl;
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PreStep()
{
    CVector2 cPos1, cPos2;
    CSpace::TMapPerType& m_cEpucks1 = GetSpace().GetEntitiesByType("e-puck");
    CSpace::TMapPerType& m_cEpucks2 = GetSpace().GetEntitiesByType("e-puck");

    for(CSpace::TMapPerType::iterator it = m_cEpucks1.begin();
        it != m_cEpucks1.end();
        ++it)
    {
        /* Get handle to foot-bot entity and controller */
        CEPuckEntity& cEPuck1 = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController1 = dynamic_cast<CEPuckHomSwarm&>(cEPuck1.GetControllableEntity().GetController());

        unsigned rob_id = cController1.RobotIdStrToInt();

        cPos1.Set(cEPuck1.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                  cEPuck1.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

        if(rob_id == 15){


            for(CSpace::TMapPerType::iterator it2 = m_cEpucks2.begin();
                it2 != m_cEpucks2.end();
                ++it2)
            {
                CEPuckEntity& cEPuck2 = *any_cast<CEPuckEntity*>(it2->second);
                CEPuckHomSwarm& cController2 = dynamic_cast<CEPuckHomSwarm&>(cEPuck2.GetControllableEntity().GetController());

                unsigned rob_id2 = cController2.RobotIdStrToInt();

                cPos2.Set(cEPuck2.GetEmbodiedEntity().GetOriginAnchor().Position.GetX(),
                          cEPuck2.GetEmbodiedEntity().GetOriginAnchor().Position.GetY());

                float dist = (cPos1- cPos2).Length();
                std::cerr << "Robot id: " << rob_id2 << " dist " << dist << std::endl;
            }
        }
    }

    float start_firsttrans_sec = 500.0f, finish_firsttosecondtrans_sec = 1000.0f;


    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        if(GetSpace().GetSimulationClock() < start_firsttrans_sec * cController.m_sRobotDetails.iterations_per_second)
        { }

        /*transition robot ids from robot_ids_behav1 to robot_ids_behav2 */
        else if((GetSpace().GetSimulationClock() >= start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second) &&
                (GetSpace().GetSimulationClock() < finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second))
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - start_firsttrans_sec  * cController.m_sRobotDetails.iterations_per_second;

            if(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav1.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav2.push_back(cController.GetExperimentType().robot_ids_behav1.front());
                        cController.GetExperimentType().robot_ids_behav1.pop_front();
                    }
                }
            }
        }

        /* transition robot ids from robot_ids_behav2 back to robot_ids_behav1 */
        else if (GetSpace().GetSimulationClock() >= finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second)
        {
            Real time_between_robots_trans_behav = cController.GetExperimentType().time_between_robots_trans_behav * cController.m_sRobotDetails.iterations_per_second;

            Real m_fInternalRobotTimer = GetSpace().GetSimulationClock() - finish_firsttosecondtrans_sec  * cController.m_sRobotDetails.iterations_per_second;

//            if((((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0) && (cController.GetExperimentType().robot_ids_behav2.size() > 0u))
//            {
//                cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
//                cController.GetExperimentType().robot_ids_behav2.pop_front();
//            }

            if(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
            {
                if(time_between_robots_trans_behav > 0.0f)
                {
                    if(((unsigned)m_fInternalRobotTimer)%((unsigned)time_between_robots_trans_behav) == 0)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
                else if(time_between_robots_trans_behav == 0.0f)
                {
                    while(cController.GetExperimentType().robot_ids_behav2.size() > 0u)
                    {
                        cController.GetExperimentType().robot_ids_behav1.push_back(cController.GetExperimentType().robot_ids_behav2.front());
                        cController.GetExperimentType().robot_ids_behav2.pop_front();
                    }
                }
            }

        }
    }
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::PostStep()
{
    if(GetSpace().GetSimulationClock() <= 450.0)
        return;

#ifndef RECORDSELFVOTESONLY
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        unsigned observed_rob_id = cController.RobotIdStrToInt();
        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

        std::list<unsigned> list_Consensus_Tolerators, list_Consensus_Attackers;
        list_Consensus_Tolerators.clear(); list_Consensus_Attackers.clear();


        for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
        {
            CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
            CEPuckHomSwarm& cController_Observers = dynamic_cast<CEPuckHomSwarm&>(cEPuck_Observers.GetControllableEntity().GetController());

            unsigned observers_rob_id = cController_Observers.RobotIdStrToInt();

            t_listConsensusInfoOnRobotIds listConsensusInfoOnRobotIds = cController_Observers.GetListConsensusInfoOnRobotIds();

            for (t_listConsensusInfoOnRobotIds ::iterator it_con = listConsensusInfoOnRobotIds.begin(); it_con != listConsensusInfoOnRobotIds.end(); ++it_con)
            {
                if(it_con->uRobotId == observed_rob_id && it_con->consensus_state == 1)
                    list_Consensus_Attackers.push_back(observers_rob_id);

                else if(it_con->uRobotId == observed_rob_id && it_con->consensus_state == 2)
                    list_Consensus_Tolerators.push_back(observers_rob_id);
            }
        }

        m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t"
                  << "Id: " << observed_rob_id << "\t"
                  << "FV: " << observed_rob_fv << "\t";

        m_cOutput << "Consensus_Tolerators: ";
        for (std::list<unsigned>::iterator it_tolcon = list_Consensus_Tolerators.begin(); it_tolcon != list_Consensus_Tolerators.end(); ++it_tolcon)
            m_cOutput << (*it_tolcon) << " ";
        for (int i = 0; i < u_num_epucks - list_Consensus_Tolerators.size(); ++i)
            m_cOutput << " -1 ";

        m_cOutput <<  "\t" << "Consensus_Attackers: ";
        for (std::list<unsigned>::iterator it_atkcon = list_Consensus_Attackers.begin(); it_atkcon != list_Consensus_Attackers.end(); ++it_atkcon)
            m_cOutput << (*it_atkcon) << " ";
        for (int i = 0; i < u_num_epucks - list_Consensus_Attackers.size(); ++i)
            m_cOutput << " -1 ";
        m_cOutput << std::endl;

        //if (observed_rob_id == 16)
        {
            if((list_Consensus_Attackers.size() == list_Consensus_Tolerators.size()) && (list_Consensus_Tolerators.size() == 0))
            {
                if (observed_rob_id == 15)
                    cController.GetLEDsPtr()->SetAllColors(CColor::YELLOW);
                else
                    cController.GetLEDsPtr()->SetAllColors(CColor::BLACK);
            }
            else if(list_Consensus_Attackers.size() > list_Consensus_Tolerators.size())
                cController.GetLEDsPtr()->SetAllColors(CColor::RED);
            else
                cController.GetLEDsPtr()->SetAllColors(CColor::GREEN);
        }
    }
#else
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("e-puck");
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it)
    {
        /* Get handle to e-puck entity and controller */
        CEPuckEntity& cEPuck = *any_cast<CEPuckEntity*>(it->second);
        CEPuckHomSwarm& cController = dynamic_cast<CEPuckHomSwarm&>(cEPuck.GetControllableEntity().GetController());

        unsigned observed_rob_id = cController.RobotIdStrToInt();
        unsigned observed_rob_fv = cController.GetRobotFeatureVector();

        std::list<unsigned> list_Voters_Tolerators, list_Voters_Attackers;
        list_Voters_Tolerators.clear(); list_Voters_Attackers.clear();


        for(CSpace::TMapPerType::iterator it_ob = m_cEpucks.begin(); it_ob != m_cEpucks.end(); ++it_ob)
        {
            CEPuckEntity& cEPuck_Observers = *any_cast<CEPuckEntity*>(it_ob->second);
            CEPuckHomSwarm& cController_Observers = dynamic_cast<CEPuckHomSwarm&>(cEPuck_Observers.GetControllableEntity().GetController());

            unsigned observers_rob_id = cController_Observers.RobotIdStrToInt();

            t_listVoteInformationRobots listVoteInformationRobots = cController_Observers.GetListVoteInfoOnRobotIds();

            for (t_listVoteInformationRobots::iterator it_vot = listVoteInformationRobots.begin(); it_vot != listVoteInformationRobots.end(); ++it_vot)
            {
                assert(it_vot->uVoterIds.size() == 1u); // only self-vote should be registered
                assert(it_vot->attackvote_count + it_vot->toleratevote_count == 1u); // only self-vote should be registered

                if(it_vot->uRobotId == observed_rob_id && it_vot->attackvote_count > 0)
                    list_Voters_Attackers.push_back(observers_rob_id);

                else if(it_vot->uRobotId == observed_rob_id && it_vot->toleratevote_count > 0)
                    list_Voters_Tolerators.push_back(observers_rob_id);
            }
        }

        m_cOutput << "Clock: " << GetSpace().GetSimulationClock() << "\t"
                  << "Id: " << observed_rob_id << "\t"
                  << "FV: " << observed_rob_fv << "\t";

        m_cOutput << "Voters_Tolerators: ";
        for (std::list<unsigned>::iterator it_tolcon = list_Voters_Tolerators.begin(); it_tolcon != list_Voters_Tolerators.end(); ++it_tolcon)
            m_cOutput << (*it_tolcon) << " ";
        for (int i = 0; i < u_num_epucks - list_Voters_Tolerators.size(); ++i)
            m_cOutput << " -1 ";

        m_cOutput <<  "\t" << "Voters_Attackers: ";
        for (std::list<unsigned>::iterator it_atkcon = list_Voters_Attackers.begin(); it_atkcon != list_Voters_Attackers.end(); ++it_atkcon)
            m_cOutput << (*it_atkcon) << " ";
        for (int i = 0; i < u_num_epucks - list_Voters_Attackers.size(); ++i)
            m_cOutput << " -1 ";
        m_cOutput << std::endl;
    }
#endif
}

/****************************************/
/****************************************/

void CHomSwarmLoopFunctions::Destroy()
{
   /* Close the file */
   m_cOutput.close();
}

/****************************************/
/****************************************/

CColor CHomSwarmLoopFunctions::GetFloorColor(const CVector2& c_position_on_plane) // used to paint the floor by the floor entity
{
   return CColor::WHITE;
}

/****************************************/
/****************************************/

REGISTER_LOOP_FUNCTIONS(CHomSwarmLoopFunctions, "homswarm_loop_functions")
