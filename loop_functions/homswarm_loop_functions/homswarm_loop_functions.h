#ifndef HOMSWARM_LOOP_FUNCTIONS_H
#define HOMSWARM_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>


using namespace argos;

class CHomSwarmLoopFunctions : public CLoopFunctions {

public:

   CHomSwarmLoopFunctions();
   virtual ~CHomSwarmLoopFunctions() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual CColor GetFloorColor(const CVector2& c_position_on_plane);
   virtual void PreStep();
   virtual void PostStep();

private:

   CFloorEntity* m_pcFloor;
   CRandom::CRNG* m_pcRNG;

   std::string m_strOutput;
   std::ofstream m_cOutput;

   unsigned u_num_epucks;

   Real fArenaLength;


//   unsigned int     m_unCoordCurrQueueIndex, CurrentStepNumber;
//    int              m_iDistTravelledTimeWindow;
//   argos::CVector2  *m_pvecCoordAtTimeStep;

};

#endif
