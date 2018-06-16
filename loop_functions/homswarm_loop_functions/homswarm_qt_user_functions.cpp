#include "homswarm_qt_user_functions.h"
#include <controllers/epuck_hom_swarm/epuck_hom_swarm.h>
#include <argos3/core/simulator/entity/controllable_entity.h>

using namespace argos;

/****************************************/
/****************************************/

CHomSwarmQTUserFunctions::CHomSwarmQTUserFunctions()
{
   RegisterUserFunction<CHomSwarmQTUserFunctions,CEPuckEntity>(&CHomSwarmQTUserFunctions::Draw);
}

/****************************************/
/****************************************/

void CHomSwarmQTUserFunctions::Draw(CEPuckEntity& c_entity)
{
//   CEPuckForaging& cController = dynamic_cast<CEPuckForaging&>(c_entity.GetControllableEntity().GetController());
//   CEPuckForaging::SFoodData& sFoodData = cController.GetFoodData();
//   if(sFoodData.HasFoodItem)
//   {
//      DrawCylinder(
//         CVector3(0.0f, 0.0f, 0.3f),
//         CQuaternion(),
//         0.01f,
//         0.05f,
//         CColor::BLACK);
//   }
}

/****************************************/
/****************************************/

REGISTER_QTOPENGL_USER_FUNCTIONS(CHomSwarmQTUserFunctions, "homswarm_qt_user_functions")
