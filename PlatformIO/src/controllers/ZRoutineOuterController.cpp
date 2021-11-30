
#include "zippies/controllers/ZRoutineOuterController.h"

ZRoutineOuterController::ZRoutineOuterController(SensorFusor* s, ZippyController* ic)
  : sensors(s),
    innerController(ic)
{

}