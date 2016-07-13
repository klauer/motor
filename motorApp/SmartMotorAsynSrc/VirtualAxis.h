/*
VirtualAxis.h
Virtual Axis
Represents parameters such as GAP,PITCH,TAPER for a set of motors.

Supports
    -processdeferedmoves
    -stop
    -poll
*/
#ifndef VIRTUAL_AXIS
#define VIRTUAL_AXIS

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "SmartController.h"
#include "VirtualAxis.h"
#include "SmartAxisBase.h"

class VirtualAxis : public SmartAxisBase {
 public:
  /* These are the methods we override from the base class */
  VirtualAxis(class SmartController *pC, int axis);
  asynStatus processDeferredMoves();
  asynStatus storeVelocity(double velocity);
  asynStatus storeAcceleration(double acceleration);
  asynStatus resetFlags();
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);

 protected:
  friend class SmartController;
};

#endif
