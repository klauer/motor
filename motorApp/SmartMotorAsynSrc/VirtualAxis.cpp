/*
VirtualAxis.cpp
Virtual Axis Driver
Drives a set of motors using a meta-parameter such as GAP,TAPER,PITCH.
Author: Jacob DeFilippis
June 30 2016
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <map>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SmartController.h"

#define MOVEMASK         0xF
#define WATCHDOGMASK     0x1
#define SYNCMASK         0x2
#define NUM_SUBROUTINES  3
#define MAX_VELO_ST_LOC "vvv"
#define ACCL_ST_LOC "aaa"

#define DEBUG 1
#undef DEBUG

static const char *driverName = "VirtualAxisDriver";

/**Constructor
*  Derives from SmartAxisBase
*  Fields set baseAxis_
*      This is the base address for serial communication, the virtual axis
*      will commnicate with the master motor at the base address
**/
VirtualAxis::VirtualAxis(SmartController *pC, int axisNo)
    : SmartAxisBase(pC, axisNo) {
/*empty constructor*/
#ifdef DEBUG
  printf("VirtualAxis::VirtualAxis(%p,%d)\n", pC, axisNo);
#endif
  asynStatus status = asynError;
  axisNo_ = axisNo;
  canAddr_ = axisNo + 1;

  sprintf(pC_->outString_, "RSAMP");
  status = pC_->writeReadController();
  if (status) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Axis %d sample rate not reported, default to 8000\n",
              driverName, axisNo);
    setIntegerParam(pC_->motorStatusProblem_, 1);
  } else {
    sampleRate_ = atof(pC_->inString_);
  }
/*  status = pC_->writeReadController(); */
#ifdef DEBUG
  printf("axisNO = %d  sampleRate= %f, command return = %s", axisNo,
         sampleRate_, pC_->inString_);
#endif
  /* velocity units are 65536/sample rate */
  velConst_ = 65536.0 / sampleRate_;
  /* acclerations units are 65536/(sample*sample)  */
  aclConst_ = 65536.0 / (sampleRate_ * sampleRate_);
}

/**VirtualAxis::processDeferredMoves()
*  Writes interupt to motor controller. The interupt
*  signals the subroutine which sychronizes motors on CAN
*  network
**/
asynStatus VirtualAxis::processDeferredMoves() {
  asynStatus status;
  int virtualAxisIndex = axisNo_ - (pC_->numAxes_ - pC_->numVirtualAxes_) + 1;
  double vaVelocity;
  double vaAcceleration;
#ifdef DEBUG
  printf("VirtualAxis::processDeferredMoves(), coord status = %d\n",
         syncStatus_);
#endif

  /*Store the virtual axis's velocity and acceleration before move*/
  pC_->getDoubleParam(axisNo_, pC_->motorVelocity_, &vaVelocity);
  pC_->getDoubleParam(axisNo_, pC_->motorAccel_, &vaAcceleration);
  status = storeVelocity(vaVelocity);
  if(status) goto skip;
  status = storeAcceleration(vaAcceleration);
  if(status) goto skip;

  /*Which virtual axis is it, index ranges from 1 to  NUM_SUBROUTINES*/
  if (virtualAxisIndex > NUM_SUBROUTINES || virtualAxisIndex < 1) {
    asynPrint(
        pC_->pasynUserSelf, ASYN_TRACE_ERROR,
        "VirtualAxis::processDeferredMoves Port doesn't map to subroutine \n");
    goto skip;
  }

  status = resetFlags();
  if (status) goto skip;
  sprintf(pC_->outString_, GOSYNC, virtualAxisIndex);
  status = pC_->writeController();

skip:
  setIntegerParam(pC_->motorStatusDone_, 0);
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  pC_->wakeupPoller();
  return asynSuccess;
}

asynStatus VirtualAxis::resetFlags() {
  asynStatus status = asynSuccess;
  sprintf(pC_->outString_, CLEARSYNCFLAG);
  status = pC_->writeController();
  if (status) goto skip;
  sprintf(pC_->outString_, CLEARFAULTS,0);
  status = pC_->writeController();

skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

/**VirtualAxis::stop(double acceleration)
*  Stops sychronized motion, by issuing command for
*  all motors on CAN network
*  \param[in] acceleration defines the deacceleration of the motor
**/
asynStatus VirtualAxis::stop(double acceleration) {
#ifdef DEBUG
  printf("VirtualAxis::stop(%f)\n", acceleration);
#endif
  asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Stop issued\n");
  asynStatus status;
  sprintf(pC_->outString_, STOP);  // S:0 stops all motors on CAN network
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus VirtualAxis::storeVelocity(double velocity) {
  asynStatus status = asynSuccess;
  /* velocity units are 65536/sample rate */
  epicsInt32 velocityInt32 =
      static_cast<epicsInt32>(round(velocity * velConst_));

#ifdef DEBUG
  printf("STORE VELOCITY  ");
  printf("velConst_ = %f\n", velConst_);
#endif
  status = pC_->setControllerMemory(MAX_VELO_ST_LOC,
                               velocityInt32);  // Set target velocity
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus VirtualAxis::storeAcceleration(double acceleration) {
  asynStatus status = asynSuccess;
  acceleration = (acceleration < 1) ? 1 : acceleration;
  /* acclerations units are 65536/(sample*sample) */
  epicsInt32 accelerationInt32 =
      static_cast<epicsInt32>(round(acceleration * aclConst_));
#ifdef DEBUG
  printf("STORE ACCELERATION  ");
  printf("acceleration= %d\n", accelerationInt32);
#endif
  status = pC_->setControllerMemory(ACCL_ST_LOC,
                               accelerationInt32);  // Set target acceleration
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

/** VirtualAxis::poll(bool *moving)
*     Polls the axis for information on whether the
*     motor is still completing it's move
*     \param[in] moving storage for flag indicating movement in progress
*/
asynStatus VirtualAxis::poll(bool *moving) {
  asynStatus status;
  int done;
  int numRealAxes = pC_->numAxes_ - pC_->numVirtualAxes_;

  // Read the drive 16-bit status word 7
  sprintf(pC_->outString_, REPORTTRAJECTORY);  // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) goto skip;
  // Is it moving?
  currentStatus_ = atoi(pC_->inString_);
  setIntegerParam(pC_->smartStatus_, currentStatus_);
  done = (currentStatus_ & MOVEMASK) ? 0 : 1;  // Trajectory in progress
  *moving = done ? false : true;
  setIntegerParam(pC_->motorStatusDone_, done);

#ifdef DEBUG
  printf("VirtualAxis::poll() moving = %d\n", *moving);
#endif

  // Read sychronous trajectory and watchdog status'
  syncStatus_ = 0;
  watchDogStatus_ = 0;
  for (int addr = 1; addr <= numRealAxes; addr++) {
    sprintf(pC_->outString_, REPORTSTATUS12, addr);
    status = pC_->writeReadController();
    if (status) break;
    /*Places a bit for each axis sync flag that is asserted*/
    syncStatus_ += (atoi(pC_->inString_) & SYNCMASK) << (addr - 1);
    watchDogStatus_ += (atoi(pC_->inString_) & WATCHDOGMASK) << (addr - 1);
  }

  setIntegerParam(pC_->syncStatus_, syncStatus_);
  setIntegerParam(pC_->watchDogStatus_, watchDogStatus_);

skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}
