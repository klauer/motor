/*
:FILENAME... SmartMotorDriver.cpp
USAGE...    Motor driver support for the Amimatics SmartMotor series of
integrated motor/controllers.

Mark Rivers
March 4, 2011

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>

#include "SmartController.h"

/** Polling divisor **/
#define POLL_DIVISOR 50
/** System 16-bit status word CLASS 5 SmartMotor */
#define DRIVE_READY 0x1
#define MOFF 0x2
#define MOVING 0x4
#define VOLT_ERROR 0x8
#define OVERCURRENT 0x10
/* TEMP_MINOR - excessive temperature real time */
#define TEMP_MINOR 0x20
#define POS_ERROR 0x40
#define VEL_ERROR 0x80
/* TEMP_MAJOR - real-time temperature limit */
#define TEMP_MAJOR 0x100
#define DERIV_ERROR 0x200
#define POS_LIM_EN 0x400
#define NEG_LIM_EN 0x800
#define HIST_POS_LIM 0x1000
#define HIST_NEG_LIM 0x2000
#define POS_LIM 0x4000
#define NEG_LIM 0x8000
#define DEBUG 1
#undef DEBUG
/*Storage Variables for deferred move*/
#define POS_ST_LOC1 "iii"
#define POS_ST_LOC2 "jjj"
#define POS_ST_LOC3 "kkk"
#define POS_ST_LOC4 "lll"

static const char *driverName = "SmartAxis5Driver";

/** Creates a new SmartAxis object.
  * \param[in] pC Pointer to the SmartController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
SmartAxis5::SmartAxis5(SmartController *pC, int axisNo)
    : SmartAxisBase(pC, axisNo) {
  asynStatus status;
  axisNo_ = axisNo;
  /*Can addresses (1..n), axis numbers (0...m)*/
  canAddr_ = axisNo + 1;
  pollPeriodDivisor_ = POLL_DIVISOR;
  pollCounter_ = 0;
  sprintf(pC_->outString_, SAMPLERATE, canAddr_);
  status = pC_->writeReadController();
  if (status) {
    sampleRate_ = 8000.0;  // assume default sample rate
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Axis %d sample rate not reported, default to 8000\n",
              driverName, axisNo);
    setIntegerParam(pC_->motorStatusProblem_, 1);
  } else {
    sampleRate_ = atof(pC_->inString_);
  }
  status = pC_->writeReadController();
#ifdef DEBUG
  printf("axisNO = %d  sampleRate= %f, command return = %s", axisNo,
         sampleRate_, pC_->inString_);
#endif
  /* velocity units are 65536/sample rate */
  velConst_ = 65536.0 / sampleRate_;
  /* acclerations units are 65536/(sample*sample)  */
  aclConst_ = 65536.0 / (sampleRate_ * sampleRate_);
  setClosedLoop(true);

  setIntegerParam(pC_->motorStatusGainSupport_, 1);
  setIntegerParam(pC_->motorStatusHasEncoder_, 1);
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
}

asynStatus SmartAxis5::move(double position, int relative, double minVelocity,
                            double maxVelocity, double acceleration) {
  asynStatus status = asynSuccess;

#ifdef DEBUG
  printf(
      "Move : position=%ld, relative=%d, minVelocity=%f, maxVelocity=%f, "
      "acceleration=%f \n",
      (long)round(position), relative, minVelocity, maxVelocity, acceleration);
  printf("sync flag = %d\n", pC_->syncMode_);
#endif

  if (pC_->syncMode_) {
#ifdef DEBUG
    printf("%s\n", "Move stored");
#endif
    if (relative) {
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Cannot defer relative motion\n");
      goto skip;
    }
    if (status) goto skip;
    status = storePosition(position);
    if (status) goto skip;

  } else {
    status = resetFlags();
    if (status) goto skip;
    sprintf(pC_->outString_, MODEPOSITION, canAddr_);  // Set position mode
    status = pC_->writeController();
    if (status) goto skip;
    status = setAcceleration(acceleration);
    if (status) goto skip;
    status = setVelocity(minVelocity, maxVelocity);
    if (status) goto skip;
    if (relative) {
      sprintf(pC_->outString_, SETRELATIVETARGET, canAddr_,
              (long)round(position));  // Set target relative position
      status = pC_->writeController();
      if (status) goto skip;
    } else {
      sprintf(pC_->outString_, SETTARGET, canAddr_,
              (long)round(position));  // Set target absolute position
      status = pC_->writeController();
      if (status) goto skip;
    }
    status = go();
  }
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  if (!pC_->syncMode_) pC_->wakeupPoller();
  return status;
}

asynStatus SmartAxis5::home(double minVelocity, double maxVelocity,
                            double acceleration, int forwards) {
  asynStatus status = asynSuccess;

  // set velocity use slower settings - this will be in animatics variable
  // assumes default program on smartmotor, homing routine after RUN?
  sprintf(pC_->outString_, STOREVELO, canAddr_, round(maxVelocity * velConst_));
  status = pC_->writeController();
  if (status) goto skip;
  // set acceleration
  status = setAcceleration(acceleration);
  if (status) goto skip;
  // assumes default program on smartmotor, homing routine after RUN?
  sprintf(pC_->outString_, HOME, canAddr_);
  status = pC_->writeController();
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();

  return status;
}

asynStatus SmartAxis5::moveVelocity(double minVelocity, double maxVelocity,
                                    double acceleration) {
  asynStatus status = asynError;

#ifdef DEBUG
  printf("minVelocity=%f, maxVelocity=%f, acceleration=%f \n", minVelocity,
         maxVelocity, acceleration);
  printf("sync mode flag = %d\n", pC_->syncMode_);
#endif

  if (pC_->syncMode_) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Cannot jog in sync mode\n");

  } else {
    status = resetFlags();
    if (status) goto skip;
    sprintf(pC_->outString_, MODEVELOCITY, canAddr_);  // Velocity mode
    status = pC_->writeController();
    if (status) goto skip;
    status = setAcceleration(acceleration);
    if (status) goto skip;
    status = setVelocity(minVelocity, maxVelocity);
    if (status) goto skip;
    status = go();
  }
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::stop(double acceleration) {
#ifdef DEBUG
  printf("STOP\n");
#endif

  asynStatus status;
  if (!pC_->syncMode_) {
    sprintf(pC_->outString_, STOPAXIS, canAddr_);  // stop without deceleration
    status = pC_->writeController();
    if (status) goto skip;
    status = setPosition(encoderPosition_);
    if (status) goto skip;
    status = setEncoderPosition(encoderPosition_);
  } else {
    // If in sync mode moves do a global stop
    sprintf(pC_->outString_, "%cS:0", canAddr_);  // stop without deceleration
    status = pC_->writeController();
    if (status) goto skip;
    status = setPosition(encoderPosition_);
    if (status) goto skip;
    status = setEncoderPosition(encoderPosition_);
  }
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setPosition(double position) {
#ifdef DEBUG
  printf("SET POSITION:  ");
  printf("Offset = %ld", (long)round(position));
#endif
  asynStatus status;
  sprintf(pC_->outString_, OFFSET, canAddr_, (long)round(position));
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setEncoderPosition(double position) {
  return setPosition(position);
}

asynStatus SmartAxis5::setClosedLoop(bool closedLoop) {
  asynStatus status;
  printf("SmartAxis5::setClosedLoop, closedLoop=%d\n", closedLoop);


  if (closedLoop) {
    /* set motor to ON, servo at current position */
    sprintf(pC_->outString_, REPORTPOSITION, canAddr_);
    status = pC_->writeReadController();
    if (status) goto skip;
    commandedPosition_ = atof(pC_->inString_);
    /*Ensure holding current on motor by moving it*/
    status = move(commandedPosition_, 0, 1, 10, 10);
    if(status) goto skip;

    if(!pC_->syncMode_){
      sprintf(pC_->outString_, BRAKERELEASE, canAddr_);
      status = pC_->writeController();
      if(status) goto skip;
    }

  } else {
    if(!pC_->syncMode_){
      sprintf(pC_->outString_, BRAKEENGAGE, canAddr_);
      status = pC_->writeController();
      if(status) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Brake did not engage!\n");
        goto skip;
      }
    }
    sprintf(pC_->outString_, TURNOFF, canAddr_);  // enable software limits
    status = pC_->writeController();
  }

skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setPGain(double pGain) {
  asynStatus status;
  int pval;
  pGain = pGain > 1
              ? 1
              : pGain;  // expecting value between 0 and 1 from motor record
  pGain = pGain < 0 ? 0 : pGain;
  pval = (int)(pGain * 65535);
#ifdef DEBUG
  printf("Set P Gain:\n");
  printf(SETPROPORTIONAL, canAddr_, pval);
#endif
  sprintf(pC_->outString_, SETPROPORTIONAL, canAddr_, pval);
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setIGain(double iGain) {
  asynStatus status;
  int ival;
  iGain = iGain > 1 ? 1 : iGain;
  iGain = iGain < 0 ? 0 : iGain;
  ival = (int)(iGain * 65535);
#ifdef DEBUG
  printf("Set I Gain:  \n");
  printf(SETINTEGRAL, canAddr_, ival);
#endif
  sprintf(pC_->outString_, SETINTEGRAL, canAddr_, ival);
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setDGain(double dGain) {
  asynStatus status;
  int dval;
  dGain = dGain > 1 ? 1 : dGain;
  dGain = dGain < 0 ? 0 : dGain;
  dval = (int)(dGain * 65535);
#ifdef DEBUG
  printf("Set D Gain:  \n");
  printf(SETDERIVATIVE, canAddr_, dval);
#endif
  sprintf(pC_->outString_, SETDERIVATIVE, canAddr_, dval);
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setHighLimit(double highLimit) {
#ifdef DEBUG
  printf("High limit %ld\n", (long)round(highLimit));
#endif
  asynStatus status;
  sprintf(pC_->outString_, SWLIMITENABLE, canAddr_);  // enable software limits
  status = pC_->writeController();
  if (status) goto skip;
  sprintf(pC_->outString_, SWLIMITMODE,
          canAddr_);  // software limit doesn't cause fault, allows move off
  status = pC_->writeController();
  if (status) goto skip;
  sprintf(pC_->outString_, SWLIMITPLUS, canAddr_, (long)round(highLimit));
  status = pC_->writeController();
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setLowLimit(double lowLimit) {
#ifdef DEBUG
  printf("Low limit %ld\n", (long)round(lowLimit));
#endif
  asynStatus status;
  sprintf(pC_->outString_, SWLIMITENABLE, canAddr_);  // enable software limits
  status = pC_->writeController();
  if (status) goto skip;
  sprintf(pC_->outString_, SWLIMITMODE,
          canAddr_);  // software limit doesn't cause fault, allows move off
  status = pC_->writeController();
  if (status) goto skip;
  sprintf(pC_->outString_, SWLIMITMINUS, canAddr_, (long)round(lowLimit));
  status = pC_->writeController();
skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::setVelocity(double minVelocity, double maxVelocity) {
  asynStatus status;
  /* velocity units are 65536/sample rate */
  sprintf(pC_->outString_, SETVELO, canAddr_,
          (long)round(maxVelocity * velConst_));  // Set target velocity
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::setAcceleration(double acceleration) {
  asynStatus status;
  acceleration = (acceleration < 1) ? 1 : acceleration;
  /* acclerations units are 65536/(sample*sample) */
  sprintf(pC_->outString_, SETACC, canAddr_,
          (long)round(acceleration * aclConst_));  // Set target acceleration
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::storePosition(double position) {
  asynStatus status = asynSuccess;
  epicsInt32 positionInt32 = static_cast<epicsInt32>(round(position));
#ifdef DEBUG
  printf("STORE POSITION:  ");
#endif
  switch (canAddr_) {
    case 1:
      status = pC_->setControllerMemory(POS_ST_LOC1, positionInt32);
      break;
    case 2:
      status = pC_->setControllerMemory(POS_ST_LOC2, positionInt32);
      break;
    case 3:
      status = pC_->setControllerMemory(POS_ST_LOC3, positionInt32);
      break;
    case 4:
      status = pC_->setControllerMemory(POS_ST_LOC4, positionInt32);
      break;
    default:
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
                "Store failure, axis number %d not available for store\n",
                axisNo_);
  }

  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::go() {
  asynStatus status;
  sprintf(pC_->outString_, GO, canAddr_);  // GO
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::resetFlags() {
#ifdef DEBUG
  printf("Reset flags\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_, CLEARFAULTS, canAddr_);
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis5::processDeferredMoves() {
#ifdef DEBUG
  printf("SmartAxis5::processDeferredMoves()\n");
#endif
  /*Empty process only called on VA*/
  return asynSuccess;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit
 * status, the moving status,
  * and the drive power-on status.  It does not current detect following error,
 * etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1)
 * or done (0). */
asynStatus SmartAxis5::poll(bool *moving) {
  asynStatus status;
  status = pollStatus(moving);
  if (status) goto skip;
  status = pollPosition();
  if (status) goto skip;
  /**To reduce resource consumption reduce polling for these parameters**/
  if (pollCounter_ < pollPeriodDivisor_) {
    pollCounter_ = 0;
    status = pollSmartParameters();
  } else {
    pollCounter_ += 1;
  }

skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis5::pollStatus(bool *moving) {
  int done;
  int driveOn;
  int limit;
  int followingError;
  int motorError;
  asynStatus status;

  // Read the drive 16-bit status word
  sprintf(pC_->outString_, REPORTSTATUS0,
          canAddr_);  // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) return status;
  currentStatus_ = atoi(pC_->inString_);
  setIntegerParam(pC_->smartStatus_, currentStatus_);
  done = (currentStatus_ & MOVING) ? 0 : 1;  // Trajectory in progress
  *moving = done ? false : true;
  setIntegerParam(pC_->motorStatusDone_, done);
  setIntegerParam(pC_->motorStatusMoving_,!done);
  limit =
      (currentStatus_ & POS_LIM) ? 1 : 0;  // Real time positive limit active
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit =
      (currentStatus_ & NEG_LIM) ? 1 : 0;  // Real time negative limit switch
  setIntegerParam(pC_->motorStatusLowLimit_, limit);

  driveOn = (currentStatus_ & MOFF) ? 0 : 1;  // Motor OFF bit
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  followingError =
      (currentStatus_ & POS_ERROR) ? 1 : 0;  // Check for following error
  setIntegerParam(pC_->motorStatusFollowingError_, followingError);
  motorError = (currentStatus_ & (VOLT_ERROR | OVERCURRENT | TEMP_MINOR |
                                  TEMP_MAJOR | VEL_ERROR | DERIV_ERROR));
  setIntegerParam(pC_->motorStatusProblem_, motorError ? 1 : 0);

  if (motorError) {
    handleError(motorError);
  } else {
    pC_->setStringParam(axisNo_, pC_->smartError_, "No Error");
  }
  return status;
}

asynStatus SmartAxis5::pollPosition() {
  asynStatus status = asynSuccess;

  // Read the current encoder position
  sprintf(pC_->outString_, REPORTPOSITION, canAddr_);  // Report actual position
  status = pC_->writeReadController();
  if (status) return status;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_, encoderPosition_);
  enc0_ = (int) encoderPosition_;
  setDoubleParam(pC_->smartCtr0_, enc0_);

  /* Read the absolute position
    Commanded position is the controllers calculated position of the motor but 
    does not account for the pid servoing of the motor*/
  sprintf(pC_->outString_, REPORTCMDEDPOS,
          canAddr_);  // Report commanded position
  status = pC_->writeReadController();
  if (status) return status;
  commandedPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorPosition_, commandedPosition_);
  return status;
}

asynStatus SmartAxis5::pollSmartParameters() {
  asynStatus status = asynSuccess;

  sprintf(pC_->outString_, REPORTTEMP);  // Report temp
  status = pC_->writeReadController();
  if (status) return status;
  temp_ = atof(pC_->inString_);
  setDoubleParam(pC_->smartMotorTemp_, temp_);

  sprintf(pC_->outString_, REPORTCURRENT,
          canAddr_);  // Report current being applied, in mA
  status = pC_->writeReadController();
  if (status) return status;
  torque_ = atof(pC_->inString_);
  setDoubleParam(pC_->smartMotorTorque_, torque_);

  sprintf(pC_->outString_, REPORTEXTENC, canAddr_);  // Report actual position
  status = pC_->writeReadController();
  if (status) return status;
  enc1_ = atoi(pC_->inString_);
  setDoubleParam(pC_->smartCtr1_, enc1_);

  // Read the drive 16-bit status word 2
  sprintf(pC_->outString_, REPORTSTATUS2, canAddr_);  // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) return status;
  statusWord2_ = atoi(pC_->inString_);
  setIntegerParam(pC_->statusWord2_, statusWord2_);

  // Read the drive 16-bit status word 3
  sprintf(pC_->outString_, REPORTSTATUS3, canAddr_);  // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) return status;
  statusWord3_ = atoi(pC_->inString_);
  setIntegerParam(pC_->statusWord3_, statusWord3_);

  return status;
}

asynStatus SmartAxis5::handleError(int error) {
  static const char *functionName = "handleError()";
  asynStatus status = asynSuccess;
  if (error & TEMP_MINOR) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Minor temperature error\n", driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Minor Temerature Error");
  } else if (error & TEMP_MAJOR) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Major temperature error\n", driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Major temerature error");
  } else if (error & VOLT_ERROR) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Voltage error\n",
              driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Voltage error");
  } else if (error & OVERCURRENT) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Over current error\n", driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Over current error");
  } else if (error & VEL_ERROR) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Velocity error\n",
              driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Velocity error");
  } else if (error & DERIV_ERROR) {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Derivative error\n",
              driverName, functionName);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Overcurrent Error");
  } else {
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Other Error - status: %d\n", driverName, functionName,
              status);
    pC_->setStringParam(axisNo_, pC_->smartError_, "Other Error");
  }

  return status;
}
