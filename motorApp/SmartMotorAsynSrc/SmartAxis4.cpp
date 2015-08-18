/*
FILENAME... SmartMotorDriver.cpp
USAGE...    Motor driver support for the Amimatics SmartMotor series of integrated motor/controllers.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "SmartController.h"
#include <epicsExport.h>


/** System 16-bit status word CLASS 4 SmartMotor */
#define MOVING	          0x0001
#define HIST_POS_LIM      0x0002
#define HIST_NEG_LIM      0x0004
#define HIST_INDEX        0x0008
#define POS_WRAP          0x0010
#define POS_ERROR         0x0020
#define TEMP_ERROR        0x0040
#define MOFF		  0x0080
#define INDEX             0x0100
#define POS_LIM	          0x0200
#define NEG_LIM	          0x0400
#define OVERFLOW          0x0800
#define ARRAY_INDEX_ERROR 0x1000
#define SYNTAX_ERROR      0x2000
#define OVERCURRENT       0x4000
#define EEPROM_ERROR      0x8000
#define DEBUG 1
#undef DEBUG
static const char *driverName = "SmartAxis4Driver";


// These are the SmartAxis4 methods

/** Creates a new SmartAxis4 object.
  * \param[in] pC Pointer to the SmartController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
SmartAxis4::SmartAxis4(SmartController *pC, int axisNo)
  : SmartAxisBase(pC, axisNo)
{
  asynStatus status = asynSuccess;
  axisNum_ = Smart_addr[axisNo];
  printf("axis NUM %d, %c", axisNum_, axisNum_);
  sampleRate_ = 4096.0;  //sample rate for class 4 motors
  /* velocity units are 65536/sample rate */
  velConst_ = 65536.0/sampleRate_;
  /* acclerations units are 65536/(sample*sample)  */
  aclConst_ = 65536.0/(sampleRate_*sampleRate_);
#ifdef DEBUG
  printf("Sample rate = %f\n", sampleRate_);
  printf("Velocity constant = %f\n", velConst_);
  printf("Accleration constant = %f\n", aclConst_);
#endif
  
  getPID();
  setClosedLoop(true);

  skip:
// assume servo motor with encoder
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);

  callParamCallbacks();
}

asynStatus SmartAxis4::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveAxis";
  
  status = resetFlags();
  if (status) goto skip;
  sprintf(pC_->outString_, "%cMP", axisNum_); // Set position mode
  status = pC_->writeController();
  if (status) goto skip;
  status = setAcceleration(acceleration);
  if (status) goto skip;
  status = setVelocity(minVelocity, maxVelocity);
  if (status) goto skip;
  if (relative) {
    sprintf(pC_->outString_, "%cD=%ld", axisNum_, (long)round(position)); // Set target relative position
    status = pC_->writeController();
    commandedPosition_ += (long)round(position);
    if (status) goto skip;
  } else {
    sprintf(pC_->outString_, "%cP=%ld", axisNum_, (long)round(position)); // Set target absolute position
    status = pC_->writeController();
    commandedPosition_ = (long)round(position);
    if (status) goto skip;
  }
  status = go();
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();  
  return status;
}

asynStatus SmartAxis4::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  // static const char *functionName = "homeAxis";
  //status = setPosition(0);
  //status = setAcceleration(acceleration);
  //status = setVelocity(minVelocity, maxVelocity);
  return status;
}

asynStatus SmartAxis4::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status;
  // static const char *functionName = "moveVelocityAxis";
  status = resetFlags();
  if(status) goto skip;
  sprintf(pC_->outString_, "%cMV", axisNum_); // Velocity mode
  status = pC_->writeController();
  if(status) goto skip;
  status = setAcceleration(acceleration);
  if(status) goto skip;
  status = setVelocity(minVelocity, maxVelocity);
  if(status) goto skip;
  status = go();
  
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::stop(double acceleration)
{
  asynStatus status;
  //static const char *functionName = "stopAxis";

  sprintf(pC_->outString_,"%cS", axisNum_); // stop without deceleration
//  sprintf(pC_->outString_,"%cADT=%f", axisNum_, acceleration); //set target decleration
//  sprintf(pC_->outString_,"%cX", axisNum_); // decelerate to stop
  status = pC_->writeController();
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setPosition(double position)
{
  asynStatus status;
  sprintf(pC_->outString_, "%cO=%ld", axisNum_, (long)round(position));
  status = pC_->writeController();
  commandedPosition_=position;
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setEncoderPosition(double position)
{
  return setPosition(position);
}

asynStatus SmartAxis4::setClosedLoop(bool closedLoop)
{
  asynStatus status;

  if(closedLoop){
/* Turn motor on, servo in current position */  
  sprintf(pC_->outString_,"RP");
  status = pC_->writeReadController();
  if (status) goto skip;
  commandedPosition_ = atof(pC_->inString_); //commanded position
  status = move(commandedPosition_,0,0,0,0);
  }
  else{
    sprintf(pC_->outString_,"%cOFF", axisNum_);  //enable software limits
    status = pC_->writeController();
  }

skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setPGain(double pGain)
{
  asynStatus status;
  int pval;
  pGain = pGain > 1 ? 1:pGain; // expecting value between 0 and 1 from motor record
  pGain = pGain < 0 ? 0:pGain;
  pval = (int) (pGain * 65535);
#ifdef DEBUG
  printf("Set P Gain:  \n");
  printf("%cKP=%d", axisNum_, pval);
#endif
  sprintf(pC_->outString_, "%cKP=%d", axisNum_, pval);
  status = pC_->writeController();
  if(status) goto skip;
  status = forcePIDUpdate();
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setIGain(double iGain)
{
  asynStatus status;
  int ival;
  iGain = iGain > 1 ? 1:iGain;
  iGain = iGain < 0 ? 0:iGain;
  ival = (int) (iGain * 65535);
#ifdef DEBUG
  printf("Set I Gain:  \n");
  printf("%cKI=%d", axisNum_, ival);
#endif
  sprintf(pC_->outString_, "%cKI=%d", axisNum_, ival);
  status = pC_->writeController();
  if(status) goto skip;
  status = forcePIDUpdate();
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setDGain(double dGain)
{
  asynStatus status;
  int dval;
  dGain = dGain > 1 ? 1:dGain;
  dGain = dGain < 0 ? 0:dGain;
  dval = (int) (dGain * 65535);
#ifdef DEBUG
  printf("Set D Gain:  \n");
  printf("%cKD=%d", axisNum_, dval);
#endif
  sprintf(pC_->outString_, "%cKD=%d", axisNum_, dval);
  status = pC_->writeController();
  if(status) goto skip;
  status = forcePIDUpdate();
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::forcePIDUpdate()
{
  asynStatus status;
  sprintf(pC_->outString_,"%cF", axisNum_); // F forces latest PID filter to update to latest values
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis4::getPID()
{
  asynStatus status;
  double kp, ki, kd;
  sprintf(pC_->outString_,"%cRKP", axisNum_); // F forces latest PID filter to update to latest values
  status = pC_->writeReadController();
  if (status) goto skip;
  kp = atof(pC_->inString_);
  setDoubleParam(pC_->motorPGain_,kp/32767.0);

  sprintf(pC_->outString_,"%cRKI", axisNum_); // F forces latest PID filter to update to latest values
  status = pC_->writeReadController();
  if (status) goto skip;
  ki = atof(pC_->inString_);
  setDoubleParam(pC_->motorIGain_,ki/32767.0);

  sprintf(pC_->outString_,"%cRKD", axisNum_); // F forces latest PID filter to update to latest values
  status = pC_->writeReadController();
  if (status) goto skip;
  kd = atof(pC_->inString_);
  setDoubleParam(pC_->motorDGain_,kd/32767.0);
  
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0);
  callParamCallbacks();
  return status;
}

asynStatus SmartAxis4::setVelocity(double minVelocity, double maxVelocity)
{
  asynStatus status;
  /* velocity units are 65536/sample rate */
  sprintf(pC_->outString_, "%cV=%ld", axisNum_, (long)round(maxVelocity*velConst_)); // Set target velocity
#ifdef DEBUG  
  printf("Set velocity:  V=%ld\n", (long)round(maxVelocity*aclConst_));
#endif
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis4::setAcceleration(double acceleration)
{
  asynStatus status;
  /* acclerations units are 65536/(sample*sample) */
  sprintf(pC_->outString_, "%cA=%ld", axisNum_, (long)round(acceleration*aclConst_)); // Set target acceleration
#ifdef DEBUG
  printf("Set acceleration:  A=%ld\n", (long)round(acceleration*aclConst_));
#endif
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis4::go()
{
#ifdef DEBUG
  printf("GO\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cG", axisNum_); // GO
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxis4::resetFlags()
{
#ifdef DEBUG
  printf("Reset flags\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_,"%cZS", axisNum_); //
  status = pC_->writeController();
  return status;
}



/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus SmartAxis4::poll(bool *moving)
{ 
  int done;
  int driveOn;
  int limit;
  int followingError;
  int motorError;
  asynStatus status;
  // Read the current encoder position
  sprintf(pC_->outString_, "%cRP", axisNum_);
  status = pC_->writeReadController();
  if (status) goto skip;
  encoderPosition_ = atof(pC_->inString_);
  setDoubleParam(pC_->motorEncoderPosition_,encoderPosition_);
  enc0_ = encoderPosition_;
  setDoubleParam(pC_->smartCtr0_,enc0_);
  

  // Read the current theoretical position
  sprintf(pC_->outString_, "%cRP", axisNum_); //x=P\r Rx
  status = pC_->writeReadController();
/*
  sprintf(pC_->outString_, "%cx=P", axisNum_);
  status = pC_->writeController();
  sprintf(pC_->outString_,"Rx");
  status = pC_->writeReadController();
*/
  if (status) goto skip;
  //commandedPosition_ = atof(pC_->inString_); //commanded position
  //setDoubleParam(pC_->motorPosition_, commandedPosition_);
  setDoubleParam(pC_->motorPosition_, encoderPosition_);
  
// get smart motor temp
  sprintf(pC_->outString_, "PRINT(TEMP,#13)");
  status = pC_->writeReadController();
  if (status) goto skip;
  temp_ = atof(pC_->inString_);
  setDoubleParam(pC_->smartMotorTemp_,temp_);

// get smart motor torque
  sprintf(pC_->outString_, "%cRT", axisNum_);
  status = pC_->writeReadController();
  if (status) goto skip;
  torque_ = atof(pC_->inString_);
  setDoubleParam(pC_->smartMotorTorque_,torque_);

  // Read the drive 16-bit status word
  sprintf(pC_->outString_, "%cRW", axisNum_); // Report 16-bit status word
  status = pC_->writeReadController();
  if (status) goto skip;
  currentStatus_ = atoi(pC_->inString_);
  done = (currentStatus_ & MOVING)?0:1; // Trajectory in progress
  setIntegerParam(pC_->motorStatusDone_, done);
  *moving = done ? false:true;
  limit = (currentStatus_ & POS_LIM)?1:0; // Real time positive limit active
  setIntegerParam(pC_->motorStatusHighLimit_, limit);
  limit = (currentStatus_ & NEG_LIM)?1:0; // Real time negative limit switch
  setIntegerParam(pC_->motorStatusLowLimit_, limit);
//  limit = (currentStatus_ & 0x300)?1:0;
//  setIntegerParam(pC_->motorStatusAtHome_, limit);
  driveOn = (currentStatus_ & MOFF) ? 0:1; //Motor OFF bit
  setIntegerParam(pC_->motorStatusPowerOn_, driveOn);
  followingError = (currentStatus_ & POS_ERROR) ? 1:0; //Check for following error
  setIntegerParam(pC_->motorStatusFollowingError_,followingError);
//check for any other error
  motorError = (currentStatus_ & ( TEMP_ERROR | OVERFLOW | ARRAY_INDEX_ERROR | SYNTAX_ERROR | OVERCURRENT | EEPROM_ERROR));
  setIntegerParam(pC_->motorStatusProblem_, motorError ? 1:0);
//  if(motorError){
//    handleError(currentStatus_);
//  }
//  else{
//      pC_->setStringParam(axisNum_, pC_->smartError_, "No Error");
//  }
  sprintf(pC_->outString_, "%cRCTR", axisNum_); // Report actual position
  status = pC_->writeReadController();
  if (status) goto skip;
  enc1_ = atof(pC_->inString_);
  setDoubleParam(pC_->smartCtr1_,enc1_);
  skip:
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1:0); 
  callParamCallbacks();
  return status;
}
/*
asynStatus SmartAxis4::handleError(int status){
  static int lastStatus = 0;
  static const char *functionName = "handleError()";
  asynStatus s = asynSuccess;
  if (status == lastStatus){
  }
  else {
    lastStatus = status;
    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: New Errors:\n", driverName, functionName);
    if(status & TEMP_ERROR){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Temperature Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Temerature Error");
    }
    if(status & OVERFLOW){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Overflow Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Overflow Error");
    }
    if(status & ARRAY_INDEX_ERROR){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Array Index Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Array Index Error");
    }
    if(status & SYNTAX_ERROR){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Syntax Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Syntax Error");
    }
    if(status & OVERCURRENT){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Overcurrent Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Overcurrent Error");
    }
    if(status & EEPROM_ERROR){
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: EEPROM Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "EEPROM Error");
    }
    else{
      asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: Other Error\n", driverName, functionName);
      pC_->setStringParam(axisNum_, pC_->smartError_, "Other Error");
    }
  }
  return s;
}*/
