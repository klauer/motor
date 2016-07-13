/*
FILENAME... SmartMotorDriver.cpp
USAGE...    Motor driver support for the Amimatics SmartMotor series of
integrated motor/controllers.

Mark Rivers
March 4, 2011

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <typeinfo>
#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>

#include "SmartController.h"
#include <epicsExport.h>

#define inputEos "\r"
#define outputEos " "
#define ANIMATICS_SMC5 1
#define DEBUG 1
#undef DEBUG

static const char *driverName = "SmartController";

/** Creates a new SmartController object.
  * \param[in] portName          The name of the asyn port that will be created
 * for this driver
  * \param[in] SmartPortName     The name of the drvAsynIPPPort that was created
 * previously to connect to the Smart controller
  * \param[in] numAxes           The number of axes that this controller
 * supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
SmartController::SmartController(const char *portName,
                                 const char *SmartPortName, int numAxes,
                                 int numVirtualAxes, double movingPollPeriod,
                                 double idlePollPeriod)
    : asynMotorController(portName, numAxes + numVirtualAxes + 1,
                          NUM_SMART_PARAMS, asynUInt32DigitalMask,
                          asynUInt32DigitalMask,
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,     // autoconnect
                          0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  SmartAxisBase *pAxis;
  static const char *functionName = "SmartController";
  int fwMajor;
  char fw[20];

  createParam(smartTempString, asynParamFloat64, &smartMotorTemp_);
  createParam(smartTorqueString, asynParamFloat64, &smartMotorTorque_);
  createParam(smartErrorString, asynParamOctet, &smartError_);
  createParam(smartFirmwareString, asynParamOctet, &smartFirmwareVersion_);
  createParam(smartStatusString, asynParamInt32, &smartStatus_);
  createParam(smartCTR0String, asynParamFloat64, &smartCtr0_);
  createParam(smartCTR1String, asynParamFloat64, &smartCtr1_);
  createParam(syncStatusString, asynParamInt32, &syncStatus_);
  createParam(watchDogStatusString, asynParamInt32, &watchDogStatus_);
  createParam(statusWord2String, asynParamInt32, &statusWord2_);
  createParam(statusWord3String, asynParamInt32, &statusWord3_);
  createParam(motorSyncModeString, asynParamInt32, &motorSyncMode_);

  /* Connect to Smart controller */
  status =
      pasynOctetSyncIO->connect(SmartPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: cannot connect to Smart controller\n", driverName,
              functionName);
  }

  pasynOctetSyncIO->setOutputEos(pasynUserController_, outputEos,
                                 strlen(outputEos));
  pasynOctetSyncIO->setInputEos(pasynUserController_, inputEos,
                                strlen(inputEos));
  // Wait a short while so that any responses to the above commands have time to
  // arrive so we can flush
  // them in the next writeReadController()
  epicsThreadSleep(0.5);
  pasynOctetSyncIO->flush(pasynUserController_);

  // Create the axis objects
  numAxes_ = numAxes + numVirtualAxes;
  if (numAxes_ < 1) {
    numAxes_ = 1;
  }
  numVirtualAxes_ = numVirtualAxes;
  /*  Check FW Version "connect" to axis */
  status = getFW(fwMajor, fw);
  for (axis = 0; axis < numAxes; axis++) {
    if (status) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s cannot connect to axis %d\n", driverName, functionName,
                axis);
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s Default to class 5 on axis %d\n", driverName,
                functionName, axis);
    }
    if (fwMajor == 4) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s NEW Class 4 Motor\n", driverName, functionName);
      pAxis = new SmartAxis4(this, axis);
    } else {  // default to class 5 smart motor...
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
                "%s:%s NEW Class 5 Motor\n", driverName, functionName);
      pAxis = new SmartAxis5(this, axis);
    }
    setStringParam(axis, smartFirmwareVersion_, fw);
  }
  createVirtualAxes(this, numAxes, numVirtualAxes);
#if DEBUG
  printf("moving poll= %f, idle poll %f\n", movingPollPeriod, idlePollPeriod);
#endif
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

asynStatus SmartController::getFW(int &fwMajor, char *fw) {
  asynStatus status;
  static const char *functionName = "getFW";
  fwMajor = 0;
  char *pst, *tokSave;
  char fwMajorc[1];

  sprintf(outString_, "%cRSP",Smart_addr[0]);
  status = writeReadController();  // should get echo and response here
  sprintf(fw, inString_);
  if (status == asynSuccess) {
    pst =
        epicsStrtok_r(inString_, "/",
                      &tokSave);  // response is clock/firmware (ex 23576/440C)
    pst = epicsStrtok_r(NULL, "/", &tokSave);
    strncpy(fwMajorc, pst, 1);  // get 1st character - FW major revision
    fwMajor = atoi(fwMajorc);
  } else {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s cannot get FW major\n",
              driverName, functionName);
  }
  return status;
}

/**SmartController::createVirtualAxes(SmartController* pC, int numAxes, int
*numVirtualAxes)
*   Takes in the number virtual axes selected by the user, and creates VA
*objects which
*   are attached to the SmartController object
*   \param[in] pC pointer to the smart controller which own the virtual axis
*   \param[in] numAxes number of regular axes the controller owns
*   \param[in] numVirtualAxes the number of virtual axes to be created
**/
asynStatus SmartController::createVirtualAxes(SmartController *pC, int numAxes,
                                              int numVirtualAxes) {
  VirtualAxis *pAxis;
  int totalAxes = numAxes + numVirtualAxes;
  if (pC) {
    for (int axis = numAxes; axis < totalAxes; axis++) {
      pAxis = new VirtualAxis(pC, axis);
    }
  } else {
    return asynError;
  }
  return asynSuccess;
}

/** Creates a new SmartController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created
 * for this driver
  * \param[in] SmartPortName       The name of the drvAsynIPPPort that was
 * created previously to connect to the Smart controller
  * \param[in] numAxes           The number of axes that this controller
 * supports
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is
 * moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is
 * moving
  */
extern "C" int SmartCreateController(const char *portName,
                                     const char *SmartPortName, int numAxes,
                                     int numVirtualAxes,
                                     double movingPollPeriod,
                                     double idlePollPeriod) {
  SmartController *pSmartController =
      new SmartController(portName, SmartPortName, numAxes, numVirtualAxes,
                          movingPollPeriod / 1000., idlePollPeriod / 1000.);
  pSmartController = NULL;
  return (asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls
 * asynMotorController::report()
  */
void SmartController::report(FILE *fp, int level) {
  fprintf(fp,
          "Smart motor driver %s, numAxes=%d, moving poll period=%f, idle poll "
          "period=%f\n",
          this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
  }

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SmartMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number.
 */
SmartAxisBase *SmartController::getAxis(asynUser *pasynUser) {
  return static_cast<SmartAxisBase *>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SmartMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SmartAxisBase *SmartController::getAxis(int axisNo) {
  return static_cast<SmartAxisBase *>(asynMotorController::getAxis(axisNo));
}

/** Called when asyn clients call pasynInt32->write().
  * Calls base asynMotorController::writeInt32, setDefferedMoves is overidden in
 * the class
  * writeInt32 must also be override to ensure pasynUser can be given to new
  * setDefferedMoves without disrupting base function
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus SmartController::writeInt32(asynUser *pasynUser, epicsInt32 value) {
  if (pasynUser->reason == motorDeferMoves_) {
    return setDeferredMoves(value, pasynUser);
  } else if (pasynUser->reason == motorSyncMode_) {
    asynStatus status;
    syncMode_ = (bool)value;
    status = setIntegerParam(motorSyncMode_, syncMode_);
    callParamCallbacks();
    return status;
  } else {
    return asynMotorController::writeInt32(pasynUser, value);
  }
}

asynStatus SmartController::writeFloat64(asynUser *pasynUser,
                                         epicsFloat64 value) {
  int function = pasynUser->reason;

  if (function == motorMoveAbs_) {
    double baseVelocity, velocity, acceleration;
    SmartAxisBase *pAxis;
    int axis;
    asynStatus status = asynError;

    pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    axis = pAxis->axisNo_;

    /* Set the parameter and readback in the parameter library. */
    status = pAxis->setDoubleParam(function, value);
    getDoubleParam(axis, motorVelBase_, &baseVelocity);
    getDoubleParam(axis, motorVelocity_, &velocity);
    getDoubleParam(axis, motorAccel_, &acceleration);

    /*Move it*/
    status = pAxis->move(value, 0, baseVelocity, velocity, acceleration);

    /*If sync mode don't set dmov or poller*/
    if (!syncMode_) {
      pAxis->setIntegerParam(motorStatusDone_, 0);
      wakeupPoller();
    }

    callParamCallbacks();
    return asynSuccess;
  } else {
    return asynMotorController::writeFloat64(pasynUser, value);
  }
}

asynStatus SmartController::writeController() {
  asynStatus status;
  static const char *functionName = "writeController";
  status = asynMotorController::writeController();
  if (status != asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Communication Error %d\n", driverName, functionName,
              status);
  }
  return status;
}

asynStatus SmartController::writeReadController() {
  asynStatus status;
  static const char *functionName = "writeReadController";

  /* Write outString_, get echo and resopnse */
  /* response is expected to be (ECHO echoEos RESPONSE inputEos) */
  status = asynMotorController::writeReadController();
  if (status != asynSuccess) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: Communication Error %d %s\n", driverName, functionName,
              status, outString_);
  }
  return status;
}

/** SmartController::setControllerMemory(char *location, epicsInt32 value)
*   Sets values to specific address in the master motor
**/
asynStatus SmartController::setControllerMemory(char *location, epicsInt32 value) {
// This will write all data to the master controller which is can addr 1
#ifdef DEBUG
  printf("%s:1=%d\n", location, value);
#endif
  asynStatus status;
  /*Writes to variable at address from the function input on the master motor*/
  sprintf(outString_, "%s:1=%d", location, value);
  status = writeController();
  if (status != asynSuccess)
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Store failure\n");
  return status;
}

/** SmartController::setDeferredMoves(bool deferMoves)
*   Sets values inside parameter list
*   True(1) will defer motion, False(0) will not defer motion
*   overrides method in asynMotorController
*   \param[in] deferMoves boolean flag indicates whether move is deffered
*   \param[in] pasynUser asynUser structure that encodes the reason and address
*/
asynStatus SmartController::setDeferredMoves(bool deferMoves,
                                             asynUser *pasynUser) {
  SmartAxisBase *pAxis = getAxis(pasynUser);
  if (!pAxis) {
    printf("Null axis %d\n", pAxis->axisNo_);
    return asynError;
  }
#ifdef DEBUG
  printf("%s\n", "SmartController::setDeferredMoves entered");
  printf("Axis No=%d, %d\n", pAxis->axisNo_, deferMoves);
#endif
  bool previousDeferMoves = deferMoves_;
  deferMoves_ = deferMoves;
  // set before entering process
  setIntegerParam(motorDeferMoves_, deferMoves_);
  callParamCallbacks();
  //  If new value is False="GO" and previous value is True="DEFER"
  //  then process the synchronized motion
  if (!deferMoves && previousDeferMoves) {
    printf("Procesing\n");
    pAxis->processDeferredMoves();
  }
  return asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg SmartCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SmartCreateControllerArg1 = {"Smart port name",
                                                   iocshArgString};
static const iocshArg SmartCreateControllerArg2 = {"Number of axes",
                                                   iocshArgInt};
static const iocshArg SmartCreateControllerArg3 = {"Number of virtual axes",
                                                   iocshArgInt};
static const iocshArg SmartCreateControllerArg4 = {"Moving poll period (ms)",
                                                   iocshArgInt};
static const iocshArg SmartCreateControllerArg5 = {"Idle poll period (ms)",
                                                   iocshArgInt};
static const iocshArg *const SmartCreateControllerArgs[] = {
    &SmartCreateControllerArg0, &SmartCreateControllerArg1,
    &SmartCreateControllerArg2, &SmartCreateControllerArg3,
    &SmartCreateControllerArg4, &SmartCreateControllerArg5};
static const iocshFuncDef SmartCreateControllerDef = {
    "SmartCreateController", 6, SmartCreateControllerArgs};
static void SmartCreateContollerCallFunc(const iocshArgBuf *args) {
  SmartCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                        args[4].ival, args[5].ival);
}

static void SmartMotorAsynRegister(void) {
  iocshRegister(&SmartCreateControllerDef, SmartCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(SmartMotorAsynRegister);
}
