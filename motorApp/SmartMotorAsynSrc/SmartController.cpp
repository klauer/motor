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

#define inputEos "\r"
#define outputEos " "

static const char *driverName = "SmartController";


/** Creates a new SmartController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SmartPortName     The name of the drvAsynIPPPort that was created previously to connect to the Smart controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
SmartController::SmartController(const char *portName, const char *SmartPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_SMART_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
//  SmartAxis *pAxis;
  SmartAxisBase *pAxis;
  static const char *functionName = "SmartController";
  int fwMajor; 
  char fw[20];

  createParam(smartTempString,         asynParamFloat64,      &smartMotorTemp_);
  createParam(smartTorqueString,       asynParamFloat64,      &smartMotorTorque_);
  createParam(smartErrorString,        asynParamOctet,        &smartError_);
  createParam(smartFirmwareString,     asynParamOctet,        &smartFirmwareVersion_);
  createParam(smartCTR0String,     asynParamFloat64,        &smartCtr0_);
  createParam(smartCTR1String,     asynParamFloat64,        &smartCtr1_);
  /* Connect to Smart controller */
  status = pasynOctetSyncIO->connect(SmartPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to Smart controller\n",
      driverName, functionName);
  }
  
  pasynOctetSyncIO->setOutputEos(pasynUserController_, outputEos,
                                          strlen(outputEos));
  pasynOctetSyncIO->setInputEos(pasynUserController_, inputEos,
                                          strlen(inputEos));
  // Wait a short while so that any responses to the above commands have time to arrive so we can flush
  // them in the next writeReadController()
  epicsThreadSleep(0.5);
  pasynOctetSyncIO->flush(pasynUserController_);
  // Create the axis objects
  if(numAxes < 1){
    numAxes = 1;
    numAxes_ = numAxes;
  }
    for (axis=0; axis<numAxes; axis++) {
/*  Check FW Version "connect" to axis */
      status = getFW(fwMajor, axis, fw);
      if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s cannont connect to axis %d\n",
        driverName, functionName, axis);
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
        "%s:%s Default to class 5 on axis %d\n",
        driverName, functionName, axis);
      }
      if(fwMajor == 4){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s NEW Class 4 Motor\n",
      driverName, functionName);
        pAxis = new SmartAxis4(this,axis);
      }
      else{  //default to class 5 smart motor...
      asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,
      "%s:%s NEW Class 5 Motor\n",
      driverName, functionName);
        pAxis = new SmartAxis5(this,axis);
      }
      setStringParam(axis, smartFirmwareVersion_, fw);
    }
exit:
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

asynStatus SmartController::getFW(int &fwMajor, int axisNum, char* fw){
  asynStatus status;
  static const char *functionName = "getFW";
  fwMajor = 0;
  char *pst, *tokSave;
  char fwMajorc[1];

  sprintf(outString_,"%cRSP",Smart_addr[axisNum]);
  status = writeReadController(); 
  sprintf(fw, inString_);

  if(status == asynSuccess){
    pst = epicsStrtok_r(inString_, "/", &tokSave); //response is clock/firmware (ex 23576/440C)
    pst = epicsStrtok_r(NULL, "/", &tokSave);
    strncpy(fwMajorc, pst, 1);  //get 1st character - FW major revision
    fwMajor = atoi(fwMajorc); 
  }
  else{
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
    "%s:%s cannot get FW major for axis %d\n", driverName, functionName, axisNum);
  }
  return status;
}


/** Creates a new SmartController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] SmartPortName       The name of the drvAsynIPPPort that was created previously to connect to the Smart controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int SmartCreateController(const char *portName, const char *SmartPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  SmartController *pSmartController
    = new SmartController(portName, SmartPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  pSmartController = NULL;
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void SmartController::report(FILE *fp, int level)
{
  fprintf(fp, "Smart motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
  }


  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an SmartMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */


//SmartAxis* SmartController::getAxis(asynUser *pasynUser)
//{
//  return static_cast<SmartAxis*>(asynMotorController::getAxis(pasynUser));
//}
//
///** Returns a pointer to an SmartMotorAxis object.
//  * Returns NULL if the axis number encoded in pasynUser is invalid.
//  * \param[in] axisNo Axis index number. */
//SmartAxis* SmartController::getAxis(int axisNo)
//{
//  return static_cast<SmartAxis*>(asynMotorController::getAxis(axisNo));
//}

SmartAxisBase* SmartController::getAxis(asynUser *pasynUser)
{
  return static_cast<SmartAxisBase*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an SmartMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
SmartAxisBase* SmartController::getAxis(int axisNo)
{
  return static_cast<SmartAxisBase*>(asynMotorController::getAxis(axisNo));
}

asynStatus SmartController::writeController(){
  asynStatus status;
  static const char *functionName = "writeController";
  status = asynMotorController::writeController();
  if(status != asynSuccess){
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
      "%s:%s: Communication Error %d\n",
      driverName, functionName, status, 0);
  }
  return status;
}

/** Code for iocsh registration */
static const iocshArg SmartCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg SmartCreateControllerArg1 = {"Smart port name", iocshArgString};
static const iocshArg SmartCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg SmartCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg SmartCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const SmartCreateControllerArgs[] = {&SmartCreateControllerArg0,
                                                           &SmartCreateControllerArg1,
                                                           &SmartCreateControllerArg2,
                                                           &SmartCreateControllerArg3,
                                                           &SmartCreateControllerArg4};
static const iocshFuncDef SmartCreateControllerDef = {"SmartCreateController", 5, SmartCreateControllerArgs};
static void SmartCreateContollerCallFunc(const iocshArgBuf *args)
{
  SmartCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void SmartMotorAsynRegister(void)
{
  iocshRegister(&SmartCreateControllerDef, SmartCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(SmartMotorAsynRegister);
}
