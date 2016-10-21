/*
FILENAME... EtherCATMotorDriver.cpp
USAGE...    Motor driver support for the Parker EtherCAT series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynInt32SyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "EtherCATMotorDriver.h"

#define NUM_EtherCAT_PARAMS 0


#define alStateString                   "AL_STATE"
#define disableString                   "DISABLE"
#define stmCtrlEnableString             "STMControl.Control__Enable"
#define stmCtrlResetString              "STMControl.Control__Reset"
#define stmCtrlReduceTorqueString       "STMControl.Control__Reducetorque"
#define posCtrlExecuteString            "POSControl.Control__Execute"
#define posCtrlTargetPositionString     "POSControl.Targetposition"
#define posCtrlVelocityString           "POSControl.Velocity"
#define posCtrlStartTypeString          "POSControl.Starttype"
#define posCtrlAccelerationString       "POSControl.Acceleration"
#define posCtrlDecelerationString       "POSControl.Deceleration"
#define encCtrlSetCounterString         "ENCControl.Control__Setcounter"
#define encCtrlSetCounterValueString    "ENCControl.Setcountervalue"
#define stmStatusMovingPosString        "STMStatus.Status__Movingpositive" 
#define stmStatusMovingNegString        "STMStatus.Status__Movingnegative"
#define stmStatusBusyString             "POSStatus.Status__Busy"
#define stmStatusInTargetString         "POSStatus.Status__In-Target"
#define stmStatusInput1String           "STMStatus.Status__Digitalinput1"
#define stmStatusInput2String           "STMStatus.Status__Digitalinput2"
#define stmStatusReadyToEnableString    "STMStatus.Status__Readytoenable"
#define stmStatusReadyString            "STMStatus.Status__Ready"
#define posStatusActualPositionString   "POSStatus.Actualposition"
#define posStatusCounterUnderflowString "ENCStatus.Status__Counterunderflow"
#define posStatusCounterOverflowString  "ENCStatus.Status__Counteroverflow"

struct startType
{
  enum type
  {
     IDLE          = 0x0000,
     ABSOLUTE      = 0x0001,
     RELATIVE      = 0x0002,
     ENDLESS_PLUS  = 0x0003,
     ENDLESS_MINUS = 0x0004
  };
};

struct slaveState
{
  enum state
  {
    INIT     = 1,
    PREOP    = 2,
    SAFEOP   = 4,
    OP       = 8
  };
};

static const char *driverName = "EtherCATMotorDriver";

/** Creates a new EtherCATController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] EtherCATPortName       The name of the drvAsynIPPPort that was created previously to connect to the EtherCAT controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
EtherCATController::EtherCATController(const char *portName, const char *EtherCATPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_EtherCAT_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "EtherCATController";
  /* Connect to EtherCAT controller */
  status = pasynInt32SyncIO->connect(EtherCATPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to EtherCAT controller\n",
      driverName, functionName);
  }
 
  try {
    alState_               = new asynInt32Client(EtherCATPortName, 0, alStateString);
    disable_               = new asynInt32Client(EtherCATPortName, 0, disableString);

    stmCtrlEnable_         = new asynInt32Client(EtherCATPortName, 0, stmCtrlEnableString);
    stmCtrlReset_          = new asynInt32Client(EtherCATPortName, 0, stmCtrlResetString);
    stmCtrlReduceTorque_   = new asynInt32Client(EtherCATPortName, 0, stmCtrlReduceTorqueString);

    posCtrlExecute_        = new asynInt32Client(EtherCATPortName, 0, posCtrlExecuteString);
    posCtrlTargetPosition_ = new asynInt32Client(EtherCATPortName, 0, posCtrlTargetPositionString);
    posCtrlVelocity_       = new asynInt32Client(EtherCATPortName, 0, posCtrlVelocityString);
    posCtrlStartType_      = new asynInt32Client(EtherCATPortName, 0, posCtrlStartTypeString);
    posCtrlDeceleration_   = new asynInt32Client(EtherCATPortName, 0, posCtrlDecelerationString);
    posCtrlAcceleration_   = new asynInt32Client(EtherCATPortName, 0, posCtrlAccelerationString);

    encCtrlSetCounter_      = new asynInt32Client(EtherCATPortName, 0, encCtrlSetCounterString);
    encCtrlSetCounterValue_ = new asynInt32Client(EtherCATPortName, 0, encCtrlSetCounterValueString);

    stmStatusMovingPos_     = new asynInt32Client(EtherCATPortName, 0, stmStatusMovingPosString);
    stmStatusMovingNeg_     = new asynInt32Client(EtherCATPortName, 0, stmStatusMovingNegString);
    stmStatusMovingNeg_     = new asynInt32Client(EtherCATPortName, 0, stmStatusMovingNegString);
    stmStatusBusy_          = new asynInt32Client(EtherCATPortName, 0, stmStatusBusyString);
    stmStatusInTarget_      = new asynInt32Client(EtherCATPortName, 0, stmStatusInTargetString);
    stmStatusInput1_        = new asynInt32Client(EtherCATPortName, 0, stmStatusInput1String);
    stmStatusInput2_        = new asynInt32Client(EtherCATPortName, 0, stmStatusInput2String);
    stmStatusReadyToEnable_ = new asynInt32Client(EtherCATPortName, 0, stmStatusReadyToEnableString);
    stmStatusReady_         = new asynInt32Client(EtherCATPortName, 0, stmStatusReadyString);

    posStatusActualPosition_   = new asynInt32Client(EtherCATPortName, 0, posStatusActualPositionString);
    posStatusCounterUnderflow_ = new asynInt32Client(EtherCATPortName, 0, posStatusCounterUnderflowString);
    posStatusCounterOverflow_  = new asynInt32Client(EtherCATPortName, 0, posStatusCounterOverflowString);

    masterCycleCount_ = new asynInt32Client("MASTER0", 0, "Cycle");
  }
  catch (...) {

  }

  // Wait a short while so that any responses to the above commands have time to arrive so we can flush
  // them in the next writeReadController()
  epicsThreadSleep(0.5);
  // Read the binary I/O registers once
  // Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    new EtherCATAxis(this, axis);
  }

//POSStatus.Actualposition
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new EtherCATController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] EtherCATPortName       The name of the drvAsynIPPPort that was created previously to connect to the EtherCAT controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int EtherCATCreateController(const char *portName, const char *EtherCATPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new EtherCATController(portName, EtherCATPortName, numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void EtherCATController::report(FILE *fp, int level)
{
  fprintf(fp, "EtherCAT motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
  }


  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an EtherCATMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
EtherCATAxis* EtherCATController::getAxis(asynUser *pasynUser)
{
  return static_cast<EtherCATAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an EtherCATMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
EtherCATAxis* EtherCATController::getAxis(int axisNo)
{
  return static_cast<EtherCATAxis*>(asynMotorController::getAxis(axisNo));
}

// These are the EtherCATAxis methods

/** Creates a new EtherCATAxis object.
  * \param[in] pC Pointer to the EtherCATController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
EtherCATAxis::EtherCATAxis(EtherCATController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;
  sprintf(axisName_, "AXIS%d", axisNo);
  int ready;
  int readyToEnable;

  /* set gain support for CNEN to enable/disable motor */
  setIntegerParam(pC_->motorStatusGainSupport_, 1);

  pC_->stmStatusReadyToEnable_->read( &readyToEnable );
  pC_->stmStatusReady_->read( &ready );
  if ( readyToEnable ) {
    setClosedLoop( true );
  }

  /* Let's enbale the reduce torque mode at startup */
  pC_->stmCtrlReduceTorque_->write( 1 );

}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void EtherCATAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n"
            "    pulsesPerUnit_ = %f\n"
            "    encoder position=%f\n"
            "    theory position=%f\n"
            "    limits=0x%x\n"
            "    flags=0x%x\n", 
            axisNo_, pulsesPerUnit_, 
            encoderPosition_, theoryPosition_,
            currentLimits_, currentFlags_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

/** Parameter 5 type=asynInt32, name=STMControl.Control__Enable, value=-2147483648, status=0
*   Parameter 8 type=asynInt32, name=POSControl.Control__Execute, value=-2147483648, status=0
*   Parameter 10 type=asynInt32, name=POSControl.Targetposition, value=-2147483648, status=0
*   Parameter 11 type=asynInt32, name=POSControl.Velocity, value=-2147483648, status=0
*   Parameter 12 type=asynInt32, name=POSControl.Starttype, value=-2147483648, status=0
*   Parameter 13 type=asynInt32, name=POSControl.Acceleration, value=-2147483648, status=0
*   Parameter 14 type=asynInt32, name=POSControl.Deceleration, value=-2147483648, status=0
*   Parameter 12 type=asynInt32, name=POSControl.Starttype, value=-2147483648, status=0
*/
asynStatus EtherCATAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "moveAxis";
  epicsInt32 ready;

  pC_->lock();
  if( slaveState_ != slaveState::OP )
    goto skip;
 
  if ( ( (epicsInt32) position > encoderPosition_ ) && ( dInput1_ == 1 ) )
    goto skip;
  if ( ( (epicsInt32) position < encoderPosition_)  && ( dInput2_ == 1 ) )
    goto skip;

  pC_->stmStatusReady_->read( &ready );
  if( !ready )
    goto skip;

  pC_->stmCtrlReduceTorque_->write( 0 );
  pC_->posCtrlTargetPosition_->write( (int32_t)position ); 
  pC_->posCtrlVelocity_->write( (int) maxVelocity ); 
  pC_->posCtrlAcceleration_->write( (int) acceleration ); 
  pC_->posCtrlDeceleration_->write( (int) acceleration ); 

  pC_->posCtrlStartType_->write( (int)  ( ( relative == 1 ) ? startType::RELATIVE : startType::ABSOLUTE ) ); 
 
  pC_->posCtrlExecute_->write( 1 ); 
  epicsThreadSleep(0.1);

  done_ = 0;
  motionStarted_ = 1;
  setIntegerParam(pC_->motorStatusDone_, done_);

skip:
  callParamCallbacks();
  pC_->unlock(); 

  return status;
}

asynStatus EtherCATAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  // static const char *functionName = "moveAxis";
  int velocity;
  int sType;
  epicsInt32 ready;

  if ( maxVelocity < 0 ) {
    velocity = abs((int) maxVelocity);
    sType = startType::ENDLESS_MINUS; 
  } else {
    velocity = (int) maxVelocity;
    sType = startType::ENDLESS_PLUS; 

  }
  pC_->lock();
  if( slaveState_ != slaveState::OP )
    goto skip;
  
  pC_->stmStatusReady_->read( &ready );
  if( !ready )
    goto skip;

  pC_->posCtrlVelocity_->write( (int) velocity ); 
  pC_->posCtrlAcceleration_->write( (int) acceleration ); 
  pC_->posCtrlDeceleration_->write( (int) acceleration ); 
  pC_->posCtrlStartType_->write( (int) sType ); 
  epicsThreadSleep(0.1);
  pC_->posCtrlExecute_->write( 1 ); 
  done_ = 0;
  motionStarted_ = 1;
  setIntegerParam(pC_->motorStatusDone_, done_);

skip:
  pC_->unlock(); 
  return status;
}



asynStatus EtherCATAxis::stop(double acceleration )
{
  asynStatus status = asynSuccess;
  pC_->lock(); 
  //static const char *functionName = "stopAxis";
  pC_->posCtrlExecute_->write( 0 ); 
  pC_->stmCtrlReduceTorque_->write( 1 );
  motionStarted_ = 0;
  done_ = 1;
  setIntegerParam(pC_->motorStatusDone_, done_);

skip:
  pC_->unlock(); 
  return status;
}


/**  
*  Parameter 2 type=asynInt32, name=ENCControl.Control__Setcounter, value=-2147483648, status=0
*  Parameter 4 type=asynInt32, name=ENCControl.Setcountervalue, value=-2147483648, status=0
*/  

asynStatus EtherCATAxis::setPosition(double position)
{
  asynStatus status = asynSuccess;
  pC_->lock(); 
  if( slaveState_ != slaveState::OP )
    goto skip;
  pC_->encCtrlSetCounterValue_->write( (int32_t)position ); 
  pC_->encCtrlSetCounter_->write( 1 ); 
  epicsThreadSleep(0.1);
  pC_->encCtrlSetCounter_->write( 0 ); 

skip:
  pC_->unlock(); 
  return status;
}

asynStatus EtherCATAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "setClosedLoop";

  pC_->lock();

  pC_->stmCtrlEnable_->write( (epicsInt32) closedLoop );

  setIntegerParam(pC_->motorStatusPowerOn_, closedLoop);
  callParamCallbacks();

bail:
  pC_->unlock();
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */

/**
Parameter 33 type=asynInt32, name=STMStatus.Status__Movingpositive, value=-2147483648, status=0
Parameter 34 type=asynInt32, name=STMStatus.Status__Movingnegative, value=-2147483648, status=0
Parameter 40 type=asynInt32, name=POSStatus.Status__Busy, value=-2147483648, status=0
Parameter 41 type=asynInt32, name=POSStatus.Status__In-Target, value=-2147483648, status=0

Parameter 47 type=asynInt32, name=POSStatus.Actualposition, value=-2147483648, status=0

Parameter 18 type=asynInt32, name=ENCStatus.Status__Counterunderflow, value=1, status=0
Parameter 19 type=asynInt32, name=ENCStatus.Status__Counteroverflow, value=0, status=0
*/
asynStatus EtherCATAxis::poll(bool *moving)
{ 
  asynStatus comStatus = asynSuccess;
  
  epicsInt32 position;
  int busy;
  int inTarget;
  //int underFlow;
  int overFlow; 
  int movingPos;
  int movingNeg;
  pC_->lock();
/* check slave state, should be in OP */
  pC_->alState_->read( &slaveState_ );
  pC_->disable_->read( &disable_ );
  slaveState_ = disable_ ? 0 : slaveState_;
  if ( slaveState_ != slaveState::OP )
  {
    comStatus = asynError;
    goto skip;
  }

  //evaluate busy and in target
  if ( motionStarted_ == 1 ) {
    pC_->stmStatusBusy_->read( &busy );
    pC_->stmStatusInTarget_->read( &inTarget );
    if(busy == 0 && inTarget == 1) {
       pC_->posCtrlExecute_->write( 0 ); 
       pC_->stmCtrlReduceTorque_->write( 1 );
       motionStarted_ = 0;
    }
  }
  
  pC_->posStatusActualPosition_->read( &position );
  /*
  pC_->posStatusCounterUnderflow_->read( &underFlow );
  if ( underFlow == 1) {
    position = ( position | (epicsInt32) (0x80000000) );
  }  
  encoderPosition_ = position;
  setDoubleParam(pC_->motorEncoderPosition_, position);
  setDoubleParam(pC_->motorPosition_, position);
  */
  
  // The position register comes as a 31-bits word, so let's sign extend it and treat it as a sign value from now on
  if (position & 0x40000000)
    position |= 0x80000000;

  encoderPosition_ = (int32_t)position;
  setDoubleParam(pC_->motorEncoderPosition_, (int32_t)position);
  setDoubleParam(pC_->motorPosition_, (int32_t)position);

  //done_ = ((movingPos_ == 0) && (movingNeg_ = 0)) ? 1:0;

  pC_->stmStatusInput1_->read( &dInput1_ );
  pC_->stmStatusInput2_->read( &dInput2_ );
  setIntegerParam(pC_->motorStatusHighLimit_, dInput1_);
  setIntegerParam(pC_->motorStatusLowLimit_, dInput2_);

  pC_->stmStatusMovingPos_->read( &movingPos );
  pC_->stmStatusMovingNeg_->read( &movingNeg );
 
  if ( movingPos && dInput1_ )
   this->stop( 1000 );
  if ( movingNeg && dInput2_ )
   this->stop( 1000 );
 
  done_ = motionStarted_ == 1 ? 0 : 1;
  setIntegerParam(pC_->motorStatusDone_, done_);
  *moving = done_ ? false:true;
  
  skip:
  setIntegerParam(pC_->motorStatusProblem_, comStatus ? 1:0);
  callParamCallbacks();
  pC_->unlock();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg EtherCATCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg EtherCATCreateControllerArg1 = {"EtherCAT port name", iocshArgString};
static const iocshArg EtherCATCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg EtherCATCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg EtherCATCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const EtherCATCreateControllerArgs[] = {&EtherCATCreateControllerArg0,
                                                           &EtherCATCreateControllerArg1,
                                                           &EtherCATCreateControllerArg2,
                                                           &EtherCATCreateControllerArg3,
                                                           &EtherCATCreateControllerArg4};
static const iocshFuncDef EtherCATCreateControllerDef = {"EtherCATCreateController", 5, EtherCATCreateControllerArgs};
static void EtherCATCreateContollerCallFunc(const iocshArgBuf *args)
{
  EtherCATCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void EtherCATMotorRegister(void)
{
  iocshRegister(&EtherCATCreateControllerDef, EtherCATCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(EtherCATMotorRegister);
}
