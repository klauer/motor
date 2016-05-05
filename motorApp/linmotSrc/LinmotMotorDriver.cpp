/*
FILENAME... LinmotMotorDriver.cpp
USAGE...    Motor driver support for the Parker Linmot series of controllers, including the Aries.

Mark Rivers
March 4, 2011

*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynInt32SyncIO.h>
#include <asynPortClient.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "LinmotMotorDriver.h"

#define NUM_Linmot_PARAMS 0

#define stateVarString       "DI.StateVar"
#define statusWordString     "DI.StatusWord"
#define warnWordString       "DI.WarnWord"
#define actualPositionString "DI.ActualPosition"
#define demandPositionString "DI.DemandPosition"
#define demandCurrentString  "DI.DemandCurrent"
#define controlWordString    "DO.ControlWord"
#define commandHeaderString  "DO.MotionCommandHeader"
#define commandParam1String  "DO.MotionCommandPar1"
#define commandParam2String  "DO.MotionCommandPar2"
#define commandParam3String  "DO.MotionCommandPar3"
#define commandParam4String  "DO.MotionCommandPar4"
#define commandParam5String  "DO.MotionCommandPar5"

/** State Var */
/**
 *     MAIN_STATE        | SUB STATE
 * 15 14 13 12 11 10 9 8 | 7 6 5 4 3 2 1 0 
 *
 */

/* MAIN_STATE 
#define NOT_READY          0x00
#define SWITCH_ON_DISABLED 0x01
#define READY_TO_SWITCH_ON 0x02
#define SETUP_ERROR        0x03
#define ERROR              0x04
#define HW_TEST            0x05
#define READY_TO_OPERATE   0x06
#define OPERATION_ENABLED  0x08
#define HOMING             0x09
#define CLEARANCE_CHECKING 0x0A
#define GOING_TO_INITIAL   0x0B
#define ABORTING           0x0C
#define FREEZING           0x0D
#define QUICK_STOPPING     0x0E
#define JOGING_POS         0x0F
#define JOGING_NEG         0x10
#define LINEARIZING        0x11
#define PHASE_SEARCHING    0x12
#define SPECIAL_MODE_      0x13
*/

struct stateVar
{
  enum state
  {
    NOT_READY                  = 0x0000,
    SWITCH_ON_DISABLED         = 0x0100,
    READY_TO_SWITCH_ON         = 0x0200,
    SETUP_ERROR                = 0x0300,
    ERROR                      = 0x0400,
    HW_TEST                    = 0x0500,
    READY_TO_OPERATE           = 0x0600,
    OPERATION_ENABLED          = 0x0800,
    HOMING                     = 0x0900,
    HOMING_FINISHED            = 0x090F,
    CLEARANCE_CHECK            = 0x0A00,
    CLEARANCE_CHECK_FINISHED   = 0x0A0F,
    GO_TO_INITIAL_POS          = 0x0B00,
    GO_TO_INITIAL_POS_FINISHED = 0x0B0F,
    ABORTING                   = 0x0C00,
    FREEZING                   = 0x0D00,
    QUICK_STOP                 = 0x0E00,
    GO_TO_POS                  = 0x0F00,
    GO_TO_POS_FINISHED         = 0x0F0F,
    JOGGING_POS                = 0x1001,    
    JOGGING_POS_FINISHED       = 0x100F,   
    JOGGING_NEG                = 0x1101,    
    JOGGING_NEG_FINISHED       = 0x110F,
    LINEARIZING                = 0x1200,
    PHASE_SEARCH               = 0x1300,
    SPECIAL_MODE               = 0x1400
  };
};

/* STATUS WORD */
struct statusWord {
  enum Status
  {
    ENABLED            = ( 1 << 0 ),
    SWITCH_ON          = ( 1 << 1 ),
    ENABLED_OPERATION  = ( 1 << 2 ),
    ERROR              = ( 1 << 3 ),
    VOLTAGE_ENABLED    = ( 1 << 4 ),
    QUICK_STOP         = ( 1 << 5 ),
    SWITCH_ON_LOCKED   = ( 1 << 6 ),
    WARNING            = ( 1 << 7 ),
    EVENT_HANDLER      = ( 1 << 8 ),
    SPECIAL_MODE       = ( 1 << 9 ),
    IN_TARGET_POS      = ( 1 << 10 ),
    HOMED              = ( 1 << 11 ),
    FATAL_ERROR        = ( 1 << 12 ),
    MOTION_ACTIVE      = ( 1 << 13 ),
    RANGED_IND1        = ( 1 << 14 ),
    RANGED_IND2        = ( 1 << 15 )
  };
};

struct warnWord {
  enum Warn
  {
    MOTOR_HOT                 = ( 1 << 0 ),
    MOTOR_SHORT_TIME_OVERLOAD = ( 1 << 1 ),
    MOTOR_SUPPLY_VOLTAGE_LOW  = ( 1 << 2 ),
    MOTOR_SUPPLY_VOLTAGE_HIGH = ( 1 << 3 ),
    POSITION_LAG_ALWAYS       = ( 1 << 4 ),
    RESERVED1                 = ( 1 << 5 ),
    DRIVE_HOT                 = ( 1 << 6 ),
    MOTOR_NOT_HOMED           = ( 1 << 7 ),
    PTC_SENSOR1_HOT           = ( 1 << 8 ),
    RESERVED_PTC2             = ( 1 << 9 ),
    RR_HOT                    = ( 1 << 10 ),
    RESERVED2                 = ( 1 << 11 ),
    RESERVED3                 = ( 1 << 12 ),
    RESERVED4                 = ( 1 << 13 ),
    INTERFACE_WARN            = ( 1 << 14 ),
    APPLICATION_WARN          = ( 1 << 15 ),
    ANY_WARN                  =  0xFFFF & ~( RESERVED1 | RESERVED2 | RESERVED3 | RESERVED4 )
  };
};

struct controlWord {
  enum Control
  {
    SWITCH_ON         = ( 1 << 0 ),
    VOLTAGE_ENABLE    = ( 1 << 1 ),
    QUICK_STOP        = ( 1 << 2 ),
    ENABLE_OPERATION  = ( 1 << 3 ),
    ABORT             = ( 1 << 4 ),
    FREEZE            = ( 1 << 5 ),
    GO_TO_POS         = ( 1 << 6 ),
    ERROR_ACKNOWLEDGE = ( 1 << 7 ),
    JOG_MOVE_POS      = ( 1 << 8 ),
    JOG_MOVE_NEG      = ( 1 << 9 ),
    SPECIAL_MODE      = ( 1 << 10 ),
    HOME              = ( 1 << 11 ),
    CLEARANCE_CHECK   = ( 1 << 12 ),
    GO_TO_INITIAL_POS = ( 1 << 13 ),
    RESERVED          = ( 1 << 14 ),
    PHASE_SEARCH      = ( 1 << 15 )
  };
};


/** Motion Interface */
#define VAI_GO_TO_POS       0x0100
#define VAI_INCREMENT_POS   0x0120

static const char *driverName = "LinmotMotorDriver";

/** Creates a new LinmotController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] LinmotPortName       The name of the drvAsynIPPPort that was created previously to connect to the Linmot controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time between polls when any axis is moving 
  * \param[in] idlePollPeriod    The time between polls when no axis is moving 
  */
LinmotController::LinmotController(const char *portName, const char *LinmotPortName, int numAxes, 
                             double movingPollPeriod, double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_Linmot_PARAMS, 
                         asynUInt32DigitalMask, 
                         asynUInt32DigitalMask,
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE, 
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  int axis;
  asynStatus status;
  static const char *functionName = "LinmotController";
lock();
  /* Connect to Linmot controller */
  status = pasynInt32SyncIO->connect(LinmotPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, 
      "%s:%s: cannot connect to Linmot controller\n",
      driverName, functionName);
  }
/* Lookup by name, must match scanner.xml asyn param names */
  try {
    stateVar_       = new asynInt32Client(LinmotPortName, 0, stateVarString);
    statusWord_     = new asynInt32Client(LinmotPortName, 0, statusWordString);
    warnWord_       = new asynInt32Client(LinmotPortName, 0, warnWordString);
    actualPosition_ = new asynInt32Client(LinmotPortName, 0, actualPositionString);
    demandPosition_ = new asynInt32Client(LinmotPortName, 0, demandPositionString);
    demandCurrent_  = new asynInt32Client(LinmotPortName, 0, demandCurrentString);

    controlWord_    = new asynInt32Client(LinmotPortName, 0, controlWordString);
    commandHeader_  = new asynInt32Client(LinmotPortName, 0, commandHeaderString);
    commandParam1_  = new asynInt32Client(LinmotPortName, 0, commandParam1String);
    commandParam2_  = new asynInt32Client(LinmotPortName, 0, commandParam2String);
    commandParam3_  = new asynInt32Client(LinmotPortName, 0, commandParam3String);
    commandParam4_  = new asynInt32Client(LinmotPortName, 0, commandParam4String);
    commandParam5_  = new asynInt32Client(LinmotPortName, 0, commandParam5String);
  }
  catch (...) {
//error
  }
unlock();

// Create the axis objects
  for (axis=0; axis<numAxes; axis++) {
    new LinmotAxis(this, axis);
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Creates a new LinmotController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] LinmotPortName       The name of the drvAsynIPPPort that was created previously to connect to the Linmot controller 
  * \param[in] numAxes           The number of axes that this controller supports 
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving 
  */
extern "C" int LinmotCreateController(const char *portName, const char *LinmotPortName, int numAxes, 
                                   int movingPollPeriod, int idlePollPeriod)
{
  new LinmotController(portName, LinmotPortName, 1, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void LinmotController::report(FILE *fp, int level)
{
  fprintf(fp, "Linmot motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  if (level > 0) {
  }


  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an LinmotMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
LinmotAxis* LinmotController::getAxis(asynUser *pasynUser)
{
  return static_cast<LinmotAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an LinmotMotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
LinmotAxis* LinmotController::getAxis(int axisNo)
{
  return static_cast<LinmotAxis*>(asynMotorController::getAxis(axisNo));
}

// These are the LinmotAxis methods

/** Creates a new LinmotAxis object.
  * \param[in] pC Pointer to the LinmotController to which this axis belongs. 
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  * 
  * Initializes register numbers, etc.
  */
LinmotAxis::LinmotAxis(LinmotController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  asynStatus status;

  pC_->lock();
  controlWord_ = 0x3E;
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);
  pC_->unlock();
  sprintf(axisName_, "AXIS%d", axisNo);

}

asynStatus LinmotAxis::sendCmd( int command, int param1, int param2, int param3, int param4, int param5)
{
  asynStatus status = asynSuccess;
  // must increment command counter
  pC_->lock();
  commandCount_ = (commandCount_ + 1) % 16;
  pC_->commandParam1_->write( param1 );
  pC_->commandParam2_->write( param2 );
  pC_->commandParam3_->write( param3 );
  pC_->commandParam4_->write( param4 );
  pC_->commandParam5_->write( param5 );
  pC_->commandHeader_->write(command | commandCount_ );
  pC_->unlock();

  return status;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void LinmotAxis::report(FILE *fp, int level)
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

/** VAI GO TO POSITION
  *   command header  010xh
  *   param 1 position 1e-7 m (0.1 um)
  *   param 2 velocity 1e-6 m/s
  *   param 3 acceleration 1e-5 m/s*s
  *   param 4 deceleration 1e-5 m/s*s
  *
  * VAI INCREMENT POSITION
  *   command header  012xh
  */

asynStatus LinmotAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;
  // static const char *functionName = "moveAxis";
  
  int pos   = (int) ( position);
  int velo  = (int) ( maxVelocity * 10 );
  int accel = (int) ( acceleration * 100 );

  printf("position %d, relative %d, velocity %d, acceleration %d", pos, relative, velo, accel);
 
  //scale everything to 0.1 um units..

 
  if( relative == 0 ) 
    status = sendCmd( VAI_GO_TO_POS, pos, velo, accel, accel );
    //status = sendCmd( VAI_GO_TO_POS, (int) position, (int) ( maxVelocity * 10 ), (int) acceleration, (int) acceleration );

  else
    status = sendCmd( VAI_INCREMENT_POS, pos, velo, accel, accel );

  pC_->lock();
  done_ = 0;
  setIntegerParam(pC_->motorStatusDone_, done_);
  pC_->unlock();
  return status;
}

asynStatus LinmotAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "home";
  epicsInt32 mask;


  pC_->lock();

  mask = controlWord::HOME;
  controlWord_ |= mask;
  pC_->controlWord_->write( controlWord_ );

  pC_->unlock();

  return status;
}

asynStatus LinmotAxis::stop(double acceleration )
{
  asynStatus status = asynSuccess;
  static const char *functionName = "stopAxis";
  epicsInt32 mask;
  /* clear the ABORT bit to do a quickstop */

  pC_->lock();

  mask = controlWord::ABORT;
  controlWord_ &= ~mask;
  pC_->controlWord_->write( controlWord_ );

  pC_->unlock();

  return status;
}

/** Set closed loop
  * This function sets closed loop -> enables the drive if not enabled
  * Will set motorStatusPowerOn_ bit true
  */

asynStatus LinmotAxis::setClosedLoop(bool closedLoop)
{
  asynStatus status = asynSuccess;
  static const char *functionName = "stopAxis";
  int mask;

  pC_->lock();

// acknowledge any errors
  if ( statusWord_ & statusWord::ERROR ) {
    mask = controlWord::ERROR_ACKNOWLEDGE;
    controlWord_ |= mask;
    pC_->controlWord_->write( controlWord_ );
  
    epicsThreadSleep(0.05);
  
    controlWord_ &= ~mask;
    pC_->controlWord_->write( controlWord_ );
  
    epicsThreadSleep(0.05);
  }

  mask = controlWord::SWITCH_ON;
  if( closedLoop ) // switch on, reset QUICK_STOP, ABORT, and FREEZE bits (active low)
    controlWord_ |= ( mask | controlWord::QUICK_STOP | controlWord::ABORT | controlWord::FREEZE );
  else
    controlWord_ &= ~mask;

  pC_->controlWord_->write( controlWord_ );

  if (status != asynSuccess)
    goto bail;


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
*/
asynStatus LinmotAxis::poll(bool *moving)
{ 
  asynStatus comStatus = asynSuccess;

  epicsInt32 mask;
  
  int homed;
  int enabled;
  int error;

  pC_->lock();

  /* Check linmot variables */
  pC_->stateVar_->read( &stateVar_ );
  pC_->statusWord_->read( &statusWord_ );
  pC_->warnWord_->read( &warnWord_ );
  pC_->demandPosition_->read( &demandPosition_ );
  pC_->actualPosition_->read( &actualPosition_ );
  pC_->demandCurrent_->read( &demandCurrent_ );


  mask = statusWord::HOMED;
  homed = statusWord_ & mask ? 1 : 0;
  setIntegerParam(pC_->motorStatusHomed_, homed);
  if( (controlWord_ & controlWord::HOME) && homed ) {
    controlWord_ &= ~controlWord::HOME;
    pC_->controlWord_->write( controlWord_ );
  }

  mask = stateVar::OPERATION_ENABLED;
  enabled = stateVar_ & mask;
  setIntegerParam(pC_->motorStatusPowerOn_, enabled);

  setDoubleParam(pC_->motorEncoderPosition_, actualPosition_);
  setDoubleParam(pC_->motorPosition_, actualPosition_);

  mask = statusWord::ERROR | statusWord::FATAL_ERROR;
  error = statusWord_ & mask ? 1 : 0;
  setIntegerParam(pC_->motorStatusProblem_, error);
  

  mask = statusWord::MOTION_ACTIVE;
  done_ = !( statusWord_ & mask );
  setIntegerParam(pC_->motorStatusDone_, done_);
  *moving = done_ ? false:true;
 
// check limits, these are on the x4.8 and x4.9 connector.  UPID 121Bh describes the
// limit error behavior
// 0 No error
// 1 Power off
// 2 Quick stop

//LS IN HIGH
    if( stateVar_ & ( stateVar::ERROR | 0x87) ) {
      setIntegerParam(pC_->motorStatusHighLimit_, 1);
    }
    else {
      setIntegerParam(pC_->motorStatusHighLimit_, 0);
    }
//LS OUT HIGH
    if( stateVar_ & ( stateVar::ERROR | 0x88) ) {
      setIntegerParam(pC_->motorStatusLowLimit_, 1);
    }
    else {
      setIntegerParam(pC_->motorStatusLowLimit_, 0);
    }
 
  skip:
  callParamCallbacks();
  pC_->unlock();
  return comStatus ? asynError : asynSuccess;
}

/** Code for iocsh registration */
static const iocshArg LinmotCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg LinmotCreateControllerArg1 = {"Linmot port name", iocshArgString};
static const iocshArg LinmotCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg LinmotCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg LinmotCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const LinmotCreateControllerArgs[] = {&LinmotCreateControllerArg0,
                                                           &LinmotCreateControllerArg1,
                                                           &LinmotCreateControllerArg2,
                                                           &LinmotCreateControllerArg3,
                                                           &LinmotCreateControllerArg4};
static const iocshFuncDef LinmotCreateControllerDef = {"LinmotCreateController", 5, LinmotCreateControllerArgs};
static void LinmotCreateContollerCallFunc(const iocshArgBuf *args)
{
  LinmotCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void LinmotMotorRegister(void)
{
  iocshRegister(&LinmotCreateControllerDef, LinmotCreateContollerCallFunc);
}

extern "C" {
epicsExportRegistrar(LinmotMotorRegister);
}
