/*
FILENAME...   EtherCATMotorDriver.h
USAGE...      Motor driver support for the Parker EtherCAT series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <asynPortClient.h>

class epicsShareClass EtherCATAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  EtherCATAxis(class EtherCATController *pC, int axis);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  
  asynStatus setPosition(double position);

private:
  class EtherCATController *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to EtherCAT controller */ 
  double pulsesPerUnit_;   /**< Pulses per engineering unit, which is what EtherCAT controller uses */ 
  int flagsReg_;           /**< Address of the flags register */ 
  int limitsReg_;          /**< Address of the limits register */ 
  int theoryPositionReg_;  /**< Address of the theoretical position register */ 
  epicsInt32 encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentFlags_;       /**< Cached copy of the current flags */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  int done_;
  int dInput1_;
  int dInput2_;
  int movingPos_;
  int movingNeg_;
  int motionStarted_;
  int slaveState_;
  int disable_;
  double targetPosition_;
friend class EtherCATController;
};

class epicsShareClass EtherCATController : public asynMotorController {
public:
  EtherCATController(const char *portName, const char *EtherCATPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  EtherCATAxis* getAxis(asynUser *pasynUser);
  EtherCATAxis* getAxis(int axisNo);

  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();
  
protected:
  /* asyn client interfaces to beckhoff PDO entries */
  asynInt32Client *alState_;
  asynInt32Client *disable_;

  asynInt32Client *stmCtrlEnable_;
  asynInt32Client *stmCtrlReset_;
  asynInt32Client *stmCtrlReduceTorque_;

  asynInt32Client *posCtrlExecute_;
  asynInt32Client *posCtrlTargetPosition_;
  asynInt32Client *posCtrlVelocity_;
  asynInt32Client *posCtrlStartType_;
  asynInt32Client *posCtrlAcceleration_;
  asynInt32Client *posCtrlDeceleration_;

  asynInt32Client *encCtrlSetCounter_;
  asynInt32Client *encCtrlSetCounterValue_;

  asynInt32Client *stmStatusMovingPos_;
  asynInt32Client *stmStatusMovingNeg_;
  asynInt32Client *stmStatusBusy_;
  asynInt32Client *stmStatusInTarget_;
  asynInt32Client *stmStatusInput1_;
  asynInt32Client *stmStatusInput2_;
  asynInt32Client *stmStatusReadyToEnable_;
  asynInt32Client *stmStatusReady_;



  asynInt32Client *posStatusActualPosition_;
  asynInt32Client *posStatusCounterUnderflow_;
  asynInt32Client *posStatusCounterOverflow_;

  asynInt32Client *masterCycleCount_;

  epicsInt32 cycle_;

private:
  
friend class EtherCATAxis;
};
