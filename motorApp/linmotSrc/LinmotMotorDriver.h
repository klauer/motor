/*
FILENAME...   LinmotMotorDriver.h
USAGE...      Motor driver support for the Parker Linmot series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include <asynPortClient.h>

class epicsShareClass LinmotAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  LinmotAxis(class LinmotController *pC, int axis);
  void report(FILE *fp, int level);
  virtual asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setClosedLoop(bool closedLoop);

  
  asynStatus sendCmd( int command, int param1 = 0, int param2 = 0, int param3 = 0, int param4 = 0, int param5 = 0);

private:
  class LinmotController *pC_;      /**< Pointer to the asynMotorController to which this axis belongs.
                                *   Abbreviated because it is used very frequently */
  char axisName_[10];      /**< Name of each axis, used in commands to Linmot controller */ 
  double pulsesPerUnit_;   /**< Pulses per engineering unit, which is what Linmot controller uses */ 
  int flagsReg_;           /**< Address of the flags register */ 
  int limitsReg_;          /**< Address of the limits register */ 
  int encoderPositionReg_; /**< Address of the encoder position register */ 
  int theoryPositionReg_;  /**< Address of the theoretical position register */ 
  double encoderPosition_; /**< Cached copy of the encoder position */ 
  double theoryPosition_;  /**< Cached copy of the theoretical position */ 
  int currentFlags_;       /**< Cached copy of the current flags */ 
  int currentLimits_;      /**< Cached copy of the current limits */ 
  int done_;
  int dInput1_;
  int dInput2_;
  int movingPos_;
  int movingNeg_;
  int motionStarted_;

  int commandCount_;

  int controlWord_;       /**< cached copy of the control word */
  int stateVar_;
  int statusWord_;
  int warnWord_;
  int demandPosition_;
  int actualPosition_;
  int demandCurrent_;
friend class LinmotController;
};

class epicsShareClass LinmotController : public asynMotorController {
public:
  LinmotController(const char *portName, const char *LinmotPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
  LinmotAxis* getAxis(asynUser *pasynUser);
  LinmotAxis* getAxis(int axisNo);

  
  /* These are the methods that are new to this class */
  asynStatus readBinaryIO();

protected:
  asynInt32Client *stateVar_;
  asynInt32Client *statusWord_;
  asynInt32Client *warnWord_;
  asynInt32Client *demandPosition_;
  asynInt32Client *actualPosition_;
  asynInt32Client *demandCurrent_;
 
  asynInt32Client *controlWord_;
  asynInt32Client *commandHeader_;
  asynInt32Client *commandParam1_;
  asynInt32Client *commandParam2_;
  asynInt32Client *commandParam3_;
  asynInt32Client *commandParam4_;
  asynInt32Client *commandParam5_;

friend class LinmotAxis;
};
