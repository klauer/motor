/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_AXIS
#define SMART_AXIS

#include "SmartAxisBase.h"



class SmartAxis4 : public SmartAxisBase
{
public:
  /* These are the methods we override from the base class */
  SmartAxis4(class SmartController *pC, int axis);
  virtual asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  virtual asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  virtual asynStatus stop(double acceleration);
  virtual asynStatus poll(bool *moving);
  virtual asynStatus setPosition(double position);
  virtual asynStatus setEncoderPosition(double position);
  virtual asynStatus setClosedLoop(bool closedLoop);
  virtual asynStatus setPGain(double pGain);
  virtual asynStatus setIGain(double iGain);
  virtual asynStatus setDGain(double dGain);


protected:
  virtual asynStatus setAcceleration(double accleration);
  virtual asynStatus setVelocity(double minVelocity, double maxVelocity);
  virtual asynStatus go();
  virtual asynStatus resetFlags();
  virtual asynStatus forcePIDUpdate();
  virtual asynStatus getPID();


  asynStatus handleError(int error);

};

#endif
