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

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "SmartController.h"
#include <epicsExport.h>

static const char *driverName = "SmartAxisBaseDriver";

#define DEBUG 1
// These are the SmartAxis methods

/** Creates a new SmartAxis object.
  * \param[in] pC Pointer to the SmartController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
SmartAxisBase::SmartAxisBase(SmartController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls
 * asynMotorController::report()
  */
void SmartAxisBase::report(FILE *fp, int level) {
  if (level > 0) {
    fprintf(fp,
            "  axis %d\n"
            "    encoder position=%f\n"
            "    commanded position=%f\n"
            "    status=0x%x\n",
            axisNo_, encoderPosition_, commandedPosition_, currentStatus_);
  }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}

asynStatus SmartAxisBase::move(double position, int relative,
                               double minVelocity, double maxVelocity,
                               double acceleration) {
  return asynSuccess;
}

asynStatus SmartAxisBase::home(double minVelocity, double maxVelocity,
                               double acceleration, int forwards) {
  return asynSuccess;
}

asynStatus SmartAxisBase::moveVelocity(double minVelocity, double maxVelocity,
                                       double acceleration) {
  return asynSuccess;
}

asynStatus SmartAxisBase::stop(double acceleration) {
#ifdef DEBUG
  printf("STOP\n");
#endif
  asynStatus status;
  // static const char *functionName = "stopAxis";

  sprintf(pC_->outString_, "%cS", axisNum_);  // stop without deceleration
  //  sprintf(pC_->outString_,"%cADT=%f", axisNum_, acceleration); //set target
  //  decleration
  //  sprintf(pC_->outString_,"%cX", axisNum_); // decelerate to stop
  status = pC_->writeController();
  setPosition(encoderPosition_);
  setEncoderPosition(encoderPosition_);
  return status;
}

asynStatus SmartAxisBase::setPosition(double position) {
#ifdef DEBUG
  printf("SET POSITION:  ");
  printf("O=%ld\n", (long)round(position));
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cO=%ld", axisNum_, (long)round(position));
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxisBase::setEncoderPosition(double position) {
  return setPosition(position);
}

asynStatus SmartAxisBase::setClosedLoop(bool closedLoop) { return asynSuccess; }

asynStatus SmartAxisBase::setPGain(double pGain) { return asynSuccess; }

asynStatus SmartAxisBase::setIGain(double iGain) { return asynSuccess; }

asynStatus SmartAxisBase::setDGain(double dGain) { return asynSuccess; }

asynStatus SmartAxisBase::setHighLimit(double highLimit) { return asynSuccess; }

asynStatus SmartAxisBase::setLowLimit(double lowLimit) { return asynSuccess; }

asynStatus SmartAxisBase::setVelocity(double minVelocity, double maxVelocity) {
  return asynSuccess;
}

asynStatus SmartAxisBase::setAcceleration(double acceleration) {
  return asynSuccess;
}

asynStatus SmartAxisBase::processDeferredMoves() {
#ifdef DEBUG
  printf("%s\n", "SmartAxisBase::processDeferredMoves");
  printf("%s%d\n", "Axis No=", axisNum_);
#endif
  return asynSuccess;
}

asynStatus SmartAxisBase::go() {
#ifdef DEBUG
  printf("GO\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cG", axisNum_);  // GO
  status = pC_->writeController();
  return status;
}

asynStatus SmartAxisBase::resetFlags() {
#ifdef DEBUG
  printf("Reset flags\n");
#endif
  asynStatus status;
  sprintf(pC_->outString_, "%cZS", axisNum_);  //
  status = pC_->writeController();
  return status;
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit
 * status, the moving status,
  * and the drive power-on status.  It does not current detect following error,
 * etc. but this could be
  * added.
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (1)
 * or done (0). */
asynStatus SmartAxisBase::poll(bool *moving) { return asynSuccess; }
