/*
FILENAME...   SmartMotorDriver.h
USAGE...      Motor driver support for the Parker Smart series of controllers, including the Aries.

Mark Rivers
March 28, 2011

*/
#ifndef SMART_CONTROLLER_H
#define SMART_CONTROLLER_H

#include "asynMotorController.h"
#include "SmartAxisBase.h"
#include "SmartAxis4.h"
#include "SmartAxis5.h"
#include <epicsString.h>

#define smartTempString "SMART_TEMP"
#define smartTorqueString "SMART_TORQUE"
#define smartErrorString "SMART_ERROR"
#define smartFirmwareString "SMART_FW"
#define smartCTR0String "SMART_CTR0"
#define smartCTR1String "SMART_CTR1"

inline double round(double num)
{
  return (num >0.0)?(num + 0.5):(num - 0.5);
}

static int Smart_addr[] = {128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
                           139, 140, 141, 142, 143, 144, 145, 146, 147, 148,
                           149, 150, 151, 152, 153, 154, 155, 156, 157, 158,
                           159, 160};

/** drvInfo strings for extra parameters that the Smart controller supports */
class SmartController : public asynMotorController {
public:
  SmartController(const char *portName, const char *SmartPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);
  /* These are the methods that we override from asynMotorDriver */
  void report(FILE *fp, int level);
//  SmartAxis* getAxis(asynUser *pasynUser);
//  SmartAxis* getAxis(int axisNo);
  SmartAxisBase* getAxis(asynUser *pasynUser);
  SmartAxisBase* getAxis(int axisNo);
  asynStatus writeController();
  
protected:
#define FIRST_SMART_PARAM smartMotorTemp_
  int smartMotorTemp_;           /**< reported smart motor temp, deg C */ 
  int smartMotorTorque_;         /**< commanded smart motor torque, scaled -1023 to 1023 */ 
  int smartError_;              /**< report smart motor status */ 
  int smartFirmwareVersion_;     /**< firmware version */
  int smartCtr0_;     /**< internal incremental encoder counter */
  int smartCtr1_;     /**< external incremental encoder counter */
#define LAST_SMART_PARAM smartCtr1_

#define NUM_SMART_PARAMS (&LAST_SMART_PARAM - &FIRST_SMART_PARAM + 1)

  asynStatus getFW(int &fwMajor, int axisNum, char* fw);
  
  
friend class SmartAxisBase;
friend class SmartAxis4;
friend class SmartAxis5;
};
#endif
