/*
FILENAME...     omsBaseController.cpp
USAGE...        Pro-Dex OMS asyn motor base controller support

Version:        $Revision$
Modified By:    $Author$
Last Modified:  $Date$
HeadURL:        $URL$
*/

/*
 *  Created on: 06/2012
 *      Author: eden
 *
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <epicsExit.h>

#include "omsBaseController.h"

#define MIN(a,b) ((a)<(b)? (a): (b))

#define motorOmsStringSendString        "OMS_STRING_SEND"
#define motorOmsStringSendRecvString    "OMS_STRING_SENDRECV"
#define motorOmsStringRecvString        "OMS_STRING_RECV"
#define motorOmsPollString              "OMS_POLL"
#define motorAuxString                  "MOTOR_AUX"
#define omsGpioString                   "OMS_GPIO"
#define omsAiString                     "OMS_AI"
#define MOTOR_OMS_PARAMS_COUNT           7 
#define MAX_QUEUE                        2559

ELLLIST omsBaseController::omsControllerList;
int omsBaseController::omsTotalControllerNumber = 0;

static const char *driverName = "omsBaseDriver";

#ifdef __GNUG__
    #ifdef      DEBUG
        #define Debug(l, f, args...) {if (l <= motorOMSBASEdebug) \
                                  errlogPrintf(f, ## args);}
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug
#endif
volatile int motorOMSBASEdebug = 0;
extern "C" {epicsExportAddress(int, motorOMSBASEdebug);}



omsBaseController::omsBaseController(const char *portName, int maxAxes, int prio, int stackSz, int extMotorParams=0)
    :   asynMotorController(portName, maxAxes, extMotorParams + MOTOR_OMS_PARAMS_COUNT,
            asynInt32Mask | asynFloat64Mask | asynOctetMask,
            asynInt32Mask | asynFloat64Mask | asynOctetMask,
            ASYN_CANBLOCK | ASYN_MULTIDEVICE,
            1, // autoconnect
            prio, stackSz)
{

    if (omsTotalControllerNumber == 0) {
        ellInit(&omsControllerList);
    }

    // check if portName is in use */
    if (findController(portName) != NULL)
        errlogPrintf("omsBaseController: ERROR: asynPort %s already in use\n", portName);

    omsBaseNode* pNode = new omsBaseNode;
    pNode->portName = epicsStrDup(portName);
    pNode->pController = this;
    ellAdd(&omsControllerList, (ELLNODE *)pNode);

    this->portName = epicsStrDup(portName);
    controllerNumber = omsTotalControllerNumber;
    ++omsTotalControllerNumber;

    sanityCounter = 0;
    useWatchdog = false;
    enabled = true;
    numAxes = maxAxes;
    controllerType = NULL;
    baseMutex = new epicsMutex;

    if (prio == 0)
        priority = epicsThreadPriorityLow;
    else
        priority = prio;

    if (stackSz == 0)
        stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
    else
        stackSize = stackSz;

    createParam(motorOmsPollString, asynParamInt32, &pollIndex);
    createParam(motorOmsStringSendString, asynParamOctet, &sendIndex);
    createParam(motorOmsStringSendRecvString, asynParamOctet, &sendReceiveIndex);
    createParam(motorOmsStringRecvString, asynParamOctet, &receiveIndex);
    createParam(motorAuxString, asynParamInt32, &motorAux);
    createParam(omsGpioString, asynParamInt32, &omsGpio);
    createParam(omsAiString, asynParamFloat64, &omsAi);

    setStringParam(sendIndex, (char *) "");
    setStringParam(sendReceiveIndex, (char *) "");
    setStringParam(receiveIndex, (char *) "");
    callParamCallbacks();

    /* Set an EPICS exit handler */
    epicsAtExit(omsBaseController::callShutdown, this);

}

asynStatus omsBaseController::startPoller(double movingPollPeriod, double idlePollPeriod, int forcedFastPolls)
{
    char threadName[20];

    movingPollPeriod_ = movingPollPeriod/1000.;
    idlePollPeriod_   = idlePollPeriod/1000.;
    forcedFastPolls_  = forcedFastPolls;

    epicsSnprintf(threadName, sizeof(threadName), "OMSPoller-%d", controllerNumber);
    this->motorThread = epicsThreadCreate(threadName,
            priority, stackSize,
            (EPICSTHREADFUNC) &omsBaseController::callPoller, (void *) this);
  return asynSuccess;
}


void omsBaseController::shutdown(){
      lock();
      shuttingDown_ = 1;
      unlock();
}

void omsBaseController::report(FILE *fp, int level)
{
    int axis;
    double velocity, position, encoderPosition;
    int haveEncoder=0;

    fprintf(fp, "Oms %s motor driver %s, numAxes=%d\n",
        controllerType, portName, numAxes);

    for (axis=0; axis < numAxes; axis++) {
        omsBaseAxis *pAxis = pAxes[axis];
        fprintf(fp, "  axis %d\n", pAxis->axisNo_);

        if (level > 0)
        {
            lock();
            getDoubleParam(pAxis->axisNo_, motorVelocity_, &velocity);
            getDoubleParam(pAxis->axisNo_, motorPosition_, &position);
            getIntegerParam(pAxis->axisNo_, motorStatusHasEncoder_, &haveEncoder);
            if (haveEncoder) getDoubleParam(pAxis->axisNo_, motorEncoderPosition_, &encoderPosition);
            unlock();
            fprintf(fp, "    Current position = %f, velocity = %f\n",
                   position,
                   velocity);

             if (haveEncoder) fprintf(fp, "    Encoder position %f\n", encoderPosition );
            if (pAxis->homing) fprintf(fp, "    Currently homing axis\n" );
        }
    }
    // Call the base class method
    asynMotorController::report(fp, level);
}

omsBaseAxis * omsBaseController::getAxis(asynUser *pasynUser)
{
    int axisNo;

    getAddress(pasynUser, &axisNo);
    return pAxes[axisNo];
}

/** Returns a pointer to an omsBaseAxis object.
  * Returns NULL if the axis number is invalid.
  * \param[in] axisNo Axis index number. */
omsBaseAxis* omsBaseController::getAxis(int axisNo)
{
    if ((axisNo < 0) || (axisNo >= numAxes)) return NULL;
    return pAxes[axisNo];
}

asynStatus omsBaseController::writeOctet(asynUser *pasynUser, const char *value,
                                    size_t nChars, size_t *nActual)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char *functionName = "writeOctet";
    char inBuffer[MAX_STRING_SIZE];
    if (strlen(value) > nChars) return asynError;

    omsBaseAxis *pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;

    if (function == sendReceiveIndex)
    {
        status = sendReceiveLock(value, inBuffer, sizeof(inBuffer),nActual);

        if (status == asynSuccess){
 
            status = (asynStatus)setStringParam(pAxis->axisNo_,sendReceiveIndex, (char *)inBuffer);
            *nActual=nChars;
        }
        else {
            *nActual = 0;
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:%s: sendReceive Error\n",
                    driverName, functionName, portName);

        }
    }
    else if (function == sendIndex)
    {
        status = sendOnlyLock(value);
        if (status == asynSuccess){
            *nActual = nChars;
        }
        else {
            *nActual = 0;
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "%s:%s:%s: send Error\n",
                    driverName, functionName, portName);
        }
    }
    return status;
}


asynStatus omsBaseController::flushOctet(asynUser *pasynUser)
{
    int function = pasynUser->reason;
    int addr=0;
    asynStatus status = asynSuccess;
   
    status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);

    status = setStringParam(addr, function, "");

    return status;
}

asynStatus omsBaseController::readOctet(asynUser *pasynUser,
                            char *value, size_t maxChars, size_t *nActual,
                            int *eomReason)
{
    int function = pasynUser->reason;
    int addr=0;
    asynStatus status = asynSuccess;
    static const char *functionName = "readOctet";

    status = getAddress(pasynUser, &addr); if (status != asynSuccess) return(status);
    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    status = (asynStatus)getStringParam(addr, function, (int)maxChars, value);
    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, value=%s",
                  driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%s\n",
              driverName, functionName, function, value);
    if (eomReason) *eomReason = ASYN_EOM_END;
    *nActual = strlen(value);
    return(status);
}


asynStatus omsBaseController::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    omsBaseAxis *pAxis = getAxis(pasynUser);
    static const char *functionName = "readInt32";
    static char outputBuffer[8];
    char auxStr;
    size_t nRead;
    char* ip;

    if (!pAxis) return asynError;

    if (function == motorPosition_) {
        strcpy(outputBuffer,"A? RP");
        sendReceiveReplace(pAxis, outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
        *value = strtol(inputBuffer, NULL, 10);
     } else if (function == motorEncoderPosition_) {
         int haveEncoder;
         getIntegerParam(pAxis->axisNo_, motorStatusHasEncoder_, &haveEncoder);
         if (haveEncoder){
             strcpy(outputBuffer,"A? RE");
             sendReceiveReplace(pAxis, outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
             *value = strtol(inputBuffer, NULL, 10);
         }

     } else if (function == motorAux) {
        strcpy(outputBuffer,"AA AB?;");
	status = sendReceiveLock(outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);

	auxStr = inputBuffer[2*pAxis->axisNo_];
 
	if (auxStr == 'l'){*value = 0;
	} else if (auxStr == 'h') {*value = 1;}
        else { 
	    asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s:%s wrong AUX state %c\n",
            driverName, functionName, portName, auxStr);
	    status = asynError;
        }
	
     } else if (function == omsGpio) {
	strcpy(outputBuffer,"BX;");
	status = sendReceiveLock(outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
        *value = strtol(inputBuffer, NULL, 16);
     }
    else {
          // Call base class
   	      status = asynMotorController::readInt32(pasynUser, value);
     }


    return status;
}

asynStatus omsBaseController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    int addr;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeInt32";
    omsBaseAxis *pAxis = getAxis(pasynUser);
    char outputBuffer[10];

    status=pasynManager->getAddr(pasynUser,&addr);

    if (!pAxis) return asynError;

    status = pAxis->setIntegerParam(function, value);

    if (function == motorDeferMoves_)
    {
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
            "%s:%s:%s Deferred Move: not yet implemented\n",
            driverName, functionName, portName );
    }
    else if (function == motorMoveToHome_) {
        /* avoid  asynMotorController::writeInt32 to handle this*/
    }
    else if (function == pollIndex)
    {
    	 if (value) {
    		 wakeupPoller();
    	 }
    }
    else if (function == motorAux)
    {
	if (value){ 
	    strcpy(outputBuffer,"A? ABH;");
            status = sendReplace(pAxis, outputBuffer);
	} else{ 
	    strcpy(outputBuffer,"A? ABL;");
	    status = sendReplace(pAxis, outputBuffer);
	}
    }
    else if (function == omsGpio)
    {
	sprintf(outputBuffer,"BS%x;",value);
    	status = sendOnlyLock(outputBuffer);
    }
    else {
        return asynMotorController::writeInt32(pasynUser, value);
    }

    pAxis->callParamCallbacks();

    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%d\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%d\n",
              driverName, functionName, function, value);
    return status;
}

asynStatus omsBaseController::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int function = pasynUser->reason;
    int addr, addr_rb;    
    asynStatus status = asynSuccess;
    static const char *functionName = "readFloat64";
    char outputBuffer[10];
    size_t nRead;

    omsBaseAxis *pAxis = getAxis(pasynUser);

    status=pasynManager->getAddr(pasynUser,&addr);

    if (!pAxis) return asynError;

    if (function == omsAi) {
        sprintf(outputBuffer,"AI%d;",addr);
//        printf("sending ,%s,\n",outputBuffer);
	status = sendReceiveLock(outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
	sscanf(inputBuffer,"AI%d=%lf\n",&addr_rb,value);
	if (addr_rb != addr){
        	asynPrint(pasynUser, ASYN_TRACE_ERROR,
                  "%s:%s:%s Requested and returned AI channel do not match (%d != %d)\n",
                  driverName, functionName, portName, addr_rb,addr);
	}
//	printf("read ,%s,\n",inputBuffer);
//        printf("AI read %d, %lf\n",addr_rb,*value);

     } else {
          // Call base class
   	      status = asynMotorController::readFloat64(pasynUser, value);
     }

    return status;

}

asynStatus omsBaseController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    static const char *functionName = "writeFloat64";
    omsBaseAxis *pAxis = getAxis(pasynUser);

    if (!pAxis) return asynError;


    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = pAxis->setDoubleParam(function, value);

    if (function == motorEncoderPosition_){
        int haveEncoder;
        getIntegerParam(pAxis->axisNo_, motorStatusHasEncoder_, &haveEncoder);
        if (haveEncoder){
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "%s:%s:%s axis %d set encoder position to %f\n",
                  driverName, functionName, portName, pAxis->axisNo_, value);
            sprintf(inputBuffer,"A? LPE%d;",(int)(value));
            status = sendReplace(pAxis, inputBuffer);
        }
     }
/*
 *   Encoder Mode is currently not supported in motor record
     else if (function == motorEncoderRatio_) {
         int haveEncoder;
         getIntegerParam(pAxis->axisNo_, motorStatusHasEncoder_, &haveEncoder);
         if (haveEncoder){
            asynPrint(pasynUser, ASYN_TRACE_FLOW,
                  "%s:%s:%s axis %d set encoder ratio to %f\n",
                  driverName, functionName, portName, pAxis->axisNo_, value);
            sprintf(inputBuffer,"A? ER%d,100000;",(int)(value*100000));
            status = sendReplace(pAxis, inputBuffer);
        }
     }
*/
     else if ((function == motorResolution_) || (function == motorLowLimit_) || (function == motorHighLimit_)) {
            // we do nothing
            status = asynSuccess;
     }
     else if (function == motorPGain_) {
	 value*=32767;
         if ((0.00 < value) && (value < 32768.00)){
             asynPrint(pasynUser, ASYN_TRACE_FLOW,
                      "%s:%s:%s axis %d set proportional gain to %f\n",
                      driverName, functionName, portName, pAxis->axisNo_, value);
              sprintf(inputBuffer,"A? KP%f;", value);
              status = sendReplace(pAxis, inputBuffer);
          }
          else {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s:%s axis %d proportional gain %f not in range 0.0 -> 32768.0\n",
                      driverName, functionName, portName, pAxis->axisNo_, value);
            }
        }
     else if (function == motorIGain_) {
	 value*=32767;
         if ((0.00 <= value) && (value < 32768.00)){
                asynPrint(pasynUser, ASYN_TRACE_FLOW,
                      "%s:%s:%s axis %d set integral gain to %f\n",
                      driverName, functionName, portName, pAxis->axisNo_, value);
                sprintf(inputBuffer,"A? KI%f;", value);
                status = sendReplace(pAxis, inputBuffer);
         }
         else {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s:%s axis %d integral gain %f not in range 0.0 -> 32768.0\n",
                      driverName, functionName, portName, pAxis->axisNo_, value);
         }
        }
     else if (function == motorDGain_)
    {
	 value*=32767;
         if ((0.00 <= value) && (value < 32768.00)){
                 asynPrint(pasynUser, ASYN_TRACE_FLOW,
                       "%s:%s:%s axis %d set derivative gain to %f\n",
                       driverName, functionName, portName, pAxis->axisNo_, value);
                sprintf(inputBuffer,"A? KD%f;", value);
                status = sendReplace(pAxis, inputBuffer);
          }
          else {
                asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s:%s axis %d derivative gain %f not in range 0.0 -> 32768.0\n",
                      driverName, functionName, portName, pAxis->axisNo_, value);
          }
    }
     else {
         /* Call base classes method (if we have our parameters check this here) */
         status = asynMotorController::writeFloat64(pasynUser, value);
    }

    pAxis->callParamCallbacks();
    if (status)
        asynPrint(pasynUser, ASYN_TRACE_ERROR,
              "%s:%s: error, status=%d function=%d, value=%f\n",
              driverName, functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, value=%f\n",
              driverName, functionName, function, value);
    return status;
}



asynStatus omsBaseController::getFirmwareVersion()
{
    /* get FirmwareVersion, try 3 times */
    int count = 0;
    asynStatus status = asynError;
    char *p;
    size_t nRead;

    while (status != asynSuccess && count < 3){
        epicsThreadSleep(1.0);
        status = sendReceiveLock((char*) "WY", inputBuffer, sizeof(inputBuffer), &nRead);
        count++;
        errlogPrintf("Reading OMS Firmware Version:\n");
    }

    if (count == 2) {
        errlogPrintf("omsBaseController::getFirmwareVersion: unable to retrieve Firmware version\n");
        status = asynError;
    }

    errlogPrintf("(%d) OMS Firmware Version is: /%s/\n",count,inputBuffer);

    if ((p = strstr(inputBuffer, "ver:"))){

	if (sscanf(p, "ver:%d.%d.%d,", &fwMajor, &fwMinor, &fwRevision)<3) {
		sscanf(p, "ver:B%d.%d,", &fwMajor, &fwMinor);
	}

    } else {
        errlogPrintf("omsBaseController::getFirmwareVersion: unable to parse Firmware version\n");
	fwMajor=0;
	fwMinor=0;
        status = asynError;
    }

    return status;
}

asynStatus omsBaseController::Init(const char* initString, int multiple){

    char *p, *tokSave;
    int totalAxes;
    char axisChrArr[OMS_MAX_AXES] = {'X','Y','Z','T','U','V','R','S'};
    char outputBuffer[10];
    size_t nRead;
    epicsInt32  axisPosArr[OMS_MAX_AXES];
    int axisQueue[OMS_MAX_AXES] = {0};

    /* Interrupt clear */
    sendOnlyLock("IC;");

    /* Stop all axes */
    sendOnlyLock("AM SA;");

    int retry = 0;
    int done = 0;
    while((done != numAxes) && retry < 10){
        done = 0;
        if (getAxesArray((char*) "AM;RQC;", axisQueue) == asynSuccess){
            for(int i=0; i<numAxes; i++){
                if(axisQueue[i] == MAX_QUEUE){
                    done++;
                }
            }
        }
        epicsThreadSleep(epicsThreadSleepQuantum());
        retry++;
    }


    /* send InitString */
    if ((initString != NULL) && (strlen(initString) > 0)) {
        if (multiple){
            char* inittmp = epicsStrDup(initString);
            for (tokSave = NULL, p = epicsStrtok_r(inittmp, ";", &tokSave);
                    p != NULL; p = epicsStrtok_r(NULL, ";", &tokSave)) {
                sendOnlyLock(p);
            }
            free(inittmp);
        }
        else {
            sendOnlyLock(initString);
        }
    }
    retry = 0;
    done = 0;
    while((done != numAxes) && retry < 10){
        done = 0;
        if (getAxesArray((char*) "AM;RQC;", axisQueue) == asynSuccess){
            for(int i=0; i<numAxes; i++){
                if(axisQueue[i] == MAX_QUEUE){
                    done++;
                }
            }
        }
        epicsThreadSleep(epicsThreadSleepQuantum());
        retry++;
    }

//    /* TEMP FIX **/
//    /* allows controller to complete init string */
//    epicsThreadSleep(0.2);
    /* get Positions of all axes */
    inputBuffer[0] = '\0';
    sendReceiveLock((char*) "AA RP;", inputBuffer, sizeof(inputBuffer), &nRead);

    if (numAxes > OMS_MAX_AXES) {
        errlogPrintf("omsBaseController:Init: number of axes=%d exceeds allowed maximum\n", numAxes);
        return asynError;
    }

    /* The return string will tell us how many axes this controller has */
    for (totalAxes = 0, tokSave = NULL, p = epicsStrtok_r(inputBuffer, ",", &tokSave);
             p != 0; p = epicsStrtok_r(NULL, ",", &tokSave), totalAxes++);
    if ((totalAxes > numAxes) || (totalAxes > OMS_MAX_AXES)) {
        errlogPrintf("omsBaseController:Init: actual number of axes=%d > MIN(numAxes=%d, OMS_MAX_AXES)\n", totalAxes, numAxes);
        return asynError;
    }
    numAxes = totalAxes;

    pAxes = new omsBaseAxis*[numAxes];
    lock();
    for (int axis=0; axis < numAxes; axis++) {

        omsBaseAxis* pAxis  = new omsBaseAxis(this, axis, axisChrArr[axis]);
        pAxes[axis] = pAxis;

        pAxis->setIntegerParam(motorStatusDone_, 1);
        pAxis->setIntegerParam(motorStatusMoving_, 0);
        pAxis->setIntegerParam(motorStatusHomed_, 0);
        pAxis->setIntegerParam(motorStatusHome_, 0);
        pAxis->setIntegerParam(motorStatusAtHome_, 0);
        pAxis->setIntegerParam(motorStatusProblem_, 0);
        pAxis->setIntegerParam(motorStatusCommsError_, 0);

        /* Determine if encoder is present and if mode is stepper or servo. */
        if (firmwareMin(1,1,30))
            strcpy(outputBuffer,"A? PS?");
        else
            strcpy(outputBuffer,"A? ?PS");
        sendReceiveReplace(pAxis, outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
        /*we expect any of "=O","=E","=M"  */
        if (inputBuffer[1] == 'O'){
            pAxis->setStepper(1);
            pAxis->setAbsoluteEncoder(0);
            pAxis->setIntegerParam(motorStatusHasEncoder_, 0);
            pAxis->setIntegerParam(motorStatusGainSupport_, 0);
            pAxis->setDoubleParam(motorEncoderPosition_, 0.0); // reset if not available
        }
        else if (inputBuffer[1] == 'M'){
            pAxis->setStepper(0);
            pAxis->setIntegerParam(motorStatusHasEncoder_, 1);
            pAxis->setIntegerParam(motorStatusGainSupport_, 1);
        }
        else if (inputBuffer[1] == 'E'){
            pAxis->setStepper(1);
            pAxis->setIntegerParam(motorStatusHasEncoder_, 1);
            pAxis->setIntegerParam(motorStatusGainSupport_, 0);
        }
        else
            errlogPrintf("omsBaseController:Init: error: unknown axis type! (%s)\n", inputBuffer);

        /* Determine limit true state high or low */
        /* CAUTION you need firmware version 1.30 or higher to do this */
        if (firmwareMin(1,1,30))
            strcpy(outputBuffer,"A? LT?");
        else
            strcpy(outputBuffer,"A? ?LS");
        sendReceiveReplace(pAxis, outputBuffer, inputBuffer, sizeof(inputBuffer), &nRead);
        /*we expect any of "=l" or "=h"  */
        if (inputBuffer[1] == 'l'){
            pAxis->setLimitInvert(1);
        }
        else if (inputBuffer[1] == 'h'){
            pAxis->setLimitInvert(0);
        }
        else
            errlogPrintf("omsBaseController:Init: error: unknown limit true state!\n");

        /* Determine limit true state high or low */
        /* CAUTION you need firmware version 1.30 or higher to do this */
        pAxis->callParamCallbacks();

    }
    if (getAxesPositions(axisPosArr) == asynSuccess){
    	for (int axis=0; axis < numAxes; axis++)
    		(pAxes[axis])->setDoubleParam(motorPosition_, (double) axisPosArr[axis]);
    }
    unlock();
    return asynSuccess;
}

void omsBaseController::callPoller(void *drvPvt)
{
    omsBaseController *pController = (omsBaseController*)drvPvt;
    pController->omsPoller();
}


#define DELTA 0.1
void omsBaseController::omsPoller()
{

    static const char *functionName = "BasePoller";

#define STATUSSTRINGLEN 5
    /* This is the task that polls the MAX card */
    double timeout, timeToWait;
    double movingPollPeriod;
    double idlePollPeriod;
    omsBaseAxis* pAxis;
    int retry_count, loopBreakCount=0, haveEncoder;
    // asynStatus status;
    int axisMoving, anyMoving=0;
    int forcedFastPolls, fastPolls=0;
    epicsInt32  axisPosArr[OMS_MAX_AXES], encPosArr[OMS_MAX_AXES], veloArr[OMS_MAX_AXES];
    char statusBuffer[OMS_MAX_AXES*STATUSSTRINGLEN+2];
    char encStatusBuffer[OMS_MAX_AXES*6+2];
    int closedLoopStatus[OMS_MAX_AXES];
    int axisQueue[OMS_MAX_AXES];
    int queueThresh = 100;  /* flush axis queue if avaliable axis queue dips below this threshold */
    char encBuffer[7];
    unsigned int limitFlags;
    epicsTimeStamp now, loopStart;
    bool haveCLStatus, haveVeloArray, haveEncStatus, haveLimits, useEncoder=false, moveDone;
    size_t nRead;

    lock();
    movingPollPeriod = movingPollPeriod_;
    idlePollPeriod = idlePollPeriod_;
    forcedFastPolls = forcedFastPolls_;
    for (int i=0; (i < numAxes); i++) {
        getIntegerParam(i, motorStatusHasEncoder_, &haveEncoder);
        if (haveEncoder) useEncoder = true;
    }
    unlock();


    while(1) {

        lock();
        if (shuttingDown_) {
          unlock();
          break;
        }
        unlock();

        if (loopBreakCount > 10){
            errlogPrintf("%s:%s:%s: Error: %d consecutive unsuccessful attempts to read from motor card\n"
                    , driverName, functionName, this->portName, loopBreakCount);
            loopBreakCount = 0;
            flushQueues();
        }

        epicsTimeGetCurrent(&loopStart);

        retry_count = 0;
        while ((getAxesPositions(axisPosArr) != asynSuccess) && (retry_count < 5)){
            epicsThreadSleepQuantum();
            ++retry_count;
        }


        if (retry_count > 4){
            errlogPrintf("%s:%s:%s: error reading axis position after %d attempts\n",
                    driverName, functionName, this->portName, retry_count);
            ++loopBreakCount;
        }
	
        if (useEncoder && (getEncoderPositions(encPosArr) != asynSuccess)){
            Debug(1, "%s:%s:%s: error executing get Encoder Positions\n", driverName, functionName, this->portName);
            ++loopBreakCount;
        }

        /* read all axis status values and reset done-field
         * MDNN,MDNN,PNLN,PNNN,PNLN,PNNN,PNNN,PNNN*/
        if (getAxesStatus(statusBuffer, sizeof(statusBuffer), &moveDone) != asynSuccess){
             Debug(1, "%s:%s:%s: error reading axes status\n", driverName, functionName, this->portName);
            ++loopBreakCount;
        }

        if (getAxesArray((char*) "AM; RQC;", axisQueue) != asynSuccess){
             Debug(1, "%s:%s:%s: error reading axes queue status\n", driverName, functionName, this->portName);
            ++loopBreakCount;
        }

/*
        if (sanityCheck() != asynSuccess){
            errlogPrintf("%s:%s:%s: error during sanity check\n", driverName, functionName, this->portName);
        }
*/

        if (anyMoving)
            haveCLStatus = false;
        else
            haveCLStatus = true;
        if (haveCLStatus && getClosedLoopStatus(closedLoopStatus) != asynSuccess){
            haveCLStatus = false;
            Debug(1, "%s:%s:%s: error executing get Closed Loop Status\n", driverName, functionName, this->portName);
        }

        haveVeloArray = true;
        if (getAxesArray((char*) "AM;RV;", veloArr) != asynSuccess){
            haveVeloArray = false;
            Debug(1,"%s:%s:%s: Error executing command Report Velocity (RV)\n", driverName, functionName, this->portName);
        }
        haveEncStatus = true;
        if (sendReceiveLock((char*) "AM;EA;", encStatusBuffer, sizeof(encStatusBuffer), &nRead) != asynSuccess){
            haveEncStatus = false;
            Debug(1,"%s:%s:%s: Error reading encoder status buffer >%s<\n", driverName, functionName, this->portName, encStatusBuffer);
        }

        /* read all limits */
        if (moveDone || (anyMoving == 0))
            haveLimits = true;
        else
              haveLimits = false;
        limitFlags =0;
        if (haveLimits){
            if ((sendReceiveLock((char*) "AM;QL;", pollInputBuffer, sizeof(pollInputBuffer), &nRead) == asynSuccess)){
                if (1 != sscanf(pollInputBuffer, "%x", &limitFlags)){
                    Debug(1,"%s:%s:%s: error converting limits: %s\n", driverName, functionName, this->portName, pollInputBuffer);
                    haveLimits = false;
                }
            }
            else {
                haveLimits = false;
                Debug(1,"%s:%s:%s: error reading limits %s\n", driverName, functionName, this->portName, pollInputBuffer);
            }
        }
        if (enabled) watchdogOK();

        anyMoving = 0;
        lock();
        for (int i=0; (i < numAxes) && enabled && (shuttingDown_ == 0); i++) {
            pAxis = pAxes[i];

            if (useEncoder){
                /* if this axis has an encoder, read encoder status */
                getIntegerParam(i, motorStatusHasEncoder_, &haveEncoder);
                if (haveEncoder){
                    if (haveEncStatus){
                        if (getSubstring(i, encStatusBuffer, encBuffer, sizeof(encBuffer)) == asynSuccess){
                            if (encBuffer[2] == 'S')
                                pAxis->setIntegerParam(motorStatusFollowingError_, 1);
                            else
                                pAxis->setIntegerParam(motorStatusFollowingError_, 0);
                        }
                        else{
                            errlogPrintf("%s:%s:%s: error parsing encoder status string %s\n",
                                    driverName, functionName, this->portName, encStatusBuffer);
                        }
                    }
                    pAxis->setDoubleParam(motorEncoderPosition_, (double) encPosArr[i]);
                }
            }

            /* check the done flag or current velocity */
            if (statusBuffer[i*STATUSSTRINGLEN + 1] == 'D'){
                Debug(3, "%s:%s:%s: found Done Flag axis %d\n", driverName, functionName, portName, i);
                pAxis->setIntegerParam(motorStatusProblem_, 0);
                pAxis->moveDelay=0;
                pAxis->setIntegerParam(motorStatusDone_, 1);
                pAxis->setIntegerParam(motorStatusMoving_, 0);
                if (pAxis->homing) pAxis->homing = 0;
                Debug(1, "%s:%s:%s: done axis %d \n", driverName, functionName,
                        this->portName, i );
            }
            else if (haveVeloArray && (veloArr[i] == 0)){
                /* keep a small delay here, to make sure that a motorAxisDone cycle is performed
                   even if the motor is on hard limits */
                getIntegerParam(pAxis->axisNo_, motorStatusMoving_, &axisMoving);
                if (axisMoving){
                    pAxis->moveDelay++ ;
                    if (pAxis->moveDelay >= 5) {
                        Debug(4, "%s:%s:%s: setting Problem Flag axis %d\n", driverName, functionName, portName, i);
                        pAxis->setIntegerParam(motorStatusDone_, 1);
                        pAxis->setIntegerParam(motorStatusMoving_, 0);
                        pAxis->setIntegerParam(motorStatusProblem_, 1);
                        pAxis->moveDelay = 0;
                        if (pAxis->homing) pAxis->homing = 0;
                        Debug(1, "%s:%s:%s: stop axis %d, moveDelay count %d\n", driverName, functionName,
                                this->portName, i, pAxis->moveDelay );
                    }
                    Debug(2, "%s:%s:%s: moveDelay axis %d, count %d\n", driverName, functionName,
                            this->portName, i, pAxis->moveDelay );
               }
            }
            else {
                Debug(4, "%s:%s:%s: poller loop: axis %d still moving\n", driverName, functionName, this->portName, i);
                pAxis->moveDelay = 0;
                pAxis->setIntegerParam(motorStatusProblem_, 0);
                pAxis->setIntegerParam(motorStatusDone_, 0);
                pAxis->setIntegerParam(motorStatusMoving_, 1);
            }
            getIntegerParam(pAxis->axisNo_, motorStatusMoving_, &axisMoving);
            anyMoving += axisMoving;

            /* check limits */
	    if (haveLimits){
                if (((limitFlags & (1 << i)) > 0) ^ (pAxis->getLimitInvert()))
                    pAxis->setIntegerParam(motorStatusLowLimit_, 1);
                else
                    pAxis->setIntegerParam(motorStatusLowLimit_, 0);
                if (((limitFlags & (1 << (i+8))) > 0) ^ (pAxis->getLimitInvert()))
                    pAxis->setIntegerParam(motorStatusHighLimit_, 1);
                else
                    pAxis->setIntegerParam(motorStatusHighLimit_, 0);
            }


            /* check home switch */
            if (statusBuffer[i*STATUSSTRINGLEN + 3] == 'H')
                pAxis->setIntegerParam(motorStatusAtHome_, 1);
            else
                pAxis->setIntegerParam(motorStatusAtHome_, 0);

            /* check direction */
            if (statusBuffer[i*STATUSSTRINGLEN] == 'P')
                pAxis->setIntegerParam(motorStatusDirection_, 1);
            else
                pAxis->setIntegerParam(motorStatusDirection_, 0);

            /* set positions */
            pAxis->setDoubleParam(motorPosition_, (double) axisPosArr[i]);

            /* set closed loop status */
            if (haveCLStatus) pAxis->setIntegerParam(motorStatusPowerOn_, closedLoopStatus[i]);

            /* check axis queue size, issue stop if below threshold */
            
            if(axisQueue[i] <= queueThresh){
                pAxis->stop(0); //will use minimum acceleration, issues ST command and flushes axis queue
            }
            // callParamCallbacks(pAxis->axisNo_, pAxis->axisNo_);
            pAxis->callParamCallbacks();

        } /* Next axis */

        if (shuttingDown_) {
          unlock();
          break;
        }
        unlock();

        if (fastPolls > 0) {
            timeout = movingPollPeriod;
            fastPolls--;
        }
        else if (anyMoving) {
            timeout = movingPollPeriod;
        }
        else {
            timeout = idlePollPeriod;
        }

        /* wait here for the next poll
           waiting may be interrupted by pollEvent or interrupt messages*/
        epicsTimeGetCurrent(&now);
        timeToWait = timeout - epicsTimeDiffInSeconds(&now, &loopStart);
        Debug(5, "%s:%s:%s: poller loop: waiting %f s\n", driverName, functionName, this->portName, timeToWait);


        if (waitInterruptible(timeToWait) == epicsEventWaitOK) {
            fastPolls = forcedFastPolls;
        }


    } /* End while */
    Debug(1, "%s:%s:%s: omsPoller shutdown\n", driverName, functionName, portName);
}

asynStatus omsBaseController::getEncoderPositions(epicsInt32 encPosArr[OMS_MAX_AXES])
{
    return getAxesArray((char*) "AM PE;", encPosArr);
}


asynStatus omsBaseController::getClosedLoopStatus(int clstatus[OMS_MAX_AXES])
{
    asynStatus status = asynSuccess;
    char clBuffer[9];
    size_t nRead;

    if (firmwareMin(1,30,0)){
        pollInputBuffer[0] = '\0';
        status = sendReceiveLock((char*) "AM;CL?;", pollInputBuffer, sizeof(pollInputBuffer), &nRead);
        if (status == asynSuccess) {
            for (int i=0; i < numAxes; ++i) {
                status = getSubstring(i, pollInputBuffer, clBuffer, sizeof(clBuffer));
                if ( status == asynSuccess){
                    if (strncmp(clBuffer, "on", 2) == 0)
                        clstatus[i] = 1;
                    else
                        clstatus[i] = 0;
                }
            }
        }
    }
    else {
        for (int i=0; i < numAxes; ++i) {
            strcpy(clBuffer,"A? ?PM");
            status = sendReceiveReplace(pAxes[i], clBuffer, pollInputBuffer, sizeof(pollInputBuffer), &nRead);
            if (status != asynSuccess){
                Debug(1, "%s:getClosedLoopStatus:%s: Error getting closed loop status %s\n",
                        driverName, portName, pollInputBuffer);
            }
            else {
                if (strncmp(pollInputBuffer, "=on", 3) == 0)
                    clstatus[i] = 1;
                else
                    clstatus[i] = 0;
            }
        }
    }
    return status;
}

asynStatus omsBaseController::sanityCheck()
{
    const char* functionName="sanityCheck";
    asynStatus status = asynSuccess;

    ++ sanityCounter;

    if (sanityCounter > 100){
        int commandBufferSize[OMS_MAX_AXES];
        char outputBuffer[10];
        // check the remaining command buffer size, which must not overflow
        sanityCounter = 0;
        if (getAxesArray((char*) "AM;RQC", commandBufferSize) != asynSuccess){
            errlogPrintf("%s:%s:%s: Error executing command: Report Command Queue (RCQ)\n", driverName, functionName, this->portName);
        }
        for (int i=0; i < numAxes; i++) {
            omsBaseAxis* pAxis = pAxes[i];
            // flush command buffer if size is below 100
            if (commandBufferSize[i] < 100) {
                errlogPrintf("%s:%s:%s: Caution: flushing command queue axis %d,remaining size %d\n",
                                    driverName, functionName, portName, pAxis->axisNo_, commandBufferSize[i]);
                strcpy(outputBuffer,"A?;FL;");
                sendReplace(pAxis, (char*) outputBuffer);
                status = asynError;
            }
        }
    }
    return status;
}

bool omsBaseController::firmwareMin(int major, int minor, int revision){
    if (major < fwMajor) return true;
    if (major == fwMajor){
        if (minor < fwMinor) return true;
        if (minor == fwMinor){
            if (revision <= fwRevision) return true;
        }
    }
    return false;
}

omsBaseController* omsBaseController::findController(const char* asynPort){

    omsBaseNode* pNode = (omsBaseNode*) ellFirst(&omsControllerList);
    while (pNode) {
        if (strcmp(pNode->portName, asynPort) == 0){
            return (pNode->pController);
        }
        pNode = (omsBaseNode *)ellNext(&pNode->node);
    }
    return (omsBaseController *) NULL;
}

epicsEventWaitStatus omsBaseController::waitInterruptible(double timeout)
{
    return epicsEventWaitWithTimeout(pollEventId_, timeout);
}

/*
    insert axis character and send to Controller
    If insertChar != 0 and 2nd Character is a "?", then replace it with axisChar
*/
asynStatus omsBaseController::sendReplace(omsBaseAxis* pAxis, char *outputBuff)
{
    asynStatus status;

    if (strncmp(outputBuff,"A? ",3) == 0 ){
        outputBuff[1] = pAxis->axisChar;
   }
   printf("%s\n",outputBuff);

    status = sendOnlyLock(outputBuff);
    return status;
}

/*
    insert axis character, send to Controller and wait for the answer
    If insertChar != 0 and 2nd Character is a "?", then replace it with axisChar
*/
asynStatus omsBaseController::sendReceiveReplace(omsBaseAxis* pAxis, char *outputBuff, char *inputBuff, unsigned int inputSize, size_t *nRead)
{
    asynStatus status;

    if (strncmp(outputBuff,"A? ",3) == 0 )
        outputBuff[1] = pAxis->axisChar;
    status = sendReceiveLock(outputBuff, inputBuff, sizeof(inputBuff), nRead);
    return status;
}

asynStatus omsBaseController::sendOnlyLock(const char *outputBuff)
{
    asynStatus status;
    baseMutex->lock();
    status = sendOnly(outputBuff);
    baseMutex->unlock();
    return status;
}

asynStatus omsBaseController::sendReceiveLock(const char *outputBuff, char *inputBuff, unsigned int inputSize, size_t *nRead)
{
    asynStatus status;
    baseMutex->lock();
    
    status = sendReceive(outputBuff, inputBuff, inputSize, nRead);
//    status = sendReceive(outputBuff, inputBuff, inputSize);

    baseMutex->unlock();
    return status;
}

asynStatus omsBaseController::getAxesStatus(char *inputBuff, int inputSize, bool *done)
{
    char *outputBuff = (char*) "AM;RI;";
    asynStatus status;
    size_t nRead;

    *done=false;

    status = sendReceiveLock(outputBuff, inputBuff, inputSize, &nRead);

    if (status == asynSuccess){
        if (strchr(inputBuff, 'D') != NULL) *done=true;
        if (!((inputBuff[0] == 'P') || (inputBuff[0] == 'M')))
            status = asynError;
        if (strlen(inputBuff) < (unsigned int)(numAxes * 5 -1))
            status = asynError;
        if (status == asynError) Debug(1, "%s:getAxesStatus:%s: corrupted status string %s\n",
                driverName, portName, inputBuff);
    }
    return status;
}

asynStatus omsBaseController::getAxesPositions(int positions[OMS_MAX_AXES] )
{
    return getAxesArray((char*) "AM PP;", positions);
}

asynStatus omsBaseController::getAxesArray(char* cmd, int positions[OMS_MAX_AXES] )
{
    //  maximum length of the ascii position / velocity values is 11 chars + comma
    //  -2147483648 <-> 2147483647
    // we expect numAxes values separated with commas
    // possible answers are "0,5000,0" ",,,," "0" ",,," (3 commas for 4 axes)

    const char* functionName="getAxesArray";
    asynStatus status = asynSuccess;
    char inputBuff[OMSINPUTBUFFERLEN] = "";
    char *start, *end, *stop;
    int i, intVal, again = 1;
    int count =0;
    size_t nRead;

    status = sendReceiveLock(cmd, inputBuff, sizeof(inputBuff), &nRead);
    if ((status == asynSuccess) && (strlen(inputBuff) >= (unsigned int)numAxes -1)) {
        start = inputBuff;
        stop = start + MIN(strlen(inputBuff), sizeof(inputBuff));
        for (i = 0; ((i < OMS_MAX_AXES) && again); ++i){
            if (*start == ','){
                positions[i] = 0;
                ++count;
                start += 1;
                if (*start == '\0'){
                    /* comma was last character */
                    ++count;
                    again=0;
                    if (i < OMS_MAX_AXES-1) positions[i+1] = 0;
                }
                continue;
            }
            intVal = strtol(start, &end, 10);
            if ((intVal == 0) && (start == end)) {
                again = 0;
            }
            else {
                positions[i] = intVal;
                ++count;
            }
            if (end >= stop) again = 0;
            start = end + 1;
        }
        if (count != numAxes) {
            errlogPrintf("%s:%s:%s: array string conversion error, count: %d, axes: %d, input: >%s<\n",
                                driverName, functionName, portName, count, numAxes, inputBuff);
            return asynError;
        }
    }
    else {
        if (status == asynSuccess){
            errlogPrintf("%s:%s:%s: read string too short %d - %s - %s\n",
                                driverName, functionName, portName, (int)strlen(inputBuff), cmd, inputBuff);
            return asynError;
        }
    }
    return status;
}


asynStatus omsBaseController::getSubstring(unsigned int number, char* inputBuffer, char *outBuffer, unsigned int outBufferLen)
{
    /* get the numbered substring from a comma-separated string */
    /* numbering starts with 0 */
    /* other as strtok accept a sequence of commas ,,, as empty substrings */
    char *start, *end, *pos, *tmpBuffer;
    unsigned int i, count;
    int doloop = 1;
    asynStatus status = asynError;


    if (strlen(inputBuffer) < number) return status;

    tmpBuffer = epicsStrDup(inputBuffer);
    start = tmpBuffer;
    end = start + strlen(start);
    for (i=0; i<=number && doloop==1; ++i){
        pos = strchr(start, ',');
        if (pos == NULL) {
            doloop = 0;
            count = MIN(strlen(start), outBufferLen-1);
        }
        else {
            count = MIN((unsigned int)(pos-start), outBufferLen-1);
            *pos = '\0';
        }
        if (i==number){
               strncpy(outBuffer, start, count);
                outBuffer[count]='\0';
                status = asynSuccess;
                break;
            }
        start = pos+1;
        if (start > end) doloop =0;
    }
    free (tmpBuffer);
    return status;
}

bool omsBaseController::watchdogOK()
{
    char inputBuff[10] = "";
    const char* functionName = "watchdogOK";
    size_t nRead;

    if (useWatchdog && (fwMinor >= 33)) {
        sendReceiveLock((char*) "#WS", (char*) inputBuff, sizeof(inputBuff), &nRead);
        if ((inputBuff[0] == '=') && (inputBuff[1] != '0')) {
            errlogPrintf("%s:%s:%s: *** CAUTION watchdog not running, disabling card ***\n",
                    driverName, functionName, portName);
            enabled = false;
            return false;
        }
    }
    return true;
}

