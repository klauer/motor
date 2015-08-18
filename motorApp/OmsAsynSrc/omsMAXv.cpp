/*
FILENAME...     omsMAXv.cpp
USAGE...        Pro-Dex OMS MAXv asyn motor controller support

*/

/*
 * based on drvMAXv.cc written by Ron Sluiter
 *
 *  Created on: 10/2010
 *      Author: eden
 */

#include <string.h>
#include <stdio.h>

#include "asynOctetSyncIO.h"
#include <epicsInterrupt.h>
#include <epicsExit.h>
#include "omsMAXv.h"
#ifdef HAS_IOOPS_H
#include "basicIoOps.h"
#endif

static const char *driverName = "omsMAXvAsyn";

#define MIN(a,b) ((a)<(b)? (a): (b))

#ifdef __GNUG__
    #ifdef      DEBUG
        #define Debug(l, f, args...) {if (l & motorMAXvdebug) \
                                  errlogPrintf(f, ## args);}
    #else
        #define Debug(l, f, args...)
    #endif
#else
    #define Debug
#endif
volatile int motorMAXvdebug = 0;
extern "C" {epicsExportAddress(int, motorMAXvdebug);}

char* omsMAXv::baseAddress = 0x0;
int   omsMAXv::numCards = 0;
epicsUInt32 omsMAXv::baseInterruptVector = OMS_INT_VECTOR;
epicsUInt8 omsMAXv::interruptLevel = OMS_INT_LEVEL;
epicsAddressType omsMAXv::addrType = atVMEA16;

static void writeReg8(volatile epicsUInt8 *a8, epicsUInt8 value)
{
#ifdef HAS_IOOPS_H
    out_8(a8, value);
#else
    *a8 = value;
#endif
}

static epicsUInt8 readReg8(volatile epicsUInt8 *a8)
{
#ifdef HAS_IOOPS_H
    return in_8(a8);
#else
    epicsUInt8 value;

    value = *a8;
    return(value);
#endif
}

static void writeReg16(volatile epicsUInt16 *a16, epicsUInt16 value)
{
#ifdef HAS_IOOPS_H
    out_be16(a16, value);
#else
    *a16 = value;
#endif
}

static epicsUInt16 readReg16(volatile epicsUInt16 *a16)
{
#ifdef HAS_IOOPS_H
    return in_be16(a16);
#else
    epicsUInt16 value;

    value = *a16;
    return(value);
#endif
}

static void writeReg32(volatile epicsUInt32 *a32, epicsUInt32 value)
{
#ifdef HAS_IOOPS_H
    out_be32(a32, value);
#else
    *a32 = value;
#endif
}

static epicsUInt32 readReg32(volatile epicsUInt32 *a32)
{
#ifdef HAS_IOOPS_H
    return in_be32(a32);
#else
    epicsUInt32 value;

    value = *a32;
    return(value);
#endif
}

static void readRegMem(char *bufptr, volatile char *memptr, int size)
{
    memcpy(bufptr, (const void *)memptr, size);
#ifdef HAS_IOOPS_H
    iobarrier_r();
#endif
}

void omsMAXv::InterruptHandler( void * param )
{
    omsMAXv* pController = (omsMAXv*) param;
    volatile struct MAXv_motor *pmotor = (MAXv_motor*) pController->getCardAddress();;
    STATUS1 status1_flag;
    static char errmsg[65];

    status1_flag.All = readReg32(&(pmotor->status1_flag.All));

    /* Motion done handling */
    if (status1_flag.Bits.done != 0) epicsEventSignal(pController->pollEventId_);

    if (status1_flag.Bits.cmndError)
    {
        strcpy(errmsg, "\nomsMAXv::InterruptHandler: command error - Port: ");
        strncat(errmsg, pController->getPortName(), sizeof(errmsg)-strlen(errmsg)-2);
        strcat(errmsg,"\n");
        epicsInterruptContextMessage(errmsg);
    }

    /* unset this bit to not clear the text_response bit */
    if (status1_flag.Bits.text_response != 0)  status1_flag.Bits.text_response = 0;

    /* Release IRQ's. Clear bits by writing a 1 */
    writeReg32(&(pmotor->status1_flag.All), status1_flag.All);

    /* do a dummy read to ensure that all previous writes, which may
     * have been queued in the VME bridge chip get processed
     */
    status1_flag.All = readReg32(&(pmotor->status1_flag.All));

}
omsMAXv::omsMAXv(const char* portName, int numAxes, int cardNo, const char* initString,
                int priority, int stackSize, int addParams)
    : omsBaseController(portName, numAxes, priority, stackSize, addParams)
{
    int vector = 0;

    if (baseInterruptVector != 0)
        vector = baseInterruptVector + cardNo;

    initialize(portName, numAxes, cardNo, initString, priority, stackSize, 1, vector, interruptLevel, addrType, addParams);
}
omsMAXv::omsMAXv(const char* portName, int numAxes, int slotNo, const char* initString, int priority,
        int stackSize, unsigned int vmeAddr, int vector, int intlevel, const char* addressType, int addParams)
    : omsBaseController(portName, numAxes, priority, stackSize, addParams)
{
    const char* functionName = "omsMAXv";
    epicsAddressType vmeAddrType = atVMEA16;

    if (vmeAddr < 0) {
        errlogPrintf("%s: invalid VME address: 0\n", functionName);
        return;
    }

    if (!strncmp(addressType, "A16",3)){
        vmeAddrType = atVMEA16;
        if (vmeAddr & 0xFFFF0FFF) {
            errlogPrintf("%s: invalid %s address: 0x%X.\n", functionName, addressType, vmeAddr);
            return;
        }
    }
    else if (!strncmp(addressType, "A24",3)){
        vmeAddrType = atVMEA24;
        if (vmeAddr & 0xFF00FFFF) {
            errlogPrintf("%s: invalid %s address: 0x%X.\n", functionName, addressType, vmeAddr);
            return;
        }
    }
    else if (!strncmp(addressType, "A32",3)){
        vmeAddrType = atVMEA32;
        if (vmeAddr & 0x00FFFFFF) {
            errlogPrintf("%s: invalid %s address: 0x%X.\n", functionName, addressType, vmeAddr);
            return;
        }
    }
    else if (strncmp(addressType, "CSR",3)){
        errlogPrintf("%s: VME CSR not supported\n", functionName);
        return;
    }
    else {
        errlogPrintf("%s: invalid address type, Please specify one of A16,A24,A32\n", functionName);
    }
    if (intlevel < 1 || intlevel > 6) {
        errlogPrintf("%s: invalid interrupt level %d, Please specify a value between 1 and 6\n", functionName, intlevel);
        return;
    }
    initialize(portName, numAxes, 0, initString, priority, stackSize, vmeAddr, vector, intlevel, vmeAddrType, addParams);
}

void omsMAXv::initialize(const char* portName, int numAxes, int cardNo, const char* initString, int prio,
                int stackSz, unsigned int vmeAddr, int intrVector, int level, epicsAddressType vmeAddrType, int paramCount)
{

    const char* functionName = "initialize";
    long status;
    void* probeAddr;

    Debug(32, "omsMAXv::initialize: start initialize\n" );

    controllerType = epicsStrDup("MAXv");

    // TODO check if cardNo has already been used
    this->cardNo = cardNo;
    if(cardNo < 0 || cardNo >= MAXv_NUM_CARDS){
        printf("invalid cardNo: %d", cardNo);
        return;
    }

    epicsUInt8 *startAddr;
    epicsUInt8 *endAddr;
    epicsUInt32 boardAddrSize = 0;

    if (vmeAddrType == atVMEA16)
        boardAddrSize = 0x1000;
    else if (vmeAddrType == atVMEA24)
        boardAddrSize = 0x10000;
    else if (vmeAddrType == atVMEA32)
        boardAddrSize = 0x1000000;

    // if vmeAddr == 1 Setup/Config is used and not Config2
    if (vmeAddr == 1)
        probeAddr = baseAddress + (cardNo * boardAddrSize);
    else
        probeAddr = (void*) vmeAddr;

    startAddr = (epicsUInt8 *) probeAddr;
    endAddr = startAddr + boardAddrSize;

    Debug(64, "motor_init: devNoResponseProbe() on addr %p\n", probeAddr);

    /* Scan memory space to assure card id */
    while (startAddr < endAddr) {
        status = devNoResponseProbe(vmeAddrType, (size_t) startAddr, 2);
        if (status != S_dev_addressOverlap) {
            errlogPrintf("%s:%s:%s: Card NOT found in specified address range! \n",
                                driverName, functionName, portName);
            enabled = false;
            return;
        }
        startAddr += (boardAddrSize / 10);
    }

    status = devRegisterAddress(controllerType, vmeAddrType,
                                (size_t) probeAddr, boardAddrSize,
                                (volatile void **) &pmotor);
    Debug(64, "motor_init: devRegisterAddress() status = %d\n", (int) status);

    if (status) {
        errlogPrintf("%s:%s:%s: Can't register address 0x%lx \n",
                        driverName, functionName, portName, (long unsigned int) probeAddr);
        return;
    }

    Debug(64, "motor_init: pmotor = %p\n", pmotor);

    int loopCount=15;
    FIRMWARE_STATUS fwStatus;
    fwStatus.All = readReg32(&(pmotor->firmware_status.All));
    while (loopCount && (fwStatus.Bits.initializing == 1)){
        Debug(1, "MAXv port %s still initializing; status = 0x%x\n",
                portName, (unsigned int) pmotor->firmware_status.All);
        epicsThreadSleep(0.2);
        fwStatus.All = readReg32(&(pmotor->firmware_status.All));
        --loopCount;
    }

    Debug(64, "motor_init: check if card is ready\n");

    fwStatus.All = readReg32(&(pmotor->firmware_status.All));
    if (fwStatus.Bits.running == 0)
        errlogPrintf("MAXv port %s is NOT running; status = 0x%x\n",
                portName, (unsigned int) pmotor->firmware_status.All);

    Debug(64, "motor_init: init card\n");

    fwStatus.All = readReg32(&(pmotor->firmware_status.All));
    Debug(64, "motor_init: firmware status register: 0x%x\n", fwStatus.All);

    writeReg32(&(pmotor->IACK_vector), intrVector);

    writeReg32(&(pmotor->status1_flag.All), 0xFFFFFFFF);
    writeReg32(&(pmotor->status2_flag), 0xFFFFFFFF);
    /* Disable all interrupts */
    writeReg32(&(pmotor->status1_irq_enable.All), 0);
    writeReg32(&(pmotor->status2_irq_enable), 0);

    Debug(64, "motor_init: clear all interrupt\n");
    //sendOnly("IC");

    Debug(64, "motor_init: firmware version\n");

    /* get FirmwareVersion */
    if(getFirmwareVersion() != asynSuccess) {
        errlogPrintf("%s:%s:%s: unable to talk to controller card %d\n",
                        driverName, functionName, portName, cardNo);
        return;
    }

    if (fwMinor < 30 ){
        errlogPrintf("%s:%s:%s: This Controllers Firmware Version %d.%d is not supported, version 1.30 or higher is mandatory\n",
                        driverName, functionName, portName, fwMajor, fwMinor);
    }

    Debug(64, "motor_init: send init string\n");

    if( Init(initString, 1) != asynSuccess) {
        errlogPrintf("%s:%s:%s: unable to send initstring to controller card %d\n",
                        driverName, functionName, portName, cardNo);
        return;
    }

    useWatchdog = true;

    if (watchdogOK()) {
        Debug(64, "motor_init: enable interrupts ( vector=%d, level=%d) \n", intrVector, level);
        /* Enable interrupt-when-done if selected */
        if (intrVector) motorIsrSetup((unsigned int)intrVector, level);
    }
    else
        return;

    if (epicsAtExit(&omsMAXv::resetOnExit, this))
        errlogPrintf("%s:%s:%s: card %d, unable to register exit function\n",
                        driverName, functionName, portName, cardNo);

    return;
}

void omsMAXv::resetIntr()
{
    enabled=false;
    writeReg32(&(pmotor->status1_irq_enable.All), 0);
}

asynStatus omsMAXv::sendOnly(const char *outputBuff)
{
    STATUS1 flag1;
    const char* functionName = "sendOnly";
    int len = strlen(outputBuff);
    double timeout = 0.01;
    epicsUInt32 getIndex, putIndex;

    if (!enabled) return asynError;
    Debug(16, "omsMAXv::send: sending: %s \n", outputBuff);

    if (len > (BUFFER_SIZE-1))
    {
        errlogPrintf("%s:%s:%s: message too long: %d character\n",
                        driverName, functionName, portName, len);
        return asynError;
    }

    /* see if junk at input port - should not be any data available */
    getIndex = readReg32(&(pmotor->inGetIndex));
    putIndex = readReg32(&(pmotor->inPutIndex));
    int flushTime = 0;
    while ((getIndex != putIndex) && (flushTime < 100))
    {
        // flush cards response Buffer
#ifdef DEBUG
        int deltaIndex = ((epicsUInt16)putIndex) - ((epicsUInt16)getIndex);
        Debug(32, "%s:%s:%s: flushing %d characters\n",
                driverName, functionName, portName, (((deltaIndex < 0) ? BUFFER_SIZE +
                        deltaIndex : deltaIndex)));
#endif
        putIndex = readReg32(&(pmotor->inPutIndex));
	writeReg32(&(pmotor->inGetIndex), putIndex);
        //pmotor->status1_flag.Bits.text_response = 0;
        epicsUInt32 t_status = readReg32(&(pmotor->status1_flag.All));
        writeReg32(&(pmotor->status1_flag.All), (t_status & 0xfdffffff));
        flag1.All = readReg32(&(pmotor->status1_flag.All));
        writeReg32(&(pmotor->status1_flag.All), flag1.All);
        writeReg32(&(pmotor->msg_semaphore), 0);
        epicsThreadSleep(timeout);
        getIndex = readReg32(&(pmotor->inGetIndex));
        putIndex = readReg32(&(pmotor->inPutIndex));
        flushTime++;
        if (flushTime == 100 ) {
        	Debug(1, "%s:%s:%s: unable to flush more than 100 strings\n", driverName, functionName, portName);
        	return asynError;
        }
    }

    putIndex = readReg32(&(pmotor->outPutIndex));
    getIndex = readReg32(&(pmotor->outGetIndex));

    for (int i = 0; (i < len); i++) {
        writeReg8(&(pmotor->outBuffer[putIndex++]), outputBuff[i]);
        if (putIndex >= BUFFER_SIZE) putIndex = 0;
    }

    writeReg32(&(pmotor->outPutIndex), putIndex);	/* Message Sent */

    int count = 0, prevdeltaIndex = 0, index = 0;
    int maxcount = (int)(0.1 / epicsThreadSleepQuantum());
    // skip busy-waiting for small epicsThreadSleepQuantum
    if (epicsThreadSleepQuantum() <= 0.01) index = 100;
    putIndex = readReg32(&(pmotor->outPutIndex));
    getIndex = readReg32(&(pmotor->outGetIndex));
    int deltaIndex = ((epicsUInt16)putIndex) - ((epicsUInt16)getIndex);
    while ((deltaIndex != 0) && (count <= maxcount))
    {
        putIndex = readReg32(&(pmotor->outPutIndex));
        getIndex = readReg32(&(pmotor->outGetIndex));
        deltaIndex  = ((epicsUInt16)putIndex) - ((epicsUInt16)getIndex);
        //  do busy-waiting but not more than 100 times
        while ((index < 100) && (deltaIndex != 0)){
            putIndex = readReg32(&(pmotor->outPutIndex));
            getIndex = readReg32(&(pmotor->outGetIndex));
            deltaIndex  = ((epicsUInt16)putIndex) - ((epicsUInt16)getIndex);
            ++index;
        }
        if ((index >= 100) && (deltaIndex != 0)) epicsThreadSleep(timeout);
        if (deltaIndex == prevdeltaIndex)
            ++count;
        else
            count = 0;
        prevdeltaIndex = deltaIndex;
    };

    if (deltaIndex != 0) {
        Debug(1, "%s:%s:%s: Timeout\n", driverName, functionName, portName);
        return asynTimeout;
    }

    Debug(64, "omsMAXv::send: done\n");

    return asynSuccess;
}
/**
 * read just one line of input
 */
asynStatus omsMAXv::sendReceive(const char *outputBuff, char *inputBuff, unsigned int inputSize)
{
    static const char* functionName = "sendReceive";

    STATUS1 flag1;
    epicsUInt16 getIndex, putIndex;
    size_t bufsize;
    size_t usedSpace = 0;
    char *start, *end;
    int itera = 0;
    asynStatus status;

    if (!enabled) return asynError;

    status = sendOnly(outputBuff);
    if (status != asynSuccess) return status;

    if (inputSize <= 0) return status;

    *inputBuff = '\0';

    double time = 0.0;
    double timeout = 0.1;
    // skip busy-waiting for small epicsThreadSleepQuantum
    if (epicsThreadSleepQuantum() <= 0.01) itera = 2001;
    flag1.All = readReg32(&(pmotor->status1_flag.All));
    while ((flag1.Bits.text_response == 0) && (time < timeout)){
        Debug(32, "%s:%s:%s: Waiting for reponse, itera:%d\n",
                driverName, functionName, portName, itera);
        //  busy-waiting but not more than 2000 times
        if (itera > 2000){
            time += epicsThreadSleepQuantum();
            epicsThreadSleep(epicsThreadSleepQuantum());
        }
        flag1.All = readReg32(&(pmotor->status1_flag.All));
        itera++;
    }

    flag1.All = readReg32(&(pmotor->status1_flag.All));
    if (flag1.Bits.text_response == 0)
    {
        Debug(1, "%s:%s:%s: Timeout occurred , %s\n",
        		driverName, functionName, portName, outputBuff);
        return asynTimeout;
    }

    getIndex = readReg32(&(pmotor->inGetIndex));
    putIndex = readReg32(&(pmotor->inPutIndex));
    bufsize  = putIndex - getIndex;
    start  = (char *) &pmotor->inBuffer[getIndex];
    end    = (char *) &pmotor->inBuffer[putIndex];

    if (start < end) {   /* Test for message wraparound in buffer. */
    	usedSpace = MIN(bufsize, inputSize);
        readRegMem(inputBuff, start, usedSpace);
    }
    else
    {
        bufsize += BUFFER_SIZE;
        size_t firstPart = ((char *) &pmotor->inBuffer[BUFFER_SIZE]) - start;

        usedSpace = MIN(firstPart, inputSize);
        readRegMem(inputBuff, start, usedSpace);
        size_t copySize = MIN(bufsize - firstPart, inputSize - usedSpace);
        readRegMem((inputBuff + usedSpace), (char *) &(pmotor->inBuffer[0]), copySize);
        usedSpace += copySize;
    }

    inputBuff[usedSpace - 1]= '\0';

    getIndex += bufsize;
    if (getIndex >= BUFFER_SIZE)
        getIndex -= BUFFER_SIZE;

    putIndex = readReg32(&(pmotor->inPutIndex));
    while (getIndex != putIndex)
    {
        epicsUInt8 t_inBuff = readReg8(&(pmotor->inBuffer[getIndex]));
        Debug(2, "readbuf(): flushed - %d\n", pmotor->inBuffer[getIndex]);
        if (++getIndex > BUFFER_SIZE)
            getIndex = 0;
    }
    //pmotor->status1_flag.Bits.text_response = 0;
    flag1.All = readReg32(&(pmotor->status1_flag.All));
    writeReg32(&(pmotor->status1_flag.All), (flag1.All & 0xfdffffff));

    writeReg32(&(pmotor->inGetIndex), (epicsUInt32) getIndex);
    flag1.All = readReg32(&(pmotor->status1_flag.All));
    writeReg32(&(pmotor->status1_flag.All), flag1.All);
    writeReg32(&(pmotor->msg_semaphore), 0);

    Debug(16, "omsMAXv::sendReceive: received %s\n", inputBuff);
    return asynSuccess;
}


void omsMAXv::motorIsrSetup(volatile unsigned int vector, volatile epicsUInt8 level)
{
    const char* functionName = "motorIsrSetup";
    STATUS1 status1_irq;
    long status;

    Debug(64, "omsMAXv::isrSetup: start\n");

    status = pdevLibVirtualOS->pDevConnectInterruptVME( vector, &omsMAXv::InterruptHandler, this);

    if (status) {
        errlogPrintf("%s:%s:%s: Can't connect to interrupt vector %d\n",
                driverName, functionName, portName, vector);
        return;
    }

    Debug(64, "omsMAXv::isrSetup: set level\n");
    status = devEnableInterruptLevel(intVME, level);
    if (status) {
        errlogPrintf("%s:%s:%s: Can't enable interrupt level %d\n",
                driverName, functionName, portName, level);
        return;
    }

    /* Setup card for interrupt-on-done */
    status1_irq.All = 0;
    status1_irq.Bits.done = 0xFF;
    status1_irq.Bits.cmndError = 1;

    writeReg32(&(pmotor->status1_irq_enable.All), status1_irq.All);   /* Enable interrupts. */
    writeReg32(&(pmotor->status2_irq_enable), 0x0);

    Debug(64, "omsMAXv::isrSetup: done\n");
    return;
}

extern "C" int omsMAXvSetup(
           int num_cards,        /* maximum number of cards in rack */
           int addr_type,        /* VME address type; 16 -> A16, 24 -> A24 or 32 -> A32. */
           unsigned int addrs,   /* Base Address. */
           unsigned int vector,  /* noninterrupting(0), valid vectors(64-255) */
           int int_level)        /* interrupt level (1-6) */

{
    const char* functionName = "omsMAXvSetup";
    if (num_cards < 1 || num_cards > MAXv_NUM_CARDS)
    {
        errlogPrintf("\n%s: number of cards specified = %d but must be 1 <= number <= %d\n",
                functionName, num_cards, MAXv_NUM_CARDS);
        epicsThreadSleep(5.0);
        return 1;
    }
    omsMAXv::numCards = num_cards;
    omsMAXv::baseAddress = (char *) addrs;

    switch (addr_type)
    {
    case 16:
        omsMAXv::addrType = atVMEA16;
        if ((epicsUInt32) addrs & 0xFFFF0FFF) {
            errlogPrintf("%s: invalid A%d address: 0x%X.\n", functionName, addr_type, (epicsUInt32) addrs);
            return 1;
        }
        break;
    case 24:
        omsMAXv::addrType = atVMEA24;
        if ((epicsUInt32) addrs & 0xFF00FFFF) {
            errlogPrintf("%s: invalid A%d address: 0x%X.\n", functionName, addr_type, (epicsUInt32) addrs);
            return 1;
        }
        break;
    case 32:
        omsMAXv::addrType = atVMEA32;
        if ((epicsUInt32) addrs & 0x00FFFFFF) {
            errlogPrintf("%s: invalid A%d address: 0x%X.\n", functionName, addr_type, (epicsUInt32) addrs);
            return 1;
        }
        break;
    default:
        errlogPrintf("%s: invalid address type, Please specify one of 16/24/32 for VME A16/A24/A32\n", functionName);
        return 1;
        break;
    }

    if ((vector != 0) && (vector < 64 || vector > 255)) {
        errlogPrintf("%s: invalid address type, Please specify a value between 64 and 255\n", functionName);
        epicsThreadSleep(5.0);
        return 1;
    }
    omsMAXv::baseInterruptVector = vector;

    if (int_level < 1 || int_level > 6) {
        errlogPrintf("%s: invalid interrupt level, Please specify a value between 1 and 6\n", functionName);
        epicsThreadSleep(5.0);
        return 1;
    }
    omsMAXv::interruptLevel = int_level;

    return 0;
}

extern "C" int omsMAXvConfig(
           int cardNo,                /* card no, starts with 0*/
           const char *portName,      /* MAXv Motor Asyn Port name */
           int numAxes,               /* Number of axes this controller supports */
           int movingPollPeriod,      /* Time to poll (msec) when an axis is in motion */
           int idlePollPeriod,        /* Time to poll (msec) when an axis is idle. 0 for no polling */
           const char *initString)    /* Init String sent to card */
{
    omsMAXv *pController = new omsMAXv(portName, numAxes, cardNo, initString, 0, 0, 0);
    pController->startPoller((double)movingPollPeriod, (double)idlePollPeriod, 10);
    return 0;
}

/*
 * extended MAXv configuration, which may be used instead of omsMAXvConfig,
 * if more details need to be specified.
 * omsMAXvConfig2 does not need and ignores omsMAXvSetup
 */
extern "C" int omsMAXvConfig2(
           int slotNo,                /* VME slot no of MAXv card*/
           const char* addr_type,     /* VME address type; "A16", "A24" or "A32" */
           unsigned int addrs,        /* Board Address */
           unsigned int vector,       /* Interrupt Vector: noninterrupting(0), (64-255) */
           int int_level,             /* interrupt level (1-6) */
           const char *portName,      /* MAXv Motor Asyn Port name */
           int numAxes,               /* Number of axes this controller supports */
           int priority,              /* priority of PollerTask (0 => epicsThreadPriorityMedium)*/
           int stackSize,             /* stackSize of PollerTask (0 => epicsThreadStackMedium)  */
           int movingPollPeriod,      /* Time to poll (msec) when an axis is in motion */
           int idlePollPeriod,        /* Time to poll (msec) when an axis is idle. 0 for no polling */
           const char *initString)    /* Init String sent to card */
{
    omsMAXv *pController = new omsMAXv(portName, numAxes, slotNo, initString, priority,
                                          stackSize, addrs, vector, int_level, addr_type, 0);
    pController->startPoller((double)movingPollPeriod, (double)idlePollPeriod, 10);
    return 0;
}

/* Code for iocsh registration */
/* omsMAXvSetup */
static const iocshArg setupArg0 = {"Max. controller count", iocshArgInt};
static const iocshArg setupArg1 = {"VME address type", iocshArgInt};
static const iocshArg setupArg2 = {"Base Address on 4K (0x1000) boundary", iocshArgInt};
static const iocshArg setupArg3 = {"noninterrupting(0), valid vectors(64-255)", iocshArgInt};
static const iocshArg setupArg4 = {"interrupt level (1-6)", iocshArgInt};
static const iocshArg * const OmsSetupArgs[5] = { &setupArg0, &setupArg1, &setupArg2,
                                                  &setupArg3, &setupArg4};
static const iocshFuncDef setupMAXv = {"omsMAXvSetup", 5, OmsSetupArgs};
static void setupMAXvCallFunc(const iocshArgBuf *args)
{
    omsMAXvSetup(args[0].ival, args[1].ival, args[2].ival, args[3].ival, args[4].ival);
}

/* omsMAXvConfig */
static const iocshArg configArg0 = {"number of card", iocshArgInt};
static const iocshArg configArg1 = {"asyn motor port name", iocshArgString};
static const iocshArg configArg2 = {"number of axes", iocshArgInt};
static const iocshArg configArg3 = {"moving poll rate", iocshArgInt};
static const iocshArg configArg4 = {"idle poll rate", iocshArgInt};
static const iocshArg configArg5 = {"initstring", iocshArgString};
static const iocshArg * const configArgs[6] = {&configArg0, &configArg1, &configArg2,
                                               &configArg3, &configArg4, &configArg5 };
static const iocshFuncDef configMAXv = {"omsMAXvConfig", 6, configArgs};
static void configMAXvCallFunc(const iocshArgBuf *args)
{
    omsMAXvConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval);
}

/* omsMAXvConfig2 */
static const iocshArg config2Arg0 = {"Slot number", iocshArgInt};
static const iocshArg config2Arg1 = {"Address type: A16,A24,A32", iocshArgString};
static const iocshArg config2Arg2 = {"Board Address on 4K (0x1000) boundary", iocshArgInt};
static const iocshArg config2Arg3 = {"Interrupt Vector: noninterrupting(0), (64-255)", iocshArgInt};
static const iocshArg config2Arg4 = {"Interrupt level (1-6)", iocshArgInt};
static const iocshArg config2Arg5 = {"Asyn motor port name", iocshArgString};
static const iocshArg config2Arg6 = {"Number of axes", iocshArgInt};
static const iocshArg config2Arg7 = {"Task priority: 0 => medium", iocshArgInt};
static const iocshArg config2Arg8 = {"Stack size: 0 => medium", iocshArgInt};
static const iocshArg config2Arg9 = {"Moving poll rate", iocshArgInt};
static const iocshArg config2Arg10 = {"Idle poll rate", iocshArgInt};
static const iocshArg config2Arg11 = {"Initstring", iocshArgString};
static const iocshArg * const config2Args[12] = {&config2Arg0, &config2Arg1, &config2Arg2, &config2Arg3, &config2Arg4,
        &config2Arg5, &config2Arg6, &config2Arg7, &config2Arg8, &config2Arg9, &config2Arg10, &config2Arg11};
static const iocshFuncDef config2MAXv = {"omsMAXvConfig2", 12, config2Args};
static void config2MAXvCallFunc(const iocshArgBuf *args)
{
    omsMAXvConfig2(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival, args[5].sval,
                   args[6].ival, args[7].ival, args[8].ival, args[9].ival, args[10].ival, args[11].sval);
}

static void OmsMAXvAsynRegister(void)
{
    iocshRegister(&setupMAXv, setupMAXvCallFunc);
    iocshRegister(&configMAXv, configMAXvCallFunc);
    iocshRegister(&config2MAXv, config2MAXvCallFunc);
}

epicsExportRegistrar(OmsMAXvAsynRegister);

