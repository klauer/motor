TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MEtherCATO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += EtherCATMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += EtherCATMotorSupport.dbd

# The following are compiled and added to the Support library
EtherCATMotor_SRCS += EtherCATMotorDriver.cpp

EtherCATMotor_LIBS += motor
EtherCATMotor_LIBS += asyn
EtherCATMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

