TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MLinmotO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += LinmotMotor

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += LinmotMotorSupport.dbd

# The following are compiled and added to the Support library
LinmotMotor_SRCS += LinmotMotorDriver.cpp

LinmotMotor_LIBS += motor
LinmotMotor_LIBS += asyn
LinmotMotor_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

