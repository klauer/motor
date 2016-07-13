TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MSmartO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += SmartMotorAsyn

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += devSmartMotorAsynSupport.dbd

# The following are compiled and added to the Support library
SmartMotorAsyn_SRCS += SmartController.cpp 
SmartMotorAsyn_SRCS += SmartAxisBase.cpp 
SmartMotorAsyn_SRCS += SmartAxis4.cpp 
SmartMotorAsyn_SRCS += SmartAxis5.cpp 
SmartMotorAsyn_SRCS += VirtualAxis.cpp 
SmartMotorAsyn_LIBS += motor
SmartMotorAsyn_LIBS += asyn
SmartMotorAsyn_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

