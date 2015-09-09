TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = TwinCATmotor
# TwinCATmotor.dbd will be created and installed
DBD += TwinCATmotor.dbd

# TwinCATmotor.dbd will be made up from these files:
TwinCATmotor_DBD += base.dbd

# Include dbd files from all support applications:
#TwinCATmotor_DBD += xxx.dbd
TwinCATmotor_DBD += asyn.dbd
TwinCATmotor_DBD += drvAsynIPPort.dbd
TwinCATmotor_DBD += motorSupport.dbd
TwinCATmotor_DBD += TwinCATmotorController.dbd

# Add source files
TwinCATmotor_SRCS += TwinCATmotorController.cpp
TwinCATmotor_SRCS += TwinCATmotorAxis.cpp

# Add all the support libraries needed by this IOC
TwinCATmotor_LIBS += asyn
TwinCATmotor_LIBS += motor

# TwinCATmotor_registerRecordDeviceDriver.cpp derives from TwinCATmotor.dbd
TwinCATmotor_SRCS += TwinCATmotor_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
TwinCATmotor_SRCS_DEFAULT += TwinCATmotorMain.cpp
TwinCATmotor_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#TwinCATmotor_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
TwinCATmotor_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

