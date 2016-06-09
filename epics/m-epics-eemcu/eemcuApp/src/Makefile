TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = eemcu
# eemcu.dbd will be created and installed
DBD += eemcu.dbd

# eemcu.dbd will be made up from these files:
eemcu_DBD += base.dbd

# Include dbd files from all support applications:
#eemcu_DBD += xxx.dbd
eemcu_DBD += asyn.dbd
eemcu_DBD += drvAsynIPPort.dbd
eemcu_DBD += axisSupport.dbd
eemcu_DBD += eemcuController.dbd

# Add source files
eemcu_SRCS += eemcuController.cpp
eemcu_SRCS += eemcuAxis.cpp

# Add all the support libraries needed by this IOC
eemcu_LIBS += asyn
eemcu_LIBS += axis

# eemcu_registerRecordDeviceDriver.cpp derives from eemcu.dbd
eemcu_SRCS += eemcu_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
eemcu_SRCS_DEFAULT += eemcuMain.cpp
eemcu_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#eemcu_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
eemcu_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

