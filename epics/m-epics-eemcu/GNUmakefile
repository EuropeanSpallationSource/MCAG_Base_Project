include ${EPICS_ENV_PATH}/module.Makefile

SOURCES = \
  eemcuApp/src/eemcuAxis.cpp \
  eemcuApp/src/eemcuController.cpp \
  eemcuApp/src/eemcuMain.cpp

TEMPLATES = \
  eemcuApp/Db/eemcu.template

MISCS = \
  iocBoot/ioceemcu/SolAxis-48.cfg \
  iocBoot/ioceemcu/SolAxis-48-1.cfg

USR_DEPENDENCIES = asyn,4.27
USR_DEPENDENCIES += motor,6.10.6-ESS
