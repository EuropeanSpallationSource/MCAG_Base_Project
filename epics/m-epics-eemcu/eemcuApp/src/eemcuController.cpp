/*
FILENAME... eemcuController.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#include <epicsExport.h>
#include "eemcu.h"

/** Creates a new eemcuController object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName     The name of the drvAsynSerialPort that was created previously to connect to the eemcu controller
  * \param[in] numAxes           The number of axes that this controller supports
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
eemcuController::eemcuController(const char *portName, const char *MotorPortName, int numAxes,
                                               double movingPollPeriod,double idlePollPeriod)
  :  asynMotorController(portName, numAxes, NUM_VIRTUAL_MOTOR_PARAMS,
                         0, // No additional interfaces beyond those in base class
                         0, // No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)  // Default priority and stack size
{
  asynStatus status;

  createParam(eemcuErrString,           asynParamInt32,       &eemcuErr_);
  createParam(eemcuErrIdString,         asynParamInt32,       &eemcuErrId_);

  createParam(eemcuProcHomString,       asynParamInt32,       &eemcuProcHom_);
  createParam(eemcuErrRstString,        asynParamInt32,       &eemcuErrRst_);
  createParam(eemcuJVELString,               asynParamFloat64,     &eemcuJVEL_);

#ifdef CREATE_MOTOR_REC_RESOLUTION
  /* Latest asynMotorController does this, but not the version in 6.81 (or 6.9x) */
  createParam(motorRecResolutionString,        asynParamFloat64,      &motorRecResolution_);
  createParam(motorRecDirectionString,           asynParamInt32,      &motorRecDirection_);
  createParam(motorRecOffsetString,            asynParamFloat64,      &motorRecOffset_);
#endif

  /* Connect to eemcu controller */
  status = pasynOctetSyncIO->connect(MotorPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "cannot connect to motor controller\n");
  }
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}


/** Creates a new eemcuController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] MotorPortName  The name of the drvAsynIPPPort that was created previously to connect to the eemcu controller
  * \param[in] numAxes           The number of axes that this controller supports (0 is not used)
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int eemcuCreateController(const char *portName, const char *MotorPortName, int numAxes,
                                            int movingPollPeriod, int idlePollPeriod)
{
  new eemcuController(portName, MotorPortName, 1+numAxes, movingPollPeriod/1000., idlePollPeriod/1000.);
  return(asynSuccess);
}

/** Writes a string to the controller and reads a response.
  * Disconnects in case of error
  */
asynStatus eemcuController::writeReadOnErrorDisconnect(void)
{
  size_t nwrite = 0;
  asynStatus status;
  int eomReason;
  size_t outlen = strlen(outString_);
  size_t nread;
  status = pasynOctetSyncIO->writeRead(pasynUserController_, outString_, outlen,
                                       inString_, sizeof(inString_),
                                       DEFAULT_CONTROLLER_TIMEOUT,
                                       &nwrite, &nread, &eomReason);
  if ((status == asynTimeout) ||
      (!nread && (eomReason & ASYN_EOM_END)))

{
#if 1
    asynInterface *pasynInterface = NULL;
    asynCommon     *pasynCommon = NULL;
    pasynInterface = pasynManager->findInterface(pasynUserController_,
                                                 asynCommonType,
                                                 0 /* FALSE */);
    if (pasynInterface) {
      pasynCommon = (asynCommon *)pasynInterface->pinterface;
      status = pasynCommon->disconnect(pasynInterface->drvPvt,
                                       pasynUserController_);
      if (status != asynSuccess) {
        asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s status=%s (%d)\n",
                  outString_, pasynManager->strStatus(status), (int)status);
      }
    } else {
      asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "pasynInterface=%p pasynCommon=%p\n",
                pasynInterface, pasynCommon);
    }
#endif
    asynPrint(pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s nread=%lu status=%s (%d)\n",
              outString_,(unsigned long)nread,
              pasynManager->strStatus(status), status);
    handleStatusChange(asynError);
    return asynError; /* TimeOut -> Error */
  }
  return status;
}

void eemcuController::handleStatusChange(asynStatus status)
{
  int i;
  for (i=0; i<numAxes_; i++) {
    eemcuAxis *pAxis=getAxis(i);
    if (!pAxis) continue;
    pAxis->handleStatusChange(status);
  }
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information it calls asynMotorController::report()
  */
void eemcuController::report(FILE *fp, int level)
{
  fprintf(fp, "Twincat motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n",
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an eemcuAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
eemcuAxis* eemcuController::getAxis(asynUser *pasynUser)
{
  return static_cast<eemcuAxis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an eemcuAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
eemcuAxis* eemcuController::getAxis(int axisNo)
{
  return static_cast<eemcuAxis*>(asynMotorController::getAxis(axisNo));
}


asynStatus eemcuController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;
  eemcuAxis *pAxis;
  pAxis = getAxis(pasynUser);
  if (!pAxis) return asynError;

  (void)pAxis->setIntegerParam(function, value);
  return asynMotorController::writeInt32(pasynUser, value);
}

/** Code for iocsh registration */
static const iocshArg eemcuCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg eemcuCreateControllerArg1 = {"EPICS ASYN TCP motor port name", iocshArgString};
static const iocshArg eemcuCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg eemcuCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg eemcuCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const eemcuCreateControllerArgs[] = {&eemcuCreateControllerArg0,
                                                             &eemcuCreateControllerArg1,
                                                             &eemcuCreateControllerArg2,
                                                             &eemcuCreateControllerArg3,
                                                             &eemcuCreateControllerArg4};
static const iocshFuncDef eemcuCreateControllerDef = {"eemcuCreateController", 5, eemcuCreateControllerArgs};
static void eemcuCreateContollerCallFunc(const iocshArgBuf *args)
{
  eemcuCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}


/* eemcuCreateAxis */
static const iocshArg eemcuCreateAxisArg0 = {"Controller port name", iocshArgString};
static const iocshArg eemcuCreateAxisArg1 = {"Axis number", iocshArgInt};
static const iocshArg eemcuCreateAxisArg2 = {"axisFlags", iocshArgInt};
static const iocshArg eemcuCreateAxisArg3 = {"axisOptionsStr", iocshArgString};
static const iocshArg * const eemcuCreateAxisArgs[] = {&eemcuCreateAxisArg0,
                                                       &eemcuCreateAxisArg1,
                                                       &eemcuCreateAxisArg2,
                                                       &eemcuCreateAxisArg3};
static const iocshFuncDef eemcuCreateAxisDef = {"eemcuCreateAxis", 4, eemcuCreateAxisArgs};
static void eemcuCreateAxisCallFunc(const iocshArgBuf *args)
{
  eemcuCreateAxis(args[0].sval, args[1].ival, args[2].ival, args[3].sval);
}

static void eemcuControllerRegister(void)
{
  iocshRegister(&eemcuCreateControllerDef, eemcuCreateContollerCallFunc);
  iocshRegister(&eemcuCreateAxisDef,       eemcuCreateAxisCallFunc);
}

extern "C" {
  epicsExportRegistrar(eemcuControllerRegister);
}
