/*
FILENAME... TwinCATmotorAxis.cpp
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <epicsThread.h>

#include "TwinCATmotor.h"

#ifndef ASYN_TRACE_INFO
#define ASYN_TRACE_INFO      0x0040
#endif

//
// These are the TwinCATmotorAxis methods
//

/** Creates a new TwinCATmotorAxis object.
  * \param[in] pC Pointer to the TwinCATmotorController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 1 to pC->numAxes_. (0 is not used)
  *
  *
  * Initializes register numbers, etc.
  */
TwinCATmotorAxis::TwinCATmotorAxis(TwinCATmotorController *pC, int axisNo,
				   int axisFlags, const char *axisOptionsStr)
  : asynMotorAxis(pC, axisNo),
    pC_(pC)
{
  memset(&drvlocal, 0, sizeof(drvlocal));
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  drvlocal.axisFlags = axisFlags;
  if (axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
    setIntegerParam(pC->motorStatusGainSupport_, 1);
  }
  if (axisOptionsStr && axisOptionsStr[0]) {
    const char * const encoder_is_str = "encoder=";

    char *pOptions = strdup(axisOptionsStr);
    char *pThisOption = pOptions;
    char *pNextOption = pOptions;

    while (pNextOption && pNextOption[0]) {
      pNextOption = strchr(pNextOption, ';');
      if (pNextOption) {
        *pNextOption = '\0'; /* Terminate */
        pNextOption++;       /* Jump to (possible) next */
      }
      if (!strncmp(pThisOption, encoder_is_str, strlen(encoder_is_str))) {
        pThisOption += strlen(encoder_is_str);
        drvlocal.externalEncoderStr = strdup(pThisOption);
        setIntegerParam(pC->motorStatusHasEncoder_, 1);
      }
    }
    free(pOptions);
  }

  pC_->wakeupPoller();
}


extern "C" int TwinCATmotorCreateAxis(const char *TwinCATmotorName, int axisNo,
				      int axisFlags, const char *axisOptionsStr)
{
  TwinCATmotorController *pC;

  pC = (TwinCATmotorController*) findAsynPortDriver(TwinCATmotorName);
  if (!pC)
  {
    printf("Error port %s not found\n", TwinCATmotorName);
    return asynError;
  }
  pC->lock();
  new TwinCATmotorAxis(pC, axisNo, axisFlags, axisOptionsStr);
  pC->unlock();
  return asynSuccess;
}

/** Connection status is changed, the dirty bits must be set and
 *  the values in the controller must be updated
  * \param[in] AsynStatus status
  *
  * Sets the dirty bits
  */
void TwinCATmotorAxis::handleStatusChange(asynStatus newStatus)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
            "TwinCATmotorAxis::handleStatusChange status=%s (%d)\n",
            pasynManager->strStatus(newStatus), (int)newStatus);
  if (newStatus != asynSuccess) {
    memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  } else {
    asynStatus status = asynSuccess;
    if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_CREATE_AXIS) {
	/* Enable the amplifier when the axis is created,
	   but wait until we have a connection to the controller.
	   After we lost the connection, Re-enable the amplifier
	   See AMPLIFIER_ON_FLAG */
      status = enableAmplifier(1);
    }
    /*  Enable "Target Position Monitoring" */
    if (status == asynSuccess) status = setValueOnAxis(501, 0x4000, 0x15, 1);
  }
}

/** Reports on status of the axis
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * After printing device-specific information calls asynMotorAxis::report()
  */
void TwinCATmotorAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  axis %d\n", axisNo_);
 }

  // Call the base class method
  asynMotorAxis::report(fp, level);
}


/** Writes a command to the axis, and expects a logical ack from the controller
  * Outdata is in pC_->outString_
  * Indata is in pC_->inString_
  * The communiction is logged ASYN_TRACE_INFO
  *
  * When the communictaion fails ot times out, writeReadOnErrorDisconnect() is called
  */
asynStatus TwinCATmotorAxis::writeReadACK(void)
{
  asynStatus status = pC_->writeReadOnErrorDisconnect();
  switch (status) {
    case asynError:
      return status;
    case asynSuccess:
      if (strcmp(pC_->inString_, "OK")) {
        status = asynError;
        asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                  "out=%s in=%s return=%s (%d)\n",
                  pC_->outString_, pC_->inString_,
                  pasynManager->strStatus(status), (int)status);
        return status;
      }
    default:
      break;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
            "out=%s in=%s status=%s (%d) oldPosition=%f\n",
            pC_->outString_, pC_->inString_,
            pasynManager->strStatus(status), (int)status,
            drvlocal.oldPosition);
  return status;
}


/** Sets an integer or boolean value on an axis
  * the values in the controller must be updated
  * \param[in] name of the variable to be updated
  * \param[in] value the (integer) variable to be updated
  *
  */
asynStatus TwinCATmotorAxis::setValueOnAxis(const char* var, int value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%d", axisNo_, var, value);
  return writeReadACK();
}

/** Sets an integer or boolean value on an axis, read it back and retry if needed
  * the values in the controller must be updated
  * \param[in] name of the variable to be updated
  * \param[in] name of the variable where we can read back
  * \param[in] value the (integer) variable to be updated
  * \param[in] number of retries
  */
asynStatus TwinCATmotorAxis::setValueOnAxisVerify(const char *var, const char *rbvar,
                                                  int value, unsigned int retryCount)
{
  asynStatus status = asynSuccess;
  unsigned int counter = 0;
  int rbvalue = 0 - value;
  while (counter < retryCount) {
    status = getValueFromAxis(rbvar, &rbvalue);
    if (status) break;
    if (rbvalue == value) break;
    status = setValueOnAxis(var, value);
    counter++;
    if (status) break;
    epicsThreadSleep(.1);
  }
  return status;
}

/** Sets a floating point value on an axis
  * the values in the controller must be updated
  * \param[in] name of the variable to be updated
  * \param[in] value the (floating point) variable to be updated
  *
  */
asynStatus TwinCATmotorAxis::setValueOnAxis(const char* var, double value)
{
  sprintf(pC_->outString_, "Main.M%d.%s=%f", axisNo_, var, value);
  return writeReadACK();
}

asynStatus TwinCATmotorAxis::setValueOnAxis(unsigned adsport,
					    unsigned group_no,
					    unsigned offset_in_group,
					    int value)
{
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,2,2=%d",
	  adsport, group_no + axisNo_, offset_in_group, value);
  return writeReadACK();
}

asynStatus TwinCATmotorAxis::setValueOnAxis(unsigned adsport,
					    unsigned group_no,
					    unsigned offset_in_group,
					    double value)
{
  sprintf(pC_->outString_, "ADSPORT=%u/.ADR.16#%X,16#%X,8,5=%f",
	  adsport, group_no + axisNo_, offset_in_group, value);
  return writeReadACK();
}

/** Gets an integer or boolean value from an axis
  * \param[in] name of the variable to be retrieved
  * \param[in] pointer to the integer result
  *
  */
asynStatus TwinCATmotorAxis::getValueFromAxis(const char* var, int *value)
{
  asynStatus comStatus;
  int res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  if (var[0] == 'b') {
    if (!strcmp(pC_->inString_, "0")) {
      res = 0;
    } else if (!strcmp(pC_->inString_, "1")) {
      res = 1;
    } else {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "command=\"%s\" response=\"%s\"\n",
                pC_->outString_, pC_->inString_);
      return asynError;
    }
  } else {
    int nvals = sscanf(pC_->inString_, "%d", &res);
    if (nvals != 1) {
      asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
                "nvals=%d command=\"%s\" response=\"%s\"\n",
                nvals, pC_->outString_, pC_->inString_);
      return asynError;
    }
  }
  *value = res;
  return asynSuccess;
}


/** Gets a floating point value from an axis
  * \param[in] name of the variable to be retrieved
  * \param[in] pointer to the double result
  *
  */
asynStatus TwinCATmotorAxis::getValueFromAxis(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "Main.M%d.%s?", axisNo_, var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

asynStatus TwinCATmotorAxis::getValueFromController(const char* var, double *value)
{
  asynStatus comStatus;
  int nvals;
  double res;
  sprintf(pC_->outString_, "%s?", var);
  comStatus = pC_->writeReadOnErrorDisconnect();
  if (comStatus)
    return comStatus;
  nvals = sscanf(pC_->inString_, "%lf", &res);
  if (nvals != 1) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "nvals=%d command=\"%s\" response=\"%s\"\n",
              nvals, pC_->outString_, pC_->inString_);
    return asynError;
  }
  *value = res;
  return asynSuccess;
}

/** Set velocity and acceleration for the axis
  * \param[in] maxVelocity, mm/sec
  * \param[in] acceleration, seconds to maximum velocity
  *
  */
asynStatus TwinCATmotorAxis::sendVelocityAndAccelExecute(double maxVelocity, double acceleration)
{
  asynStatus status;
  /* We don't use minVelocity */
  status = setValueOnAxis("fVelocity", maxVelocity);
  /* We don't send acceleration yet:
     in the motorRecord acceleration is defined "as time in seconds to reach maxVelocity",
     the motion controllers use "mm/sec2" or so.
     Until we have the proper re-calculation, we use the default acceleration
     configured in the  motion controller
  */
  if (status == asynSuccess) status = setValueOnAxis("bExecute", 1);
  drvlocal.waitNumPollsBeforeReady += 2;
  return status;
}

/** Move the axis to a position, either absolute or relative
  * \param[in] position in mm
  * \param[in] relative (0=absolute, otherwise relative)
  * \param[in] minimum velocity, mm/sec
  * \param[in] maximum velocity, mm/sec
  * \param[in] acceleration, seconds to maximum velocity
  *
  */
asynStatus TwinCATmotorAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;

  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = updateSoftLimitsIfDirty(__LINE__);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", relative ? 2 : 3);
  if (status == asynSuccess) status = setValueOnAxis("fPosition", position);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}


/** Home the motor, search the home position
  * \param[in] minimum velocity, mm/sec
  * \param[in] maximum velocity, mm/sec
  * \param[in] acceleration, seconds to maximum velocity
  * \param[in] forwards (0=backwards, otherwise forwards)
  *
  */
asynStatus TwinCATmotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  asynStatus status = asynSuccess;

  /* The controller will do the home search, and change its internal
     raw value to what we specified in fPosition. Use 0 */
  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = updateSoftLimitsIfDirty(__LINE__);
  if ((drvlocal.axisFlags & AMPLIFIER_ON_FLAG_WHEN_HOMING) &&
      (status == asynSuccess)) status = enableAmplifier(1);
  if (status == asynSuccess) status = setValueOnAxis("fPosition", 0);
  if (status == asynSuccess) status = setValueOnAxis("nCommand", 10);
  if (status == asynSuccess) status = setValueOnAxis("nCmdData", 0);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}


/** jog the the motor, search the home position
  * \param[in] minimum velocity, mm/sec (not used)
  * \param[in] maximum velocity, mm/sec (positive or negative)
  * \param[in] acceleration, seconds to maximum velocity
  *
  */
asynStatus TwinCATmotorAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynStatus status = asynSuccess;

  if (status == asynSuccess) status = stopAxisInternal(__FUNCTION__, 0);
  if (status == asynSuccess) status = updateSoftLimitsIfDirty(__LINE__);
  if (status == asynSuccess) setValueOnAxis("nCommand", 1);
  if (status == asynSuccess) status = sendVelocityAndAccelExecute(maxVelocity, acceleration);

  return status;
}


/** Set the high soft-limit on an axis
  *
  */
asynStatus TwinCATmotorAxis::setMotorHighLimitOnAxis(void)
{
  asynStatus status = asynSuccess;
  int enable = drvlocal.defined.motorHighLimit;
  if (drvlocal.motorHighLimit <= drvlocal.motorLowLimit) enable = 0;
  if (enable && (status == asynSuccess)) {
    status = setValueOnAxis(501, 0x5000, 0xE, drvlocal.motorHighLimit);
  }
  if (status == asynSuccess) status = setValueOnAxis(501, 0x5000, 0xC, enable);
  return status;
}


/** Set the low soft-limit on an axis
  *
  */
asynStatus TwinCATmotorAxis::setMotorLowLimitOnAxis(void)
{
  asynStatus status = asynSuccess;
  int enable = drvlocal.defined.motorLowLimit;
  if (drvlocal.motorHighLimit <= drvlocal.motorLowLimit) enable = 0;
  if (enable && (status == asynSuccess)) {
    status = setValueOnAxis(501, 0x5000, 0xD, drvlocal.motorLowLimit);
  }
  if (status == asynSuccess) status = setValueOnAxis(501, 0x5000, 0xB, enable);
  return status;
}

/** Set the low soft-limit on an axis
  *
  */
asynStatus TwinCATmotorAxis::setMotorLimitsOnAxis(void)
{
  asynStatus status = asynError;
  asynPrint(pC_->pasynUserController_, ASYN_TRACEIO_DRIVER, "\n");
  if (setMotorHighLimitOnAxis() == asynSuccess &&
      setMotorLowLimitOnAxis() == asynSuccess) {
    status = asynSuccess;
  }
  drvlocal.dirty.motorLimits =  (status != asynSuccess);
  return status;
}


/** Update the soft limits in the controller, if needed
  *
  */
asynStatus TwinCATmotorAxis::updateSoftLimitsIfDirty(int line)
{
  asynPrint(pC_->pasynUserController_, ASYN_TRACEIO_DRIVER,
            "called from %d\n",line);
  if (drvlocal.dirty.motorLimits) return setMotorLimitsOnAxis();
  return asynSuccess;
}


/** Enable the amplifier on an axis
  *
  */
asynStatus TwinCATmotorAxis::enableAmplifier(int on)
{
  return setValueOnAxisVerify("bEnable", "bEnabled", on ? 1 : 0, 100);
}

/** Stop the axis
  *
  */
asynStatus TwinCATmotorAxis::stopAxisInternal(const char *function_name, double acceleration)
{
  asynStatus status = setValueOnAxis("bExecute", 0); /* Stop executing */
  if (status == asynSuccess) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
	      "stopAxisInternal(%d) (%s)\n",  axisNo_, function_name);
    drvlocal.dirty.mustStop = 0;
  } else {
    drvlocal.dirty.mustStop = 1;
  }
  return status;
}

/** Stop the axis, called by motor Record
  *
  */
asynStatus TwinCATmotorAxis::stop(double acceleration )
{
  return stopAxisInternal(__FUNCTION__, acceleration);
}



asynStatus TwinCATmotorAxis::pollAll(bool *moving, st_axis_status_type *pst_axis_status)
{
  asynStatus comStatus;

  int motor_axis_no = 0;
  int nvals;

  /* Read the complete Axis status */
  sprintf(pC_->outString_, "Main.M%d.stAxisStatus?", axisNo_);
  comStatus = pC_->writeReadController();
  if (comStatus) return comStatus;
  drvlocal.dirty.stAxisStatus_V00 = 0;
  nvals = sscanf(pC_->inString_,
		 "Main.M%d.stAxisStatus="
		 "%d,%d,%d,%u,%u,%lf,%lf,%lf,%lf,%d,"
		 "%d,%d,%d,%lf,%d,%d,%d,%u,%lf,%lf,%lf,%d,%d",
		 &motor_axis_no,
		 &pst_axis_status->bEnable,        /*  1 */
		 &pst_axis_status->bReset,         /*  2 */
		 &pst_axis_status->bExecute,       /*  3 */
		 &pst_axis_status->nCommand,       /*  4 */
		 &pst_axis_status->nCmdData,       /*  5 */
		 &pst_axis_status->fVelocity,      /*  6 */
		 &pst_axis_status->fPosition,      /*  7 */
		 &pst_axis_status->fAcceleration,  /*  8 */
		 &pst_axis_status->fDecceleration, /*  9 */
		 &pst_axis_status->bJogFwd,        /* 10 */
		 &pst_axis_status->bJogBwd,        /* 11 */
		 &pst_axis_status->bLimitFwd,      /* 12 */
		 &pst_axis_status->bLimitBwd,      /* 13 */
		 &pst_axis_status->fOverride,      /* 14 */
		 &pst_axis_status->bHomeSensor,    /* 15 */
		 &pst_axis_status->bEnabled,       /* 16 */
		 &pst_axis_status->bError,         /* 17 */
		 &pst_axis_status->nErrorId,       /* 18 */
		 &pst_axis_status->fActVelocity,   /* 19 */
		 &pst_axis_status->fActPosition,   /* 20 */
		 &pst_axis_status->fActDiff,       /* 21 */
		 &pst_axis_status->bHomed,         /* 22 */
		 &pst_axis_status->bBusy           /* 23 */);

  if (nvals == 24) {
    if (axisNo_ != motor_axis_no) return asynError;
    drvlocal.supported.stAxisStatus_V00 = 1;
    return asynSuccess;
  }
  asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
	    "poll(%d) line=%d nvals=%d\n",
	    axisNo_, __LINE__, nvals);
  drvlocal.supported.stAxisStatus_V00 = 0;
  return asynError;
}


/** Polls the axis.
  * This function reads the motor position, the limit status, the home status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false). */
asynStatus TwinCATmotorAxis::poll(bool *moving)
{
  asynStatus comStatus;
  int nowMoving = 0;
  st_axis_status_type st_axis_status;

  memset(&st_axis_status, 0, sizeof(st_axis_status));
  /* Stop if the previous stop had been lost */
  if (drvlocal.dirty.mustStop) {
    comStatus = stopAxisInternal(__FUNCTION__, 0);
    if (comStatus) goto skip;
  }
  /* Check if we are reconnected */
  if (drvlocal.oldMotorStatusProblem) handleStatusChange(asynSuccess);

  if (drvlocal.supported.stAxisStatus_V00 || drvlocal.dirty.stAxisStatus_V00) {
    comStatus = pollAll(moving, &st_axis_status);
  } else {
    comStatus = asynError;
  }
  if (comStatus) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "out=%s in=%s return=%s (%d)\n",
              pC_->outString_, pC_->inString_,
              pasynManager->strStatus(comStatus), (int)comStatus);
    goto skip;
  }
  setIntegerParam(pC_->motorStatusHomed_, st_axis_status.bHomed);
  setIntegerParam(pC_->motorStatusProblem_, st_axis_status.bError);
  setIntegerParam(pC_->motorStatusAtHome_, st_axis_status.bHomeSensor);
  setIntegerParam(pC_->motorStatusLowLimit_, !st_axis_status.bLimitBwd);
  setIntegerParam(pC_->motorStatusHighLimit_, !st_axis_status.bLimitFwd);
  setIntegerParam(pC_->motorStatusPowerOn_, st_axis_status.bEnabled);

  /* Use previous fActPosition and current fActPosition to calculate direction.*/
  if (st_axis_status.fActPosition > drvlocal.oldPosition) {
    setIntegerParam(pC_->motorStatusDirection_, 1);
  } else if (st_axis_status.fActPosition < drvlocal.oldPosition) {
    setIntegerParam(pC_->motorStatusDirection_, 0);
  }
  drvlocal.oldPosition = st_axis_status.fActPosition;
  setDoubleParam(pC_->motorPosition_, st_axis_status.fActPosition);
  if (drvlocal.externalEncoderStr) {
    double fEncPosition;
    comStatus = getValueFromController(drvlocal.externalEncoderStr, &fEncPosition);
    if (!comStatus) setDoubleParam(pC_->motorEncoderPosition_, fEncPosition);
  }

  nowMoving = st_axis_status.bBusy && st_axis_status.bExecute && st_axis_status.bEnabled;
  if (drvlocal.waitNumPollsBeforeReady) {
    drvlocal.waitNumPollsBeforeReady--;
    *moving = true;
  } else {
    setIntegerParam(pC_->motorStatusMoving_, nowMoving);
    setIntegerParam(pC_->motorStatusDone_, !nowMoving);
    *moving = nowMoving ? true : false;
  }
  if (drvlocal.oldNowMoving != nowMoving) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "poll(%d) nowMoving=%d bBusy=%d bExecute=%d fActPosition=%f\n",
              axisNo_, nowMoving,
	      st_axis_status.bBusy, st_axis_status.bExecute, st_axis_status.fActPosition);
    drvlocal.oldNowMoving = nowMoving;
  }

  if (drvlocal.dirty.reportDisconnect) goto skip;

  if (drvlocal.oldMotorStatusProblem) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "reconnected\n");
    drvlocal.oldMotorStatusProblem = 0;
  }
  //setIntegerParam(pC_->motorStatusProblem_, 0);
  callParamCallbacks();
  return asynSuccess;

skip:
  memset(&drvlocal.dirty, 0xFF, sizeof(drvlocal.dirty));
  if (!drvlocal.oldMotorStatusProblem) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_ERROR|ASYN_TRACEIO_DRIVER,
              "Communication error\n");
  }
  drvlocal.dirty.reportDisconnect = 0;
  drvlocal.oldMotorStatusProblem = 1;
  setIntegerParam(pC_->motorStatusProblem_, 1);
  callParamCallbacks();
  return asynError;
}

asynStatus TwinCATmotorAxis::setIntegerParam(int function, int value)
{
  asynStatus status;
  if (function == pC_->motorClosedLoop_) {
    if (drvlocal.axisFlags & AMPLIFIER_ON_FLAG_USING_CNEN) {
      (void)enableAmplifier(value);
    }
  }

  //Call base class method
  status = asynMotorAxis::setIntegerParam(function, value);
  return status;
}

/** Set a floating point parameter on the axis
  * \param[in] function, which parameter is updated
  * \param[in] value, the new value
  *
  * When the IOC starts, we will send the soft limits to the controller.
  * When a soft limit is changed, and update is send them to the controller.
  */
asynStatus TwinCATmotorAxis::setDoubleParam(int function, double value)
{
  asynStatus status;
  if (function == pC_->motorHighLimit_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(motorHighLimit_)=%f\n", value);
    drvlocal.motorHighLimit = value;
    drvlocal.defined.motorHighLimit = 1;
    drvlocal.dirty.motorLimits = 1;
    if (drvlocal.defined.motorLowLimit) setMotorLimitsOnAxis();
  } else if (function == pC_->motorLowLimit_) {
    asynPrint(pC_->pasynUserController_, ASYN_TRACE_INFO,
              "setDoubleParam(motorLowLimit_)=%f\n", value);
    drvlocal.motorLowLimit = value;
    drvlocal.defined.motorLowLimit = 1;
    drvlocal.dirty.motorLimits = 1;
    if (drvlocal.defined.motorHighLimit) setMotorLimitsOnAxis();
  }

  // Call the base class method
  status = asynMotorAxis::setDoubleParam(function, value);
  return status;
}
