/*
FILENAME...   TwinCATmotor.h
*/

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif

#define BERRORString                    "BERROR"
#define NERRORIDString                  "NERRORID"
#define HOME_PROCString                 "HOME_PROC"

extern "C" {
  int TwinCATmotorCreateAxis(const char *TwinCATmotorName, int axisNo,
			     int axisFlags, const char *axisOptionsStr);
}

typedef struct {
  int bEnable;           /*  1 */
  int bReset;            /*  2 */
  int bExecute;          /*  3 */
  int nCommand;          /*  4 */
  int nCmdData;          /*  5 */
  double fVelocity;      /*  6 */
  double fPosition;      /*  7 */
  double fAcceleration;  /*  8 */
  double fDecceleration; /*  9 */
  int bJogFwd;           /* 10 */
  int bJogBwd;           /* 11 */
  int bLimitFwd;         /* 12 */
  int bLimitBwd;         /* 13 */
  double fOverride;      /* 14 */
  int bHomeSensor;       /* 15 */
  int bEnabled;          /* 16 */
  int bError;            /* 17 */
  int nErrorId;          /* 18 */
  double fActVelocity;   /* 19 */
  double fActPosition;   /* 20 */
  double fActDiff;       /* 21 */
  int bHomed;            /* 22 */
  int bBusy;             /* 23 */
} st_axis_status_type;

class epicsShareClass TwinCATmotorAxis : public asynMotorAxis
{
public:
  /* These are the methods we override from the base class */
  TwinCATmotorAxis(class TwinCATmotorController *pC, int axisNo,
		   int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus pollAll(bool *moving);
  asynStatus pollAll(bool *moving, st_axis_status_type *pst_axis_status);
  asynStatus poll(bool *moving);

private:
  TwinCATmotorController *pC_;          /**< Pointer to the asynMotorController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    double mres;
    double motorHighLimit;
    double motorLowLimit;
    double oldPosition;
    const char *externalEncoderStr;
    int axisFlags;
    int oldMotorStatusProblem;
    int oldNowMoving;
    int old_bError;
    int old_nErrorId;
    unsigned int waitNumPollsBeforeReady;
    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      int          nMotionAxisID;     /* Needed for ADR commands */
      unsigned int motorLimits      :1;
      unsigned int mustStop         :1;
      unsigned int reportDisconnect :1;
      unsigned int stAxisStatus_V00 :1;
    }  dirty;
    /* Which values have been defined: at startup none */
    struct {
      unsigned int motorHighLimit   :1;
      unsigned int motorLowLimit    :1;
    }  defined;
    struct {
      unsigned int stAxisStatus_V00 :1;
    }  supported;
  } drvlocal;

  void handleStatusChange(asynStatus status);

  asynStatus writeReadACK(void);
  asynStatus setValueOnAxis(const char* var, int value);
  asynStatus setValueOnAxisVerify(const char *var, const char *rbvar, 
                                  int value, unsigned int retryCount);
  asynStatus setValueOnAxis(const char* var, double value);
  int getMotionAxisID(void);
  asynStatus setValueOnAxis(unsigned adsport,
			    unsigned group_no, 
			    unsigned offset_in_group,
			    int value);

  asynStatus setValueOnAxis(unsigned adsport,
			    unsigned group_no, 
			    unsigned offset_in_group,
			    double value);

  asynStatus getValueFromAxis(const char* var, int *value);
  asynStatus getValueFromAxis(const char* var, double *value);
  asynStatus getValueFromController(const char* var, double *value);

  asynStatus setMotorHighLimitOnAxis(void);
  asynStatus setMotorLowLimitOnAxis(void);

  asynStatus setMotorLimitsOnAxisIfDefined(void);
  asynStatus updateSoftLimitsIfDirty(int);
  asynStatus resetAxis(void);
  asynStatus enableAmplifier(int);
  asynStatus sendVelocityAndAccelExecute(double maxVelocity, double acceleration);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);

  friend class TwinCATmotorController;
};

class epicsShareClass TwinCATmotorController : public asynMotorController {
public:
  TwinCATmotorController(const char *portName, const char *TwinCATmotorPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  asynStatus writeReadOnErrorDisconnect(void);
  TwinCATmotorAxis* getAxis(asynUser *pasynUser);
  TwinCATmotorAxis* getAxis(int axisNo);
  protected:
  void handleStatusChange(asynStatus status);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  /* First parameter */
  int TwinCATmotorBError_;
  int TwinCATmotorHomeProc_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif

  /* Add parameters here */

  int TwinCATmotorNErrorId_;
  /* Last parameter */

  #define FIRST_VIRTUAL_PARAM TwinCATmotorBError_
  #define LAST_VIRTUAL_PARAM TwinCATmotorNErrorId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int) (&LAST_VIRTUAL_PARAM - &FIRST_VIRTUAL_PARAM + 1))

  friend class TwinCATmotorAxis;
};
