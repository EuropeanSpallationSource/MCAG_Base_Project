/*
FILENAME...   eemcu.h
*/

#include "asynAxisController.h"
#include "asynAxis.h"

#define AMPLIFIER_ON_FLAG_CREATE_AXIS  (1)
#define AMPLIFIER_ON_FLAG_WHEN_HOMING  (1<<1)
#define AMPLIFIER_ON_FLAG_USING_CNEN   (1<<2)

#ifndef motorRecResolutionString
#define CREATE_MOTOR_REC_RESOLUTION
#define motorRecDirectionString         "MOTOR_REC_DIRECTION"
#define motorRecOffsetString            "MOTOR_REC_OFFSET"
#define motorRecResolutionString        "MOTOR_REC_RESOLUTION"
#endif

#define eemcuErrString                  "Err"
#define eemcuErrIdString                "ErrId"
#define eemcuProcHomString              "ProcHom"
#define eemcuErrRstString               "ErrRst"
#define eemcuJVELString                 "JVEL_"
#define eemcuErrMsgString               "ErrMsg"

extern "C" {
  int eemcuCreateAxis(const char *eemcuName, int axisNo,
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

class epicsShareClass eemcuAxis : public asynAxis
{
public:
  /* These are the methods we override from the base class */
  eemcuAxis(class eemcuController *pC, int axisNo,
            int axisFlags, const char *axisOptionsStr);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
  asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
  asynStatus home(double min_velocity, double max_velocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  void       callParamCallbacksUpdateError();
  asynStatus pollAll(bool *moving);
  asynStatus pollAll(bool *moving, st_axis_status_type *pst_axis_status);
  asynStatus poll(bool *moving);

private:
  typedef enum
  {
    eeAxisErrorNoError,
    eeAxisErrorMCUError,
    eeAxisErrorIOCcomError,
    eeAxisErrorIOCcfgError,
    eeAxisErrorIOCError
  } eeAxisErrorType;
  eemcuController *pC_;          /**< Pointer to the asynAxisController to which this axis belongs.
                                   *   Abbreviated because it is used very frequently */
  struct {
    double mres;
    double motorHighLimit;
    double motorLowLimit;
    double oldPosition;
    const char *externalEncoderStr;
    const char *cfgfileStr;
    int axisFlags;
    int oldNowMoving;
    int MCU_nErrorId;     /* nErrorID from MCU */
    int old_MCU_nErrorId; /* old nErrorID from MCU */
    int old_EPICS_nErrorId; /* old nErrorID from MCU */

    int old_bError;   /* copy of bError */
    unsigned int waitNumPollsBeforeReady;
    int mustStop;
    int nCommand;
    eeAxisErrorType old_eeAxisError;
    eeAxisErrorType eeAxisError;
    /* Which values have changed in the EPICS IOC, but are not updated in the
       motion controller */
    struct {
      int          nMotionAxisID;     /* Needed for ADR commands */
      unsigned int motorLimits      :1;
      unsigned int stAxisStatus_V00 :1;
      unsigned int oldStatusDisconnected : 1;
      unsigned int initialUpdate    :1;
      unsigned int sErrorMessage    :1; /* From MCU */
      unsigned int readConfigFile   :1;
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

  asynStatus handleDisconnect(void);
  asynStatus handleConnect(void);
  asynStatus readConfigFile(void);
  asynStatus initialUpdate(void);

  asynStatus handleStatusChange(asynStatus status);

  asynStatus writeReadACK(void);
  asynStatus setValueOnAxis(const char* var, int value);
  asynStatus setValueOnAxisVerify(const char *var, const char *rbvar,
                                  int value, unsigned int retryCount);
  asynStatus setValueOnAxis(const char* var, double value);
  int getMotionAxisID(void);
  asynStatus setADRValueOnAxis(unsigned adsport,
                               unsigned indexGroup,
                               unsigned indexOffset,
                               int value);

  asynStatus setADRValueOnAxisVerify(unsigned adsport,
                                     unsigned indexGroup,
                                     unsigned indexOffset,
                                     int value,
                                     unsigned int retryCount);

  asynStatus setADRValueOnAxis(unsigned adsport,
                               unsigned indexGroup,
                               unsigned indexOffset,
                               double value);

  asynStatus setADRValueOnAxisVerify(unsigned adsport,
                                     unsigned indexGroup,
                                     unsigned indexOffset,
                                     double value,
                                     unsigned int retryCount);

  asynStatus getADRValueFromAxis(unsigned adsport,
                                 unsigned indexGroup,
                                 unsigned indexOffset,
                                 int *value);

  asynStatus getADRValueFromAxis(unsigned adsport,
                                 unsigned indexGroup,
                                 unsigned indexOffset,
                                 double *value);

  asynStatus getValueFromAxis(const char* var, int *value);
  asynStatus getValueFromAxis(const char* var, double *value);
  asynStatus getStringFromAxis(const char* var, char *value, size_t maxlen);
  asynStatus getValueFromController(const char* var, double *value);

  asynStatus setMotorLimitsOnAxisIfDefined(void);
  asynStatus updateMresSoftLimitsIfDirty(int);
  asynStatus resetAxis(void);
  asynStatus enableAmplifier(int);
  asynStatus sendVelocityAndAccelExecute(double maxVelocity, double acceleration);
  asynStatus setClosedLoop(bool closedLoop);
  asynStatus setIntegerParam(int function, int value);
  asynStatus setDoubleParam(int function, double value);
  asynStatus setStringParam(int function, const char *value);
  asynStatus stopAxisInternal(const char *function_name, double acceleration);

  friend class eemcuController;
};

class epicsShareClass eemcuController : public asynAxisController {
public:
  eemcuController(const char *portName, const char *eemcuPortName, int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  asynStatus writeReadOnErrorDisconnect(void);
  eemcuAxis* getAxis(asynUser *pasynUser);
  eemcuAxis* getAxis(int axisNo);
  protected:
  void handleStatusChange(asynStatus status);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  /* First parameter */
  int eemcuErr_;
  int eemcuProcHom_;

#ifdef CREATE_MOTOR_REC_RESOLUTION
  int motorRecResolution_;
  int motorRecDirection_;
  int motorRecOffset_;
#endif

  /* Add parameters here */
  int eemcuErrMsg_;
  int eemcuErrRst_;
  int eemcuJVEL_;
  int eemcuErrId_;
  /* Last parameter */

  #define FIRST_VIRTUAL_PARAM eemcuErr_
  #define LAST_VIRTUAL_PARAM eemcuErrId_
  #define NUM_VIRTUAL_MOTOR_PARAMS ((int) (&LAST_VIRTUAL_PARAM - &FIRST_VIRTUAL_PARAM + 1))

  friend class eemcuAxis;
};
