#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "hw_motor.h"
#include "sock-util.h" /* stdlog */

#define MOTOR_POS_HOME 0
#define MOTOR_REV_ERES (-57)
#define MOTOR_PARK_POS (-64)

/* Homing procdures LS=Limit switch, HS=Home switch */
#define HOME_PROC_LOW_LS  1
#define HOME_PROC_HIGH_LS 2
#define HOME_PROC_LOW_HS  3
#define HOME_PROC_HIGH_HS 4

#define MOTOR_VEL_HOME_MAX 5.0

typedef struct
{
  struct timeval lastPollTime;

  double amplifierPercent;
  /* What the (simulated) hardware has physically.
     When homing against the high limit switch is done,
     all logical values will be re-calculated.
  */
  double HWlowPos;
  double HWhighPos;
  double HWhomeSwitchpos;
  /*
     What the (simulated) hardware has logically.
  */
  double HomeSwitchPos;     /* home switch */
  double HomeProcPos;       /* Position of used home switch */
  double highHardLimitPos;
  double lowHardLimitPos;

  /* What EPICS sends us */
  double highSoftLimitPos;
  double lowSoftLimitPos;
  int definedLowHardLimitPos;
  int definedHighHardLimitPos;
  int enabledLowSoftLimitPos;
  int enabledHighSoftLimitPos;
  int hitPosLimitSwitch;
  int hitNegLimitSwitch;
  double MotorPosNow;
  double MotorPosWanted;
  double HomeVelocityAbsWanted;
  double MaxHomeVelocityAbs;
  struct {
    double HomeVelocity;
    double PosVelocity;
    double JogVelocity;
  } velo;
  double EncoderPos;
  double ParkingPos;
  double ReverseERES;
  int homed;
  int bError;
  int nErrorId;
  char pLimits;
} motor_axis_type;


static motor_axis_type motor_axis[MAX_AXES];
static motor_axis_type motor_axis_last[MAX_AXES];
static motor_axis_type motor_axis_reported[MAX_AXES];

static void recalculate_pos(int axis_no, int nCmdData)
{
  double HWlowPos = motor_axis[axis_no].HWlowPos;
  double HWhomeSwitchpos = motor_axis[axis_no].HWhomeSwitchpos;
  double HWhighPos = motor_axis[axis_no].HWhighPos;
  double oldLowHardLimitPos = motor_axis[axis_no].lowHardLimitPos;
  switch (nCmdData) {
    case HOME_PROC_LOW_LS:
      motor_axis[axis_no].lowHardLimitPos = 0;
      motor_axis[axis_no].HomeSwitchPos = HWhomeSwitchpos - HWlowPos;
      motor_axis[axis_no].highHardLimitPos = HWhighPos - HWlowPos;
      break;
    case HOME_PROC_HIGH_LS:
      motor_axis[axis_no].lowHardLimitPos = HWlowPos - HWhighPos;
      motor_axis[axis_no].HomeSwitchPos = HWhomeSwitchpos - HWhighPos;
      motor_axis[axis_no].highHardLimitPos = 0;
      break;
    case HOME_PROC_LOW_HS:
    case HOME_PROC_HIGH_HS:
      motor_axis[axis_no].lowHardLimitPos = HWlowPos;
      motor_axis[axis_no].HomeSwitchPos = 0;
      motor_axis[axis_no].highHardLimitPos = HWhighPos;
      break;
  }
  motor_axis[axis_no].HomeProcPos = 0; /* in any case */
  motor_axis[axis_no].MotorPosWanted = 0;
  /* adjust position to "force a simulated movement" */
  motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].lowHardLimitPos - oldLowHardLimitPos;

  fprintf(stdlog,
          "%s/%s:%d axis_no=%d MotorPosNow=%g lowHardLimitPos=%f HomeSwitchPos=%f higHardLimitPos=%f\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          motor_axis[axis_no].MotorPosNow,
          motor_axis[axis_no].lowHardLimitPos,
          motor_axis[axis_no].HomeSwitchPos,
          motor_axis[axis_no].highHardLimitPos);
}

static double getEncoderPosFromMotorPos(int axis_no, double MotorPosNow)
{
  (void)axis_no;
  return ((MotorPosNow - motor_axis[axis_no].ParkingPos)) * motor_axis[axis_no].ReverseERES;
}

#if 0
static double getMotorPosFromEncoderPos(int axis_no, double EncoderPos)
{
  (void)axis_no;
  return (double)(int)((EncoderPos / motor_axis[axis_no].ReverseERES) + motor_axis[axis_no].ParkingPos);
}
#endif

void hw_motor_init(int axis_no)
{
  static char init_done[MAX_AXES];
  if (axis_no >= MAX_AXES || axis_no < 0) {
    return;
  }
  if (!init_done[axis_no]) {
    memset(&motor_axis[axis_no], 0, sizeof(motor_axis[axis_no]));
    memset(&motor_axis_last[axis_no], 0, sizeof(motor_axis_last[axis_no]));
    memset(&motor_axis_reported[axis_no], 0, sizeof(motor_axis_reported[axis_no]));
    motor_axis[axis_no].HomeSwitchPos = MOTOR_POS_HOME;
    motor_axis[axis_no].amplifierPercent = 100;
    motor_axis[axis_no].MaxHomeVelocityAbs = MOTOR_VEL_HOME_MAX;
    setMotorParkingPosition(axis_no, MOTOR_PARK_POS);
    motor_axis[axis_no].ReverseERES = MOTOR_REV_ERES;
    motor_axis[axis_no].EncoderPos = getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
    motor_axis_last[axis_no].EncoderPos  = motor_axis[axis_no].EncoderPos;
    motor_axis_last[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosNow;
    init_done[axis_no] = 1;
  }
}

static void init_axis(int axis_no)
{
  (void)axis_no;
  hw_motor_init(axis_no);
}

void setMotorParkingPosition(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].ParkingPos = value;
  motor_axis[axis_no].MotorPosNow = value;
  motor_axis[axis_no].EncoderPos =
    getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
}

void setMotorReverseERES(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].ReverseERES = value;
}


void setHomePos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].HomeSwitchPos = value;
}

void setMaxHomeVelocityAbs(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  if (((axis_no) <= 0) || ((axis_no) >=MAX_AXES)) {
    return;
  }
  motor_axis[axis_no].MaxHomeVelocityAbs = value;
}

static double getMotorVelocityInt(int axis_no)
{
  if (motor_axis[axis_no].velo.JogVelocity) return motor_axis[axis_no].velo.JogVelocity;
  if (motor_axis[axis_no].velo.PosVelocity) return motor_axis[axis_no].velo.PosVelocity;
  if (motor_axis[axis_no].velo.HomeVelocity) return motor_axis[axis_no].velo.HomeVelocity;
  return 0;
}

double getMotorVelocity(int axis_no)
{
  double velocity;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  velocity = getMotorVelocityInt(axis_no);
  return velocity;
}

int isMotorMoving(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return getMotorVelocityInt(axis_no) ? 1 : 0;
}

int getAxisDone(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = !isMotorMoving(axis_no);
  return ret;
}

int getAxisHome(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomeProcPos);
  return ret;
}

int getAxisHomed(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = motor_axis[axis_no].homed;
  return ret;
}


double getLowSoftLimitPos(int axis_no)
{
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].lowSoftLimitPos;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  return value;
}

void setLowSoftLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowSoftLimitPos = value;
}

void enableLowSoftLimit(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledLowSoftLimitPos = value;
}

void setLowHardLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].lowHardLimitPos = value;
  motor_axis[axis_no].definedLowHardLimitPos = 1;
}

double getHighSoftLimitPos(int axis_no)
{
  double value = 0;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].highSoftLimitPos;
  return value;
}

void setHighSoftLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highSoftLimitPos = value;
}

void enableHighSoftLimit(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].enabledHighSoftLimitPos = value;
}

void setHighHardLimitPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].highHardLimitPos = value;
  motor_axis[axis_no].definedHighHardLimitPos = 1;
}

static int soft_limits_clip(int axis_no, double velocity)
{
  int clipped = 0;
  /* Soft limits defined: Clip the value  */
  if (motor_axis[axis_no].enabledHighSoftLimitPos &&
      velocity > 0 &&
      motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].highSoftLimitPos) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d CLIP soft low MotorPosNow=%f highSoftLimitPos=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].highSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].highSoftLimitPos;
    clipped = 1;
  }
  if (motor_axis[axis_no].enabledLowSoftLimitPos &&
      velocity < 0 &&
      motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].lowSoftLimitPos) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d CLIP soft high MotorPosNow=%f lowSoftLimitPos=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].lowSoftLimitPos);
    motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].lowSoftLimitPos;
    clipped = 1;
  }
  return clipped;
} /* Soft limits */

void setHWlowPos (int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWlowPos = value;
}

void setHWhighPos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhighPos = value;
}

void setHWhomeSwitchpos(int axis_no, double value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].HWhomeSwitchpos = value;
}


static void simulateMotion(int axis_no)
{
  struct timeval timeNow;
  double velocity = getMotorVelocity(axis_no);
  int clipped = 0;

  AXIS_CHECK_RETURN(axis_no);

  if (motor_axis[axis_no].amplifierPercent < 100) {
    if (velocity) {
      /* Amplifier off, while moving */
      set_nErrorId(axis_no, 16992);
      set_bError(axis_no, 1);
      StopInternal(axis_no);
    }
  }

  gettimeofday(&timeNow, NULL);

  if (motor_axis[axis_no].velo.JogVelocity &&
      !soft_limits_clip(axis_no, velocity)) {
    /* Simulate jogging  */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.JogVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
  }

  if (motor_axis[axis_no].velo.PosVelocity &&
      !soft_limits_clip(axis_no, velocity)) {
    /* Simulate a move to postion */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.PosVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);

    if (((motor_axis[axis_no].velo.PosVelocity > 0) &&
         (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].MotorPosWanted)) ||
        ((motor_axis[axis_no].velo.PosVelocity < 0) &&
         (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].MotorPosWanted))) {
      /* overshoot or undershoot. We are at the target position */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosWanted;
      motor_axis[axis_no].velo.PosVelocity = 0;
    }
  }

  if (motor_axis[axis_no].velo.HomeVelocity) {
    /* Simulate move to home */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.HomeVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);

    if (((motor_axis[axis_no].velo.HomeVelocity > 0) &&
         (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].HomeProcPos)) ||
        ((motor_axis[axis_no].velo.HomeVelocity < 0) &&
         (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].HomeProcPos))) {
      /* overshoot or undershoot. We are at home */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].HomeProcPos;
    }
  }
  if (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomeProcPos) {
    motor_axis[axis_no].velo.HomeVelocity = 0;
    motor_axis[axis_no].homed = 1;
  }

  motor_axis[axis_no].lastPollTime = timeNow;
  clipped = soft_limits_clip(axis_no, velocity);
  if (motor_axis[axis_no].highHardLimitPos > motor_axis[axis_no].lowHardLimitPos) {
    /* Hard limits defined: Clip the value  */
    if (motor_axis[axis_no].definedHighHardLimitPos &&
        velocity > 0 &&
        motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].highHardLimitPos) {
      fprintf(stdlog,
              "%s/%s:%d axis_no=%d CLIP HLS MotorPosNow=%f highHardLimitPos=%f\n",
              __FILE__, __FUNCTION__, __LINE__,
              axis_no,
              motor_axis[axis_no].MotorPosNow,
              motor_axis[axis_no].highHardLimitPos);
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].highHardLimitPos;
      clipped = 1;
    }
    if (motor_axis[axis_no].definedLowHardLimitPos &&
        velocity < 0 &&
        motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].lowHardLimitPos) {
      fprintf(stdlog,
              "%s/%s:%d axis_no=%d CLIP LLS MotorPosNow=%f lowHardLimitPos=%f\n",
              __FILE__, __FUNCTION__, __LINE__,
              axis_no,
              motor_axis[axis_no].MotorPosNow,
              motor_axis[axis_no].lowHardLimitPos);
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].lowHardLimitPos;
      clipped = 1;
    }
  } /* Hard limits */

  if (memcmp(&motor_axis_last[axis_no].velo, &motor_axis[axis_no].velo, sizeof(motor_axis[axis_no].velo)) ||
      motor_axis_last[axis_no].MotorPosNow     != motor_axis[axis_no].MotorPosNow ||
      motor_axis_last[axis_no].MotorPosWanted  != motor_axis[axis_no].MotorPosWanted ||
      clipped) {
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d velocity=%g MotorPosWanted=%f JogVelocity=%g PosVelocity=%g HomeVelocity=%g home=%d MotorPosNow=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            velocity,
            motor_axis[axis_no].MotorPosWanted,
            motor_axis[axis_no].velo.JogVelocity,
            motor_axis[axis_no].velo.PosVelocity,
            motor_axis[axis_no].velo.HomeVelocity,
            getAxisHome(axis_no),
              motor_axis[axis_no].MotorPosNow);
    memcpy(&motor_axis_last[axis_no], &motor_axis[axis_no], sizeof(motor_axis[axis_no]));
  }
  if (clipped) {
    StopInternal(axis_no);
  }
}

double getMotorPos(int axis_no)
{
  simulateMotion(axis_no);
  /* This simulation has EncoderPos */
  motor_axis[axis_no].EncoderPos = getEncoderPosFromMotorPos(axis_no, motor_axis[axis_no].MotorPosNow);
  return motor_axis[axis_no].MotorPosNow;
}

double getEncoderPos(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  (void)getMotorPos(axis_no);
  if (motor_axis_reported[axis_no].EncoderPos != motor_axis[axis_no].EncoderPos) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d EncoderPos=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].EncoderPos);
    motor_axis_reported[axis_no].EncoderPos = motor_axis[axis_no].EncoderPos;
  }
  return motor_axis[axis_no].EncoderPos;
}

/* Stop the ongoing motion (like JOG),
   to be able to start a new one (like HOME)
*/
void StopInternal_fl(int axis_no, const char *file, int line_no)
{
  fprintf(stdlog, "%s/%s:%d axis_no=%d file=%s line_no=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, file, line_no);
  AXIS_CHECK_RETURN(axis_no);
  memset(&motor_axis[axis_no].velo, 0,
         sizeof(motor_axis[axis_no].velo));
}


/* caput pv.VAL */
int movePosition(int axis_no,
                 double position,
                 int relative,
                 double max_velocity,
                 double acceleration)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  fprintf(stdlog, "%s/%s:%d axis_no=%d relative=%d position=%f max_velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          relative,
          position,
          max_velocity,
          acceleration);
  StopInternal(axis_no);
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (relative) {
    position += motor_axis[axis_no].MotorPosNow;
  }
  motor_axis[axis_no].MotorPosWanted = position;

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].velo.PosVelocity = max_velocity;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].velo.PosVelocity = -max_velocity;
  } else {
    motor_axis[axis_no].velo.PosVelocity = 0;
  }

  return 0;
}


int moveHomeProc(int axis_no,
                 int direction,
                 int nCmdData,
                 double max_velocity,
                 double acceleration)
{
  double position;
  double velocity = max_velocity ? max_velocity : motor_axis[axis_no].MaxHomeVelocityAbs;
  velocity = fabs(velocity);
  fprintf(stdlog, "%s/%s:%d axis_no=%d nCmdData=%d max_velocity=%g velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          nCmdData,
          max_velocity,
          velocity,
          acceleration);
  recalculate_pos(axis_no, nCmdData);
  position = motor_axis[axis_no].HomeProcPos;
  switch (nCmdData) {
    case HOME_PROC_LOW_LS:
      if (!motor_axis[axis_no].definedLowHardLimitPos)
        return -1;
      motor_axis[axis_no].HomeProcPos = motor_axis[axis_no].lowHardLimitPos;
      break;
    case HOME_PROC_HIGH_LS:
      if (!motor_axis[axis_no].definedHighHardLimitPos)
        return -1;
      motor_axis[axis_no].HomeProcPos =
        motor_axis[axis_no].highHardLimitPos;
      break;
    case HOME_PROC_LOW_HS:
    case HOME_PROC_HIGH_HS:
      motor_axis[axis_no].HomeProcPos = motor_axis[axis_no].HomeSwitchPos;
      break;
    default:
      return -1;
  }
  position = motor_axis[axis_no].HomeProcPos;

  if (motor_axis[axis_no].MaxHomeVelocityAbs &&
      (fabs(velocity) > motor_axis[axis_no].MaxHomeVelocityAbs)) {
    velocity = motor_axis[axis_no].MaxHomeVelocityAbs;
  }
  motor_axis[axis_no].HomeVelocityAbsWanted = velocity;
  fprintf(stdlog, "%s/%s:%d axis_no=%d direction=%d max_velocity=%g velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          direction,
          max_velocity,
          velocity,
          acceleration);
  StopInternal(axis_no);
  motor_axis[axis_no].homed = 0; /* Not homed any more */
  gettimeofday(&motor_axis[axis_no].lastPollTime, NULL);

  if (position > motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].velo.HomeVelocity = velocity;
  } else if (position < motor_axis[axis_no].MotorPosNow) {
    motor_axis[axis_no].velo.HomeVelocity = -velocity;
  } else {
    motor_axis[axis_no].velo.HomeVelocity = 0;
  }

  return 0;
};

 /* caput pv.HOMF, caput pv.HOMR */
int moveHome(int axis_no,
             int direction,
             double max_velocity,
             double acceleration)
{
  return moveHomeProc(axis_no, direction,
                      HOME_PROC_LOW_HS, /* int nCmdData, */
                      max_velocity,acceleration);
}


/* caput pv.JOGF, caput pv.JOGR */
int moveVelocity(int axis_no,
                 int direction,
                 double max_velocity,
                 double acceleration)
{
  double velocity = max_velocity;
  if (!direction) {
    velocity = - velocity;
  }

  StopInternal(axis_no);

  fprintf(stdlog, "%s/%s:%d axis_no=%d direction=%d max_velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          direction,
          max_velocity,
          acceleration);
  if (direction < 0) {
    velocity = -velocity;
  }
  motor_axis[axis_no].velo.JogVelocity = velocity;
  return 0;
};



int setAmplifierPercent(int axis_no, int percent)
{
  fprintf(stdlog, "%s/%s:%d axis_no=%d percent=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no, percent);
  AXIS_CHECK_RETURN_ERROR(axis_no);
  if (percent < 0 || percent > 100) return -1;
  motor_axis[axis_no].amplifierPercent = percent;
  return 0;
}

int getAmplifierOn(int axis_no)
{
  if (motor_axis[axis_no].amplifierPercent == 100) return 1;
  return 0;
}


int getNegLimitSwitch(int axis_no)
{
  motor_axis[axis_no].hitNegLimitSwitch =
    motor_axis[axis_no].definedLowHardLimitPos &&
    (motor_axis[axis_no].MotorPosNow <= motor_axis[axis_no].lowHardLimitPos);

  if (motor_axis_reported[axis_no].hitNegLimitSwitch != motor_axis[axis_no].hitNegLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedLowHardLimitPos=%d MotorPosNow=%f lowHardLimitPos=%f hitNegLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedLowHardLimitPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].lowHardLimitPos,
            motor_axis[axis_no].hitNegLimitSwitch);
    motor_axis_reported[axis_no].hitNegLimitSwitch = motor_axis[axis_no].hitNegLimitSwitch;
  }
  return motor_axis[axis_no].hitNegLimitSwitch;
}

int getPosLimitSwitch(int axis_no)
{
  motor_axis[axis_no].hitPosLimitSwitch =
    motor_axis[axis_no].definedHighHardLimitPos &&
    (motor_axis[axis_no].MotorPosNow >= motor_axis[axis_no].highHardLimitPos);

  if (motor_axis_reported[axis_no].hitPosLimitSwitch != motor_axis[axis_no].hitPosLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedHighHardLimitPos=%d MotorPosNow=%f highHardLimitPos=%f hitPosLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedHighHardLimitPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].highHardLimitPos,
            motor_axis[axis_no].hitPosLimitSwitch);
    motor_axis_reported[axis_no].hitPosLimitSwitch = motor_axis[axis_no].hitPosLimitSwitch;
  }
  return motor_axis[axis_no].hitPosLimitSwitch;
}

int get_bError(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].bError;
}

int set_bError(int axis_no, int value)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].bError = value;
  return 0;
}

int get_nErrorId(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].nErrorId;
}

int set_nErrorId(int axis_no, int value)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  motor_axis[axis_no].nErrorId = value;
  return 0;
}

