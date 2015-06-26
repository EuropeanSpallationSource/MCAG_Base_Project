#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include "hw_motor.h"
#include "sock-util.h" /* stdlog */

#define MOTOR_POS_HOME 30.0
#define MOTOR_VEL_HOME 1.0
#define MOTOR_VEL_HOME_MAX 9.9

typedef struct
{
  struct timeval lastPollTime;

  double HomePos;     /* home switch */
  double valueMaxPos; /* limit switch */
  double valueMinPos; /* limit switch */
  int definedMinPos;
  int definedMaxPos;
  int hitPosLimitSwitch;
  int hitNegLimitSwitch;
  double MotorPosNow;
  double MotorPosWanted;
  double HomeVelocityAbsWanted;
  struct {
    double HomeVelocity;
    double PosVelocity;
    double JogVelocity;
  } velo;
  double EncoderPos;
  int homed;
  char pLimits;
} motor_axis_type;


static motor_axis_type motor_axis[MAX_AXES];
static motor_axis_type motor_axis_last[MAX_AXES];
static motor_axis_type motor_axis_reported[MAX_AXES];

static void init(void)
{
  static int init_done;
  if (!init_done) {
    unsigned int u;
    memset(motor_axis, 0, sizeof(motor_axis));
    memset(motor_axis_last, 0, sizeof(motor_axis_last));
    memset(motor_axis_reported, 0, sizeof(motor_axis_reported));
    for (u=0; u < MAX_AXES; u++) {
      motor_axis[u].HomePos = MOTOR_POS_HOME;
      motor_axis[u].EncoderPos = 10 + u; /* REP/RMP in motorRecord are UINT32 */
      motor_axis[u].MotorPosNow      = motor_axis[u].EncoderPos;
      motor_axis_last[u].EncoderPos  = motor_axis[u].EncoderPos;
      motor_axis_last[u].MotorPosNow = motor_axis[u].EncoderPos;
    }
    init_done = 1;
  }
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
  ret = (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomePos);
  return ret;
}

int getAxisHomed(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  int ret;
  ret = motor_axis[axis_no].homed;
  return ret;
}

static void calcHomePos(int axis_no)
{
  if (motor_axis[axis_no].definedMinPos &&
      motor_axis[axis_no].definedMaxPos) {
    double value =
      (motor_axis[axis_no].valueMinPos + motor_axis[axis_no].valueMaxPos) / 2;
    fprintf(stdlog,
            "%s/%s:%d axis_no=%d value=%f\n",
            __FILE__, __FUNCTION__, __LINE__, axis_no, value);
    motor_axis[axis_no].HomePos = value;
  }
}

double getMinPos(int axis_no)
{
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].valueMinPos;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  return value;
}

void setMinPos(int axis_no, double value)
{
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].valueMinPos = value;
  motor_axis[axis_no].definedMinPos = 1;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d valueMinPos=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          motor_axis[axis_no].valueMinPos);
  calcHomePos(axis_no);
}

void enableMinPos(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d (ignored) value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
}

double getMaxPos(int axis_no)
{
  double value = 0;
  AXIS_CHECK_RETURN_ZERO(axis_no);
  value = motor_axis[axis_no].valueMaxPos;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d value=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  return value;
}

void setMaxPos(int axis_no, double value)
{
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].valueMaxPos = value;
  motor_axis[axis_no].definedMaxPos = 1;
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d valueMaxPos=%f\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no,
          motor_axis[axis_no].valueMaxPos);
  calcHomePos(axis_no);
}

void enableMaxPos(int axis_no, int value)
{
  fprintf(stdlog,
          "%s/%s:%d axis_no=%d (ignored) value=%d\n",
          __FILE__, __FUNCTION__, __LINE__, axis_no, value);
  AXIS_CHECK_RETURN(axis_no);
}


double getMotorPos(int axis_no)
{
  struct timeval timeNow;

  AXIS_CHECK_RETURN_ZERO(axis_no);
  gettimeofday(&timeNow, NULL);

  if (motor_axis[axis_no].velo.JogVelocity) {
    /* Simulate jogging  */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.JogVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);
  }

  if (motor_axis[axis_no].velo.PosVelocity) {
    /* Simulate a move to postion */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.PosVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);

    if (((motor_axis[axis_no].velo.PosVelocity > 0) &&
         (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].MotorPosWanted)) ||
        ((motor_axis[axis_no].velo.PosVelocity < 0) &&
         (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].MotorPosWanted))) {
      /* overshoot or undershoot. Clip the value  */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].MotorPosWanted;
      motor_axis[axis_no].velo.PosVelocity = 0;
    }
  }

  if (motor_axis[axis_no].velo.HomeVelocity) {
    /* Simulate move to home */
    motor_axis[axis_no].MotorPosNow += motor_axis[axis_no].velo.HomeVelocity *
      (timeNow.tv_sec - motor_axis[axis_no].lastPollTime.tv_sec);

    if (((motor_axis[axis_no].velo.HomeVelocity > 0) &&
         (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].HomePos)) ||
        ((motor_axis[axis_no].velo.HomeVelocity < 0) &&
         (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].HomePos))) {
      /* overshoot or undershoot. Clip the value  */
      motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].HomePos;
    }
  }
  if (motor_axis[axis_no].MotorPosNow == motor_axis[axis_no].HomePos) {
    motor_axis[axis_no].velo.HomeVelocity = 0;
    motor_axis[axis_no].homed = 1;
  }

  motor_axis[axis_no].lastPollTime = timeNow;
  if (motor_axis[axis_no].valueMaxPos > motor_axis[axis_no].valueMinPos) {
    /* Soft limits defined: Clip the value  */
    if (motor_axis[axis_no].definedMaxPos) {
      if (motor_axis[axis_no].MotorPosNow > motor_axis[axis_no].valueMaxPos) {
        motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].valueMaxPos;
      }
    }
    if (motor_axis[axis_no].definedMinPos) {
      if (motor_axis[axis_no].MotorPosNow < motor_axis[axis_no].valueMinPos) {
        motor_axis[axis_no].MotorPosNow = motor_axis[axis_no].valueMinPos;
      }
    }
  }

  if (memcmp(&motor_axis_last[axis_no].velo, &motor_axis[axis_no].velo, sizeof(motor_axis[axis_no].velo)) ||
      motor_axis_last[axis_no].MotorPosNow     != motor_axis[axis_no].MotorPosNow ||
      motor_axis_last[axis_no].MotorPosWanted  != motor_axis[axis_no].MotorPosWanted) {
      fprintf(stdlog,
              "%s/%s:%d axis_no=%d MotorPosWanted=%f JogVelocity=%g PosVelocity=%g HomeVelocity=%g home=%d MotorPosNow=%f\n",
              __FILE__, __FUNCTION__, __LINE__,
              axis_no,
              motor_axis[axis_no].MotorPosWanted,
              motor_axis[axis_no].velo.JogVelocity,
              motor_axis[axis_no].velo.PosVelocity,
              motor_axis[axis_no].velo.HomeVelocity,
              getAxisHome(axis_no),
              motor_axis[axis_no].MotorPosNow);
      memcpy(&motor_axis_last[axis_no], &motor_axis[axis_no], sizeof(motor_axis[axis_no]));
    }


  /* This simulation has motorPos == EncoderPos */
  motor_axis[axis_no].EncoderPos = motor_axis[axis_no].MotorPosNow;
  return motor_axis[axis_no].MotorPosNow;
}

double getEncoderPos(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  if (motor_axis_reported[axis_no].EncoderPos != motor_axis[axis_no].EncoderPos) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d EncoderPos=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].EncoderPos);
    motor_axis_reported[axis_no].EncoderPos = motor_axis[axis_no].EncoderPos;
  }
  return motor_axis[axis_no].EncoderPos;
}

char getpLimits(int axis_no)
{
  AXIS_CHECK_RETURN_ZERO(axis_no);
  return motor_axis[axis_no].pLimits == '0' ? '0' : '1';
}

void setpLimits(int axis_no, char value)
{
  AXIS_CHECK_RETURN(axis_no);
  motor_axis[axis_no].pLimits = (value == '0' ? '0' : '1');
}

/* Stop the ongoing motion (like JOG),
   to be able to start a new one (like HOME)
*/
static void StopInternal(int axis_no)
{
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


/* caput pv.HOMF, caput pv.HOMR */
int moveHome(int axis_no,
             int direction,
             double max_velocity,
             double acceleration)
{
  double position = motor_axis[axis_no].HomePos;
  double velocity = max_velocity ? max_velocity :  MOTOR_VEL_HOME;
  double HomeVelocityAbsWanted = fabs(velocity);

  motor_axis[axis_no].HomeVelocityAbsWanted = HomeVelocityAbsWanted;
  if (fabs(velocity) > MOTOR_VEL_HOME_MAX) {
    velocity = 0;
  }

  fprintf(stdlog, "%s/%s:%d axis_no=%d direction=%d max_velocity=%g HomeVelocityAbsWanted=%g velocity=%g acceleration=%g\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no,
          direction,
          max_velocity,
          HomeVelocityAbsWanted,
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


int motorStop(int axis_no)
{
  fprintf(stdlog, "%s/%s:%d axis_no=%d\n",
          __FILE__, __FUNCTION__, __LINE__,
          axis_no);
  StopInternal(axis_no);
  return 0;
}

int getNegLimitSwitch(int axis_no)
{
  motor_axis[axis_no].hitNegLimitSwitch =
    motor_axis[axis_no].definedMinPos &&
    (motor_axis[axis_no].MotorPosNow <= motor_axis[axis_no].valueMinPos);

  if (motor_axis_reported[axis_no].hitNegLimitSwitch != motor_axis[axis_no].hitNegLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedMinPos=%d MotorPosNow=%f valueMinPos=%f hitNegLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedMinPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].valueMinPos,
            motor_axis[axis_no].hitNegLimitSwitch);
    motor_axis_reported[axis_no].hitNegLimitSwitch = motor_axis[axis_no].hitNegLimitSwitch;
  }
  return motor_axis[axis_no].hitNegLimitSwitch;
}

int getPosLimitSwitch(int axis_no)
{
  motor_axis[axis_no].hitPosLimitSwitch =
    motor_axis[axis_no].definedMaxPos &&
    (motor_axis[axis_no].MotorPosNow >= motor_axis[axis_no].valueMaxPos);

  if (motor_axis_reported[axis_no].hitPosLimitSwitch != motor_axis[axis_no].hitPosLimitSwitch) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d definedMaxPos=%d MotorPosNow=%f valueMaxPos=%f hitPosLimitSwitch=%d\n",
            __FILE__, __FUNCTION__, __LINE__,
            axis_no,
            motor_axis[axis_no].definedMaxPos,
            motor_axis[axis_no].MotorPosNow,
            motor_axis[axis_no].valueMaxPos,
            motor_axis[axis_no].hitPosLimitSwitch);
    motor_axis_reported[axis_no].hitPosLimitSwitch = motor_axis[axis_no].hitPosLimitSwitch;
  }
  return motor_axis[axis_no].hitPosLimitSwitch;
}

int get_bError(motor_axis_no)
{
  double HomeVelocityAbsWanted = motor_axis[motor_axis_no].HomeVelocityAbsWanted;
  if (HomeVelocityAbsWanted > MOTOR_VEL_HOME_MAX) {
    fprintf(stdlog, "%s/%s:%d axis_no=%d HomeVelocityAbsWanted=%f\n",
            __FILE__, __FUNCTION__, __LINE__,
            motor_axis_no,
           HomeVelocityAbsWanted);
    return 1;
  }
  return 0;
}
