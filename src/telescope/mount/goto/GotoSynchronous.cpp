//--------------------------------------------------------------------------------------------------
// telescope mount control, synchronous goto axis motion

#include "GotoSynchronous.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_SYNCHRONOUS == ON

#include "../Mount.h"

void GotoSynchronous::setAccelerationRates(float rate, float rateAbort) {
  accelerationRate = rate;
  accelerationRateAbort = rateAbort;
}

CommandError GotoSynchronous::start(float rate1, float rate2) {
  scale(&rate1, &rate2);

  float delay = backlashTakeupTime(&axis1) - backlashTakeupTime(&axis2);
  delayedAxis = 0;
  if (delay >= 1.0F/FRACTIONAL_SEC) { delayedAxis = 2; delayedRate = rate2; } else
  if (delay <= -1.0F/FRACTIONAL_SEC) { delayedAxis = 1; delayedRate = rate1; }
  delayedStartUs = micros() + lroundf(fabsf(delay)*1000000.0F);

  CommandError e = CE_NONE;
  if (delayedAxis != 1) e = axis1.autoGoto(rate1);
  if (e == CE_NONE && delayedAxis != 2) e = axis2.autoGoto(rate2);
  if (e != CE_NONE) delayedAxis = 0;

  return e;
}

void GotoSynchronous::poll() {
  if (delayedAxis == 0 || (long)(micros() - delayedStartUs) < 0) return;

  if (delayedAxis == 1) axis1.autoGoto(delayedRate); else axis2.autoGoto(delayedRate);
  delayedAxis = 0;
}

void GotoSynchronous::scale(float *rate1, float *rate2) {
  float rate = *rate1;
  double distance1 = axis1.getTargetDistance();
  double distance2 = axis2.getTargetDistance();
  double distance = max(distance1, distance2*(rate / *rate2));
  if (distance <= 0.0) return;

  // the axes cap acceleration at the backlash rate per frac-sec so the shares start from the capped rate
  float cap = min(axis1.getBacklashFrequency(), axis2.getBacklashFrequency())*FRACTIONAL_SEC;
  float accel = min(accelerationRate, cap);
  float accelAbort = min(accelerationRateAbort, cap);

  if (distance1 > 0.0) {
    float share = distance1/distance;
    *rate1 = rate*share;
    axis1.setSlewAccelerationRate(accel*share);
    axis1.setSlewAccelerationRateAbort(accelAbort*share);
  }
  if (distance2 > 0.0) {
    float share = distance2/distance;
    *rate2 = rate*share;
    axis2.setSlewAccelerationRate(accel*share);
    axis2.setSlewAccelerationRateAbort(accelAbort*share);
  }
}

float GotoSynchronous::backlashTakeupTime(Axis *axis) {
  long instrument = axis->getInstrumentCoordinateSteps();
  long toTarget = axis->getTargetCoordinateSteps() - instrument;
  if (toTarget == 0) return 0.0F;

  long takenUp = axis->getMotorPositionSteps() - (instrument - axis->getIndexPositionSteps());
  long amount = lround(axis->getBacklash()*axis->getStepsPerMeasure());
  long remaining = toTarget > 0 ? amount - takenUp : takenUp;
  if (remaining <= 0) return 0.0F;

  return remaining/(axis->getBacklashFrequency()*axis->getStepsPerMeasure());
}

GotoSynchronous gotoSynchronous;

#endif
