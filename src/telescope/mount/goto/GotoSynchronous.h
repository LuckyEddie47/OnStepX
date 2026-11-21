//--------------------------------------------------------------------------------------------------
// telescope mount control, synchronous goto axis motion
#pragma once

#include "../../../Common.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_SYNCHRONOUS == ON

#include "../../../lib/axis/Axis.h"

class GotoSynchronous {
  public:
    // set the goto acceleration rates the axes are scaled from, in radians per second per second
    void setAccelerationRates(float rate, float rateAbort);

    // start both axes on their targets so they move on a straight line in axis coordinates
    // rate1 is the axis1 slew rate and rate2 the axis2 slew rate, in radians per second
    CommandError start(float rate1, float rate2);

    // start any axis whose start is delayed
    void poll();

    // cancel any delayed start
    inline void cancel() { delayedAxis = 0; }

  private:
    // scale each axis rate and acceleration by its share of the distance
    void scale(float *rate1, float *rate2);

    // time this axis spends taking up backlash at the start of the slew to its target, in seconds
    float backlashTakeupTime(Axis *axis);

    float accelerationRate = 0.0F;
    float accelerationRateAbort = 0.0F;

    // the axis with less backlash to take up is started later so both begin moving together
    uint8_t delayedAxis = 0;
    float delayedRate = 0.0F;
    unsigned long delayedStartUs = 0;
};

extern GotoSynchronous gotoSynchronous;

#endif
