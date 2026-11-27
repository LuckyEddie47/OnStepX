//--------------------------------------------------------------------------------------------------
// telescope mount control, goto planning zones
#pragma once

#include "../../../../Common.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_PLANNER == ON && GOTO_PLANNER_GRID == ON

#include "../../coordinates/Transform.h"

// the zones a goto plans around: the first GZ_AVOID are kept clear on every leg, the others are the existing
// limits, kept clear on the legs to and from a waypoint so a route can't take the mount outside them
enum GotoZone: uint8_t {GZ_OVERHEAD, GZ_BOXES, GZ_HORIZON, GZ_MERIDIAN_E, GZ_MERIDIAN_W,
                        GZ_AXIS1_MIN, GZ_AXIS1_MAX, GZ_AXIS2_MIN, GZ_AXIS2_MAX, GZ_COUNT};
#define GZ_AVOID 2

// clearance of a zone that doesn't apply at this position
#define GZ_CLEAR 1.0e6F

class GotoZones {
  public:
    // clearance of each zone at an instrument position, in radians, negative inside
    // each is a lower bound on the distance to the zone, and changes by no more than |a1 change| + |a2 change|
    // within one pier side, so a line or box can be checked from its midpoint; lastPierSide carries the pier side
    // through a pole crossing
    void clearance(double a1, double a2, PierSide *lastPierSide, float *clear);

    // true for a limit at the pole, which the mount passes by folding, so a route need not keep clear of it
    bool isFolded(uint8_t zone);

    // the instrument axis2 coordinate of the pole, where the pier side changes
    double poleAxis2();

    // true in the hemisphere whose instrument coordinates put the pole at axis2 +90 degrees
    bool isPoleNorth();

    // clearance evaluations since power up
    unsigned long evaluations = 0;
};

extern GotoZones gotoZones;

#endif
