//--------------------------------------------------------------------------------------------------
// telescope mount control, goto planning zones

#include "GotoZones.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_PLANNER == ON && GOTO_PLANNER_GRID == ON

#include "../../Mount.h"
#include "../../site/Site.h"
#include "../../limits/Limits.h"

#ifdef GOTO_PLANNER_BOXES
  // boxes in instrument axis coordinates, {axis1 min, axis1 max, axis2 min, axis2 max} in degrees
  static const float boxes[][4] = GOTO_PLANNER_BOXES;
  #define BOX_COUNT (sizeof(boxes)/sizeof(boxes[0]))
#endif

void GotoZones::clearance(double a1, double a2, PierSide *lastPierSide, float *clear) {
  evaluations++;

  // boxes are in instrument coordinates, the clearance is the distance outside along the furthest axis
  clear[GZ_BOXES] = GZ_CLEAR;
  #ifdef GOTO_PLANNER_BOXES
    for (unsigned int i = 0; i < BOX_COUNT; i++) {
      float c = (float)(degToRad(boxes[i][0]) - a1);
      c = max(c, (float)(a1 - degToRad(boxes[i][1])));
      c = max(c, (float)(degToRad(boxes[i][2]) - a2));
      c = max(c, (float)(a2 - degToRad(boxes[i][3])));
      clear[GZ_BOXES] = min(clear[GZ_BOXES], c);
    }
  #endif

  // the existing limits, worked out on the mount coordinate as Limits::poll() does
  Coordinate mount = transform.instrumentToMount(a1, a2, *lastPierSide, 1.0e-7);
  *lastPierSide = mount.pierSide;
  if (transform.mountType == ALTALT) transform.aaToHor(&mount); else
  if (transform.isEquatorial()) transform.equToAlt(&mount);

  if (fabs(limits.settings.altitude.max - Deg90) > OneArcSec) {
    clear[GZ_OVERHEAD] = limits.settings.altitude.max - mount.a;
  } else clear[GZ_OVERHEAD] = GZ_CLEAR;
  clear[GZ_HORIZON] = mount.a - limits.settings.altitude.min;

  clear[GZ_MERIDIAN_E] = GZ_CLEAR;
  clear[GZ_MERIDIAN_W] = GZ_CLEAR;
  if (transform.mountType == GEM) {
    if (mount.pierSide == PIER_SIDE_EAST) clear[GZ_MERIDIAN_E] = mount.h + limits.settings.pastMeridianE;
    if (mount.pierSide == PIER_SIDE_WEST) clear[GZ_MERIDIAN_W] = limits.settings.pastMeridianW - mount.h;
  }

  double limitA1 = mount.a1;
  if (transform.mountType != GEM && mount.pierSide == PIER_SIDE_WEST) limitA1 += Deg180;
  clear[GZ_AXIS1_MIN] = limitA1 - axis1.getLimitMin();
  clear[GZ_AXIS1_MAX] = axis1.getLimitMax() - limitA1;
  clear[GZ_AXIS2_MIN] = mount.a2 - axis2.getLimitMin();
  clear[GZ_AXIS2_MAX] = axis2.getLimitMax() - mount.a2;
}

bool GotoZones::isFolded(uint8_t zone) {
  if (zone == GZ_AXIS2_MIN) return !isPoleNorth() && axis2.getLimitMin() <= -Deg90 + OneArcSec;
  if (zone == GZ_AXIS2_MAX) return isPoleNorth() && axis2.getLimitMax() >= Deg90 - OneArcSec;
  return false;
}

double GotoZones::poleAxis2() {
  return isPoleNorth() ? Deg90 : -Deg90;
}

// the same choice as Transform::instrumentToMount()
bool GotoZones::isPoleNorth() {
  return site.location.latitude >= 0.0 || transform.mountType == ALTAZM;
}

GotoZones gotoZones;

#endif
