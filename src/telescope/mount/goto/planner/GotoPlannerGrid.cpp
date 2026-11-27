//--------------------------------------------------------------------------------------------------
// telescope mount control, goto planning engine: routes around the goto zones through a grid

#include "GotoPlannerGrid.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_PLANNER == ON && GOTO_PLANNER_GRID == ON

#include <new>
#include "../../../../lib/tasks/OnTask.h"
#include "../../Mount.h"
#include "../../site/Site.h"
#include "../../limits/Limits.h"
#include "GotoZones.h"

#define MARGIN       ((float)degToRad(GOTO_PLANNER_MARGIN))
#define LINE_FLOOR   degToRad(0.1)
#define BOX_FLOOR    degToRad(0.01)
#define CONNECT      2
#define NO_CELL      -1

static_assert(GOTO_PLANNER_MARGIN >= 0.5 && GOTO_PLANNER_MARGIN <= 10.0, "Configuration (Config.h): Setting GOTO_PLANNER_MARGIN invalid, use 0.5 to 10 degrees.");
static_assert(GOTO_PLANNER_CELL >= 5.0 && GOTO_PLANNER_CELL <= 30.0, "Configuration (Config.h): Setting GOTO_PLANNER_CELL invalid, use 5 to 30 degrees.");

// the grid is built at startup, before the task scheduler runs, so without yielding
void GotoPlannerGrid::init() {
  yieldAllowed = false;
  updateGrid();
  yieldAllowed = true;
}

CommandError GotoPlannerGrid::plan(Coordinate *from, Coordinate *to, GotoStage stage) {
  UNUSED(stage);
  unsigned long startMicros = micros();
  unsigned long startEvaluations = gotoZones.evaluations;
  waypointCount = 0;
  waypointNext = 0;

  GridPoint s, e;
  transform.mountToInstrument(from, &s.a1, &s.a2);
  transform.mountToInstrument(to, &e.a1, &e.a2);
  e.a1 += to->a1Correction;

  // the start and end must convert between instrument and mount coordinates and back to the same place, they don't
  // on the west pier side of an Alt/Alt mount in the southern hemisphere (Transform uses different conventions)
  if (!roundTrip(from, s) || !roundTrip(to, {e.a1 - to->a1Correction, e.a2})) {
    VLF("WRN: Mount, goto planner can't plan this goto (coordinates don't convert back), not planned");
    return CE_NONE;
  }

  CommandError result = CE_NONE;
  if (!lineClear(s, e, false)) {
    if (updateGrid()) result = route(s, e); else result = CE_SLEW_ERR_UNSPECIFIED;
  }

  lastEvaluations = gotoZones.evaluations - startEvaluations;
  lastMicros = micros() - startMicros;
  VF("MSG: Mount, goto planner "); V(waypointCount); VF(" waypoints, "); V(lastEvaluations); VF(" evaluations, ");
  V(lastMicros/1000UL); VLF(" ms");
  return result;
}

// true if this instrument position converts to a mount coordinate with the same pier side and back again
bool GotoPlannerGrid::roundTrip(Coordinate *mount, GridPoint p) {
  Coordinate m = transform.instrumentToMount(p.a1, p.a2);
  if (mount->pierSide != PIER_SIDE_NONE && m.pierSide != mount->pierSide) return false;
  double a1, a2;
  transform.mountToInstrument(&m, &a1, &a2);
  return fabs(a1 - p.a1) < 1.0e-6 && fabs(a2 - p.a2) < 1.0e-6;
}

bool GotoPlannerGrid::nextWaypoint(Coordinate *waypoint) {
  if (waypointNext >= waypointCount) return false;
  *waypoint = transform.instrumentToMount(waypoints[waypointNext].a1, waypoints[waypointNext].a2);
  waypointNext++;
  return true;
}

// the zones a leg keeps clear: those clear at its start, by the margin, or by less where an end is closer; the leg may
// come up to twice the line check's floor closer than its nearer end (never inside a zone), otherwise an end at or
// inside the margin has no spare clearance and the line check can't show the leg is clear
void GotoPlannerGrid::legRule(GridPoint s, GridPoint e, bool allLimits, bool *active, float *required) {
  float c0[GZ_COUNT], c1[GZ_COUNT];
  PierSide side0 = PIER_SIDE_NONE, side1 = PIER_SIDE_NONE;
  gotoZones.clearance(s.a1, s.a2, &side0, c0);
  gotoZones.clearance(e.a1, e.a2, &side1, c1);
  for (int k = 0; k < GZ_COUNT; k++) {
    active[k] = c0[k] > 0.0F && (allLimits || k < GZ_AVOID) && !gotoZones.isFolded(k);
    float r = min(c0[k], max(c1[k], 0.0F));
    required[k] = min(MARGIN, max(r - 2.0F*(float)LINE_FLOOR, 0.0F));
  }
}

// the least clearance above that required, over the active zones, at one point
float GotoPlannerGrid::spare(double a1, double a2, const bool *active, const float *required) {
  float c[GZ_COUNT];
  PierSide side = PIER_SIDE_NONE;
  gotoZones.clearance(a1, a2, &side, c);
  float s = GZ_CLEAR;
  for (int k = 0; k < GZ_COUNT; k++) if (active[k]) s = min(s, c[k] - required[k]);
  return s;
}

// true if the straight line from s to e keeps clear, the line is split at the pole and then wherever the clearance at
// a piece's midpoint is less than half its length (axis1 plus axis2), until the pieces are too short
bool GotoPlannerGrid::lineClear(GridPoint s, GridPoint e, bool allLimits) {
  bool active[GZ_COUNT];
  float required[GZ_COUNT];
  legRule(s, e, allLimits, active, required);

  double d1 = e.a1 - s.a1, d2 = e.a2 - s.a2;
  double tp = d2 != 0.0 ? (gotoZones.poleAxis2() - s.a2)/d2 : -1.0;

  struct Piece { double t0, t1; };
  Piece stack[32];
  int n = 0;
  if (tp > 1.0e-9 && tp < 1.0 - 1.0e-9) { stack[n++] = {0.0, tp}; stack[n++] = {tp, 1.0}; } else stack[n++] = {0.0, 1.0};
  while (n > 0) {
    Piece p = stack[--n];
    double tm = (p.t0 + p.t1)/2.0, half = (p.t1 - p.t0)/2.0*(fabs(d1) + fabs(d2));
    float sp = spare(s.a1 + d1*tm, s.a2 + d2*tm, active, required);
    if (sp < 0.0F) return false;
    if (sp >= half) continue;
    if (half < LINE_FLOOR || n > 30) return false;
    stack[n++] = {p.t0, tm}; stack[n++] = {tm, p.t1};
  }
  return true;
}

// true if every point of the box keeps clear, the box is split at the pole and then wherever the clearance at its
// centre is less than its half size (axis1 plus axis2), until the pieces are too small
bool GotoPlannerGrid::boxClear(double lo1, double hi1, double lo2, double hi2, const bool *active, const float *required) {
  struct Box { double lo1, hi1, lo2, hi2; };
  Box stack[48];
  int n = 0;
  stack[n++] = {lo1, hi1, lo2, hi2};
  double pole = gotoZones.poleAxis2();
  while (n > 0) {
    Box b = stack[--n];
    if (b.lo2 < pole - 1.0e-9 && b.hi2 > pole + 1.0e-9) {
      if (n > 46) return false;
      stack[n++] = {b.lo1, b.hi1, b.lo2, pole}; stack[n++] = {b.lo1, b.hi1, pole, b.hi2};
      continue;
    }
    double h1 = (b.hi1 - b.lo1)/2.0, h2 = (b.hi2 - b.lo2)/2.0;
    float sp = spare(b.lo1 + h1, b.lo2 + h2, active, required);
    if (sp < 0.0F) return false;
    if (sp >= h1 + h2) continue;
    if (h1 + h2 < BOX_FLOOR || n > 46) return false;
    if (h1 >= h2) { stack[n++] = {b.lo1, b.lo1 + h1, b.lo2, b.hi2}; stack[n++] = {b.lo1 + h1, b.hi1, b.lo2, b.hi2}; }
    else { stack[n++] = {b.lo1, b.hi1, b.lo2, b.lo2 + h2}; stack[n++] = {b.lo1, b.hi1, b.lo2 + h2, b.hi2}; }
  }
  return true;
}

bool GotoPlannerGrid::settingsChanged() {
  return !gridValid ||
    gridSettings.latitude != site.location.latitude ||
    gridSettings.altitudeMin != limits.settings.altitude.min ||
    gridSettings.altitudeMax != limits.settings.altitude.max ||
    gridSettings.pastMeridianE != limits.settings.pastMeridianE ||
    gridSettings.pastMeridianW != limits.settings.pastMeridianW ||
    gridSettings.axis1Min != axis1.getLimitMin() ||
    gridSettings.axis1Max != axis1.getLimitMax() ||
    gridSettings.axis2Min != axis2.getLimitMin() ||
    gridSettings.axis2Max != axis2.getLimitMax() ||
    gridSettings.mountType != transform.mountType ||
    gridSettings.meridianFlips != transform.meridianFlips;
}

// the grid covers instrument coordinates on both pier sides where the mount flips, a cell is go if it keeps clear
// of every zone by the margin
bool GotoPlannerGrid::updateGrid() {
  if (!settingsChanged()) return true;

  unsigned long startMicros = micros();
  unsigned long startEvaluations = gotoZones.evaluations;
  gridValid = false;
  bool flips = transform.meridianFlips, north = gotoZones.isPoleNorth();
  cell = degToRad(GOTO_PLANNER_CELL);
  lo1 = axis1.getLimitMin();
  double hi1 = axis1.getLimitMax() + (flips ? Deg180 : 0.0);
  lo2 = !flips || north ? -Deg90 : -3.0*Deg90;
  double hi2 = !flips || !north ? Deg90 : 3.0*Deg90;
  cells1 = max(1L, lround((hi1 - lo1)/cell));
  cells2 = max(1L, lround((hi2 - lo2)/cell));

  delete[] go;
  go = new (std::nothrow) uint8_t[(cells1*cells2 + 7)/8];
  if (go == NULL) { DLF("WRN: Mount, goto planner grid allocation failed"); return false; }
  memset(go, 0, (cells1*cells2 + 7)/8);

  bool active[GZ_COUNT];
  float required[GZ_COUNT];
  for (int k = 0; k < GZ_COUNT; k++) { active[k] = !gotoZones.isFolded(k); required[k] = MARGIN; }

  for (int i = 0; i < cells1; i++) {
    for (int j = 0; j < cells2; j++) {
      int c = i*cells2 + j;
      if (boxClear(lo1 + i*cell, lo1 + (i + 1)*cell, lo2 + j*cell, lo2 + (j + 1)*cell, active, required)) go[c >> 3] |= 1 << (c & 7);
    }
    if (yieldAllowed) Y;
  }

  gridSettings.latitude = site.location.latitude;
  gridSettings.altitudeMin = limits.settings.altitude.min;
  gridSettings.altitudeMax = limits.settings.altitude.max;
  gridSettings.pastMeridianE = limits.settings.pastMeridianE;
  gridSettings.pastMeridianW = limits.settings.pastMeridianW;
  gridSettings.axis1Min = axis1.getLimitMin();
  gridSettings.axis1Max = axis1.getLimitMax();
  gridSettings.axis2Min = axis2.getLimitMin();
  gridSettings.axis2Max = axis2.getLimitMax();
  gridSettings.mountType = transform.mountType;
  gridSettings.meridianFlips = transform.meridianFlips;
  gridValid = true;

  VF("MSG: Mount, goto planner grid "); V(cells1); VF("x"); V(cells2); VF(" cells, ");
  V(gotoZones.evaluations - startEvaluations); VF(" evaluations, "); V((micros() - startMicros)/1000UL); VLF(" ms");
  return true;
}

GridPoint GotoPlannerGrid::cellCentre(int c) {
  return {lo1 + (c/cells2 + 0.5)*cell, lo2 + (c%cells2 + 0.5)*cell};
}

int GotoPlannerGrid::cellIndex(GridPoint p) {
  int i = (int)floor((p.a1 - lo1)/cell), j = (int)floor((p.a2 - lo2)/cell);
  i = min(cells1 - 1, max(0, i));
  j = min(cells2 - 1, max(0, j));
  return i*cells2 + j;
}

// Dijkstra over the go cells, 8-connected (a diagonal step only where both cells beside it are go too), the start and
// end joined through exactly checked legs to go cells nearby; the route is then merged into the fewest clear legs
CommandError GotoPlannerGrid::route(GridPoint s, GridPoint e) {
  int cells = cells1*cells2, nodeS = cells, nodeE = cells + 1;
  float *dist = new (std::nothrow) float[cells + 2];
  float *toEnd = new (std::nothrow) float[cells];
  int16_t *prev = new (std::nothrow) int16_t[cells + 2];
  uint8_t *done = new (std::nothrow) uint8_t[(cells + 2 + 7)/8];
  CommandError result = CE_SLEW_ERR_OUTSIDE_LIMITS;

  if (dist == NULL || toEnd == NULL || prev == NULL || done == NULL) {
    DLF("WRN: Mount, goto planner search allocation failed");
    result = CE_SLEW_ERR_UNSPECIFIED;
    goto finish;
  }

  for (int c = 0; c < cells + 2; c++) { dist[c] = 1.0e30F; prev[c] = NO_CELL; }
  for (int c = 0; c < cells; c++) toEnd[c] = -1.0F;
  memset(done, 0, (cells + 2 + 7)/8);

  // join the start and end to go cells nearby
  dist[nodeS] = 0.0F;
  for (int pass = 0; pass < 2; pass++) {
    GridPoint p = pass == 0 ? s : e;
    int cp = cellIndex(p), pi = cp/cells2, pj = cp%cells2;
    for (int i = max(0, pi - CONNECT); i <= min(cells1 - 1, pi + CONNECT); i++) {
      for (int j = max(0, pj - CONNECT); j <= min(cells2 - 1, pj + CONNECT); j++) {
        int c = i*cells2 + j;
        if (!isGo(c)) continue;
        GridPoint m = cellCentre(c);
        float d = hypot(m.a1 - p.a1, m.a2 - p.a2)/cell;
        if (pass == 0) { if (lineClear(p, m, true)) { dist[c] = d; prev[c] = nodeS; } }
        else { if (lineClear(m, p, true)) toEnd[c] = d; }
      }
      Y;
    }
  }

  for (int settled = 0;; settled++) {
    if ((settled & 31) == 31) Y;
    int u = NO_CELL;
    for (int c = 0; c < cells + 2; c++) {
      if ((done[c >> 3] >> (c & 7)) & 1) continue;
      if (dist[c] < 1.0e29F && (u == NO_CELL || dist[c] < dist[u])) u = c;
    }
    if (u == NO_CELL || u == nodeE) break;
    done[u >> 3] |= 1 << (u & 7);
    if (u == nodeS) continue;

    if (toEnd[u] >= 0.0F && dist[u] + toEnd[u] < dist[nodeE]) { dist[nodeE] = dist[u] + toEnd[u]; prev[nodeE] = u; }
    int i = u/cells2, j = u%cells2;
    for (int di = -1; di <= 1; di++) {
      for (int dj = -1; dj <= 1; dj++) {
        if (!di && !dj) continue;
        int ni = i + di, nj = j + dj;
        if (ni < 0 || nj < 0 || ni >= cells1 || nj >= cells2) continue;
        int n = ni*cells2 + nj;
        if (!isGo(n)) continue;
        if (di && dj && (!isGo(i*cells2 + nj) || !isGo(ni*cells2 + j))) continue;
        float d = dist[u] + (di && dj ? 1.41421356F : 1.0F);
        if (d < dist[n]) { dist[n] = d; prev[n] = u; }
      }
    }
  }

  if (prev[nodeE] == NO_CELL) { VLF("MSG: Mount, goto planner found no route"); goto finish; }

  {
    // the route as a list of nodes from the start, reusing toEnd's storage for the list
    int16_t *path = (int16_t *)toEnd;
    int length = 0;
    for (int c = nodeE; c != NO_CELL && length < cells; c = prev[c]) path[length++] = c;
    for (int k = 0; k < length/2; k++) { int16_t t = path[k]; path[k] = path[length - 1 - k]; path[length - 1 - k] = t; }

    // fewest legs: from each point take the furthest route point the line to which keeps clear
    int at = 0;
    GridPoint from = s;
    while (at < length - 1) {
      int best = NO_CELL;
      for (int k = length - 1; k > at; k--) {
        GridPoint to = path[k] == nodeE ? e : cellCentre(path[k]);
        if (lineClear(from, to, true)) { best = k; break; }
        Y;
      }
      if (best == NO_CELL) { VLF("MSG: Mount, goto planner route legs not clear"); waypointCount = 0; goto finish; }
      if (path[best] == nodeE) break;
      if (waypointCount >= GOTO_PLANNER_MAX_WAYPOINTS) { VLF("MSG: Mount, goto planner route has too many waypoints"); waypointCount = 0; goto finish; }
      from = cellCentre(path[best]);

      // a waypoint must convert to a mount coordinate and back to the same place
      Coordinate mount = transform.instrumentToMount(from.a1, from.a2);
      if (!roundTrip(&mount, from)) {
        VLF("MSG: Mount, goto planner waypoint doesn't convert to mount coordinates");
        waypointCount = 0;
        goto finish;
      }

      waypoints[waypointCount++] = from;
      at = best;
    }
    result = CE_NONE;
  }

  finish:
  delete[] dist;
  delete[] toEnd;
  delete[] prev;
  delete[] done;
  return result;
}

static GotoPlannerGrid gotoPlannerGrid;
GotoPlanner& gotoPlanner = gotoPlannerGrid;

#endif
