//--------------------------------------------------------------------------------------------------
// telescope mount control, goto planning engine: routes around the goto zones through a grid
#pragma once

#include "../../../../Common.h"

#if defined(MOUNT_PRESENT) && GOTO_FEATURE == ON && GOTO_PLANNER == ON && GOTO_PLANNER_GRID == ON

#include "../GotoPlanner.h"

typedef struct GridPoint {
  double a1;
  double a2;
} GridPoint;

typedef struct GridSettings {
  double latitude;
  float altitudeMin, altitudeMax, pastMeridianE, pastMeridianW;
  float axis1Min, axis1Max, axis2Min, axis2Max;
  int8_t mountType;
  bool meridianFlips;
} GridSettings;

class GotoPlannerGrid : public GotoPlanner {
  public:
    // build the grid at startup
    void init();

    // plan the goto segment, see GotoPlanner.h
    CommandError plan(Coordinate *from, Coordinate *to, GotoStage stage);

    // get the next waypoint of the planned route
    bool nextWaypoint(Coordinate *waypoint);

    // limit evaluations and time taken by the last plan, in microseconds
    unsigned long lastEvaluations = 0;
    unsigned long lastMicros = 0;

  private:
    // the zone kept clear on a leg: avoid only the goto zones, or also the existing limits
    bool lineClear(GridPoint s, GridPoint e, bool allLimits);
    bool boxClear(double lo1, double hi1, double lo2, double hi2, const bool *active, const float *required);
    void legRule(GridPoint s, GridPoint e, bool allLimits, bool *active, float *required);
    float spare(double a1, double a2, const bool *active, const float *required);

    // true if this instrument position converts to a mount coordinate with the same pier side and back again
    bool roundTrip(Coordinate *mount, GridPoint p);

    // (re)build the grid of cells clear of every zone, when the settings it depends on have changed
    bool updateGrid();
    bool settingsChanged();
    GridPoint cellCentre(int cell);
    int cellIndex(GridPoint p);
    inline bool isGo(int cell) { return (go[cell >> 3] >> (cell & 7)) & 1; }

    // find a route from s to e through the grid, merged into the fewest clear legs
    CommandError route(GridPoint s, GridPoint e);

    GridSettings gridSettings;
    bool gridValid = false;
    bool yieldAllowed = false;
    uint8_t *go = NULL;
    int cells1 = 0, cells2 = 0;
    double cell = 0.0, lo1 = 0.0, lo2 = 0.0;

    GridPoint waypoints[GOTO_PLANNER_MAX_WAYPOINTS];
    int waypointCount = 0;
    int waypointNext = 0;
};

#endif
