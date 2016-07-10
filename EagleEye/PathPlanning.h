
#ifndef _DerWeg_PATHPLANNING_H__
#define _DerWeg_PATHPLANNING_H__

#include "../Blackboard/Blackboard.h"
#include "../Elementary/ThreadSafeLogging.h"

#include "DataObjects.h"
#include "../Elementary/Vec.h"
#include "../Elementary/Angle.h"
#include "../Vehicle/vehicle.h"

using namespace std;

namespace DerWeg{


class PathPlanning {
private:
    // How many steps the tree is expanded
    int max_depth;
    // All seen obstacles modeled as circles
    vector<Circle> obstacles;

    // How long a is Velocity applied in seconds for path segment simulation
    double dt;

    // Big number to penalize collisions with
    double collision_penalty;

    double axis_distance;

    double car_circle_radius;

public:
    // Empty default constructor
    PathPlanning() {}

    PathPlanning(const ConfigReader& cfg);

    Velocity findPath(vector<Circle> obst);

private:
    double treeSearch(const State state, const int depth, Velocity& minimizing) const;

    vector<Velocity> getVelocities(const State state) const;

    double simulatePath(State& state) const;

    // ICM - Momentanpol
    // vehicle point: point on the vehicle for which collision should be checked
    // vehicle point end: end point of the given vehicle point after the circular path segment
    // Direction flag: 1 in left curve, -1 in right curve, 0 in straight movement
    double calculateDistance(const Vec& ICM, const Circle& obstacle, const Circle& vehicle_point,
                            const Circle& vehicle_point_end, const int direction_flag) const;

    vector<Circle> getCarCircles(const State state) const;

};

} // Namespace

#endif // _DerWeg_PATHPLANNING_H__
