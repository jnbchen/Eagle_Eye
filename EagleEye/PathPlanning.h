
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
    // radius of the circles covering the car
    double car_circle_radius;

    // Object containing all states already simulated, so they dont have to be simulated again
    // First index for the depth
    // Second index for the state
    // State here consists of a Vec (Stargazer position) and an Angle (orientation)
    vector< vector< pair< Vec, Angle> > > simulated_states;

    double cutoff_distance; // millimetres
    double cutoff_angle; // degree
    double min_sim_velocity;

    int counter;

public:
    // Empty default constructor
    PathPlanning() {}

    PathPlanning(const ConfigReader& cfg);

    Velocity findPath(vector<Circle> obst);

private:
    double treeSearch(const State state, const int depth, Velocity& minimizing) ;

    vector<Velocity> getVelocities(const State state, const int depth) const;

    double simulatePath(State& state, const int depth) ;

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
