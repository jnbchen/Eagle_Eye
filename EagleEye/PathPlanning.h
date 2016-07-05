
#ifndef _DerWeg_PATHPLANNING_H__
#define _DerWeg_PATHPLANNING_H__

#include "../Blackboard/Blackboard.h"
#include "../Elementary/ThreadSafeLogging.h"

#include "DataObjects.h"
#include "../Elementary/Vec.h"
#include "../Vehicle/vehicle.h"

using namespace std;

namespace DerWeg{

struct Circle {
    Vec center;
    double radius;
};


class PathPlanning {
private:
    // How many steps the tree is expanded
    int max_depth;
    // All seen obstacles modeled as circles
    vector<Circle> obstacles;
    // How long a is Velocity applied in seconds
    double time_step_duration;
    // How many steps until a new velocity is set
    int constant_steps;

    double collision_penalty;

public:
    Velocity findPath(vector<Circle> obst);

private:
    Velocity treeSearch(State state, int depth, Velocity& minimizing);

    vector<Velocity> getVelocities(State state);

    vector<State> simulatePath(Velocity v, double dt);

    bool collisionCheck(State state)

    double stateValue(State state);

};

} // Namespace

#endif // _DerWeg_PATHPLANNING_H__
