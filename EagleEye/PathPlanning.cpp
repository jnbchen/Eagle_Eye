
#include "PathPlanning.h"

Velocity PathPlanning::findPath(vector<Circle> obst) {
    obstacles = pylons;
    Velocity minimizing_velocity;
    treeSearch(BBOARD->getState(), 0, minimizing_velocity);
    return minimizing_velocity;
}

double PathPlanning::treeSearch(State state, int depth, Velocity& minimizing) {
    if (depth >= max_depth) {
        return stateValue(state);
    }

    if (collisionCheck(state)) {
        return collision_penalty;
    }

    if (depth % constant_steps == 0) {
    // sample new steering angles
        vector<Velocity> velocities getVelocities(state);
        vector<double> values;

        for (int i=0; i<velocities.size(); i++) {
            State new_state = simulatePath(velocities[i], time_step_duration);
            values.push_back(treeSearch(new_state, depth + 1, velocities[i]));
        }
        int arg_min = std::distance(v.begin(), std::min_element(v.begin(), v.end()));
        minimizing = velocities[arg_min];
        return values[arg_min];
    } else {
    // keep steering angle, in this case the argument "minimizing" is used to propagate the velocity that should be used
        State new_state = simulatePath(minimizing, time_step_duration);
        double value = treeSearch(new_state, depth + 1, minimizing);
        return value;
    }

}

vector<Velocity> PathPlanning::getVelocities(State state) {
    //TODO
}


State PathPlanning::simulatePath(Velocity v, double dt) {
    //TODO
}


double PathPlanning::stateValue(State state) {
    vector<Circle> car;
    //TODO
}

bool PathPlanning::collisionCheck(State state) {
    vector<Circle> car;
    //TODO
}
