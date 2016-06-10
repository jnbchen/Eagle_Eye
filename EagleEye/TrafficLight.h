#ifndef _DerWeg_TRAFFICLIGHT_H__
#define _DerWeg_TRAFFICLIGHT_H__

#include <cmath>
#include "../Elementary/Vec.h"


namespace DerWeg {

// none, if no traffic light was detected
// "red" represents the red signal, yellow signal and red-yellow signal
enum TrafficLightState {none=0, red=1, green=2};

/*
struct TL_StateTransition {
    bool has_changed;
    TrafficLightState previous_state;
    TrafficLightState new_state;

    TL_StateTransition(bool changed, TrafficLightState previous_st, TrafficLightState new_st) :
        has_changed(changed), previous_state(previous_st), new_state(new_st) {}
};
*/

class TrafficLight {

private:
    // If an observation is made, it requires a certain number of observations in a row
    // before the state is accepted as a new state
    TrafficLightState potential_state;
    // Count the observations of a new state in a row to eventually accept it
    int potential_state_observations;
    // how many votes are needed for a transition from one state to another
    // This is used because it could happen that there is some image with no traffic light recognized, even if there was one just before.
    // Hence, for a transition from green to none it requires e.g. 3 images in a row where
    int min_observations;

public:
    TrafficLightState state;
    Vec position;
    double distance;

    TrafficLight(int transition_votes);

    void observe_state(TrafficLightState signal);

    void observe_position(Vec pos, double distance, double confidence = 1);

};




enum DrivingMode {drive_on, stop, accelerate};

class TrafficLightBehaviour {

private:
    double default_acceleration;
    double default_deceleration;
    //duration of yellow signal
    double yellow_phase;
    // distance to the point where the car has to stop before traffic light
    // (calculated via arclength or eucildean distance, should both work similarly)
    double halt_point_distance;
    // distance to the projection point of the traffic light onto the curve
    double projected_distance;

    TrafficLightState last_known_state;

    //determines whether stopping, accelerating or driving on with same speed
    DrivingMode mode;
    //maximal acceleration/deceleration to use in trajectory generator
    //double max_acceleration;

public:
    TrafficLightBehaviour(double a, double yellow_t) :
        default_acceleration(a), default_deceleration(a), last_known_state(none), yellow_phase(yellow_t),
        mode(drive_on) {}

    void calculate_curve_distances(TrafficLight& tlight, Segment seg);

    void process_state(TrafficLight& tlight, double current_velocity);

    double calculate_velocity(TrafficLight& tlight, double current_velocity, Segment seg);
};

} // namespace DerWeg

#endif // TRAFFICLIGHT_H__
