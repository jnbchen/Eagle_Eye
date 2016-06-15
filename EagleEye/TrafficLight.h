#ifndef _DerWeg_TRAFFICLIGHT_H__
#define _DerWeg_TRAFFICLIGHT_H__

#include "../Elementary/Vec.h"
#include "DataObjects.h"
#include "Segment.h"
#include <Eigen/Dense>
#include <string>


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

typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;


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

    // Localization variables
    Vector2d mean_est;
    Matrix2d C_est;

public:
    TrafficLightState state;

    TrafficLight(int transition_votes);

    void observe_state(TrafficLightState signal);

    void update_position(Vec& measurement, State& state, double distance, double confidence = 1);
    void set_position(double x, double y);
    void set_covar(double var_x, double var_y, double covar);
    Vec get_position() const;
    void plot_estimate(const std::string& color) const;

};




enum DrivingMode {stop, drive_on, accelerate};

// IN THIS CLASS USE DISTANCES IN METRES, VELOCITIES IN METRES PER SECOND,
// ACCELERATION IN METRES PER SECONDS SQUARED !!!

class TrafficLightBehaviour {

private:
    double default_acceleration;
    double default_deceleration;
    //duration of yellow signal
    double yellow_phase = 1;
    // distance from halt point in which velocity is set to zero because it's close enough
    double halt_point_radius = 0.02;
    // distance to the point where the car has to stop before traffic light
    // (calculated via arclength or eucildean distance, should both work similarly)
    double halt_point_distance;
    // distance to the projection point of the traffic light onto the curve
    double tl_projected_distance;


    TrafficLightState last_known_state;

    //determines whether stopping, accelerating or driving on with same speed
    DrivingMode mode;

    // Variables used for finding the position of the traffic light and its projection
    // onto the curve plus the point where the car has to stop
    int qf_N = 4;
    int sppm = 4;
    int newton_steps = 20;
    double newton_tol = 0.01;
    // is the traffic light is found further than this distance from the given segment,
    // there probably something went wrong
    // This distance is still in MILLIMETRES
    double tl_max_distance_to_curve = 2000;
    // How far before the traffic light should the car stop?
    // This distance is still in MILLIMETRES
    double stopping_distance = 500;

    // Calculates the projection of the traffic light onto the current segment
    // and the point where the car has to stop on the reference curve.
    // tl_seg has to be the segment on which the traffic light is located
    // !! CALCULATE THE DISTANCES IN METRES !!
    void calculate_curve_distances(TrafficLight& tlight, Segment tl_seg, SegmentPosition current_pos);

    // Processes the current traffic light
    // Depending on its state and distance the implemented
    // logic decides whether to drive on or stop
    void process_state(TrafficLight& tlight, double current_velocity);

public:
    TrafficLightBehaviour(double a, double yellow_t) :
        default_acceleration(a), default_deceleration(a), yellow_phase(yellow_t), last_known_state(none),
        mode(drive_on) {}

    // Returns the velocity the car should drive at most to stop at the traffic light
    // tl_seg has to be the segment on which the traffic light is located
    // the parameter current_pos contains information on the position of the car on the current segment
    //
    // if the current segment is the same as tl_seg, current_pos.min_distance is set to zero
    // and the other values are set to the position on the segment
    // if the current segment is the segment prior to the tl_seg, current_pos.min_distance is set to
    // the distance remaining til the end of the current segment and the other values contain
    // the start of tl_seg
    double calculate_max_velocity(TrafficLight& tlight, double current_velocity,
                                    Segment tl_seg, SegmentPosition current_pos);
};

} // namespace DerWeg

#endif // TRAFFICLIGHT_H__
