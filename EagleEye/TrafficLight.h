#ifndef _DerWeg_TRAFFICLIGHT_H__
#define _DerWeg_TRAFFICLIGHT_H__

#include "../Elementary/Vec.h"
#include "../Elementary/ConfigReader.h"
#include "DataObjects.h"
#include "Segment.h"
#include <eigen3/Eigen/Dense>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>


namespace DerWeg {

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
    TrafficLightState state;
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
    double covarcoeff_x;
    double covarcoeff_y;

public:
    TrafficLightState getState();

    TrafficLight(){}

    TrafficLight(int transition_votes, double covarcoeff_x, double covarcoeff_y);

    void observe_state(TrafficLightState signal);

    void update_position(cv::Mat& position, double distance, State state);
    void set_position(std::vector<double> position);
    void set_covar(std::vector<double> covar);
    Vec get_position() const;
    void plot_estimate() const;

};




enum DrivingMode {stop, drive_on, accelerate};

// IN THIS CLASS USE DISTANCES IN METRES, VELOCITIES IN METRES PER SECOND,
// ACCELERATION IN METRES PER SECONDS SQUARED !!!

class TrafficLightBehaviour {

private:
    // distance to the point where the car has to stop before traffic light
    // (calculated via arclength or eucildean distance, should both work similarly)
    // in METRES
    double halt_point_distance;
    // distance to the projection point of the traffic light onto the curve
    // in METRES
    double tl_projected_distance;

    TrafficLightState last_known_state;

    //determines whether stopping, accelerating or driving on with same speed
    DrivingMode mode;

    //================================
    //Config variables
    // in METRES PER SECOND SQUARED
    double default_acceleration;
    double default_deceleration;

    double v_max;

    //duration of yellow signal in seconds
    double yellow_phase;
    // distance from halt point in which velocity is set to zero because it's close enough
    // in METRES
    double halt_point_radius;

    // Variables used for finding the position of the traffic light and its projection
    // onto the curve plus the point where the car has to stop
    int qf_N;
    int sppm;
    int newton_steps;
    double newton_tol;
    // is the traffic light is found further than this distance from the given segment,
    // there probably something went wrong
    // This distance is still in MILLIMETRES
    double tl_max_distance_to_curve;
    // How far before the traffic light should the car stop?
    // This distance is still in MILLIMETRES
    double stopping_distance;

    // Calculates the projection of the traffic light onto the current segment
    // and the point where the car has to stop on the reference curve.
    // tl_seg has to be the segment on which the traffic light is located
    // !! CALCULATE THE DISTANCES IN METRES !!
    void calculate_curve_distances(const TrafficLightData& tlight,  Segment& tl_seg, const SegmentPosition& current_pos);

    // Processes the current traffic light
    // Depending on its state and distance the implemented
    // logic decides whether to drive on or stop
    void process_state(const TrafficLightData& tlight, double current_velocity);

public:
    // empty default constructor
    TrafficLightBehaviour() {}

    TrafficLightBehaviour(const ConfigReader& cfg);

    // Returns the velocity the car should drive at most to stop at the traffic light
    // tl_seg has to be the segment on which the traffic light is located
    // the parameter current_pos contains information on the position of the car on the current segment
    //
    // if the current segment is the same as tl_seg, current_pos.min_distance is set to zero
    // and the other values are set to the position on the segment
    // if the current segment is the segment prior to the tl_seg, current_pos.min_distance is set to
    // the distance remaining til the end of the current segment and the other values contain
    // the start of tl_seg
    double calculate_max_velocity(const TrafficLightData& tlight, double current_velocity,
                                     Segment& tl_seg, const SegmentPosition& current_pos);
};

} // namespace DerWeg

#endif // TRAFFICLIGHT_H__
