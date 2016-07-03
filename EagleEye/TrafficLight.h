#ifndef _DerWeg_TRAFFICLIGHT_H__
#define _DerWeg_TRAFFICLIGHT_H__

#include "../Elementary/Vec.h"
#include "../Elementary/ConfigReader.h"
#include "DataObjects.h"
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

    TrafficLight();

    TrafficLight(int transition_votes, double covarcoeff_x, double covarcoeff_y);

    void observe_state(TrafficLightState signal);

    void update_position(Vec position, double distance, State state);
    void set_position(std::vector<double> position);
    void set_covar(std::vector<double> covar);
    Vec get_position() const;
    Vec get_stddev() const;
    void plot_estimate() const;

};

} // namespace DerWeg

#endif // TRAFFICLIGHT_H__
