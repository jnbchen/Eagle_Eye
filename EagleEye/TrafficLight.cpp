#include "TrafficLight.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>
#include <cmath>

using namespace std;

namespace DerWeg {

// -------------------------------------------------------------------------------
// TrafficLight implementation
// -------------------------------------------------------------------------------

TrafficLight::TrafficLight() :
    state(none), potential_state(none), potential_state_observations(0), min_observations(3),
    covarcoeff_x(0), covarcoeff_y(0) {
        //LOUT("Constructor, state = "<< state << std::endl);
    }

TrafficLight::TrafficLight(int transition_observations, double ccoeff_x, double ccoeff_y) :
    state(none), potential_state(none), potential_state_observations(0), min_observations(transition_observations),
    covarcoeff_x(ccoeff_x), covarcoeff_y(ccoeff_y) {}

TrafficLightState TrafficLight::getState() {
    return state;
}

void TrafficLight::observe_state(TrafficLightState signal) {
    //LOUT("potential state = " << potential_state << std::endl);
    //LOUT("state = " << state << std::endl);
    //LOUT("signal = " << signal << std::endl);
    //Do nothing if the observed signal is the same as the current state
    if (state == signal) {
        //potential_state = state;
        //potential_state_observations = 0;
        return;
    }

    // if a red signal was observed, always accept it, even if it was only in one image!
    if (signal == red) {
        state = red;
        //potential_state = state;
        potential_state_observations = 0;
        return;
    }

    // If a new potential signal was observed reset counter to zero
    if (potential_state != signal) {
        potential_state = signal;
        potential_state_observations = 0;
    }
    // Update observations of potential new state
    potential_state_observations += 1;
    // If the new potential state was supported by enough observaations, accept it and reset counter
    if (potential_state_observations >= min_observations) {
        state = potential_state;
        potential_state_observations = 0;
    }
}

void TrafficLight::update_position(Vec measurement, double distance, State car_state) {
    //LOUT("measurement = " << measurement << "\n");
    Vector2d mean_measure(measurement.x, measurement.y);

    // Set up measurement covariance
    Matrix2d C_measure;
    C_measure(0, 0) = pow(distance, 2) * covarcoeff_x;
    C_measure(1, 1) = distance * covarcoeff_y;
    C_measure(0, 1) = 0;
    C_measure(1, 0) = 0;

    // Rotate measurement covariance matrix
    double x = mean_measure(0) - car_state.sg_position.x;
    double y = mean_measure(1) - car_state.sg_position.y;
    double alpha = car_state.orientation.get_rad_pi() + std::atan2(y, x);
    double alpha_sin = std::sin(alpha);
    double alpha_cos = std::cos(alpha);
    Matrix2d R;
    R(0, 0) = alpha_cos;
    R(0, 1) = alpha_sin;
    R(1, 0) = -alpha_sin;
    R(1, 1) = alpha_cos;
    C_measure = R.transpose() * (C_measure * R);

    // Update estimate, simple average weighted by the covariances
    Matrix2d C_inv = (C_measure + C_est).inverse();
    mean_est = C_measure * (C_inv * mean_est) +
                      C_est * (C_inv * mean_measure);
    C_est = C_measure * (C_inv * C_est);
}

void TrafficLight::set_position(std::vector<double> position) {
    mean_est(0) = position[0];
    mean_est(1) = position[1];
}

void TrafficLight::set_covar(std::vector<double> covar) {
    C_est(0, 0) = covar[0];
    C_est(1, 1) = covar[1];
    C_est(0, 1) = covar[2];
    C_est(1, 0) = covar[2];
}

Vec TrafficLight::get_position() const {
    return Vec(mean_est(0), mean_est(1));
}

Vec TrafficLight::get_stddev() const {
    Eigen::EigenSolver<Matrix2d> eigsolve(C_est, true);
    Vector2d eigvals = eigsolve.eigenvalues().real();
    return Vec(std::sqrt(eigvals(0)), std::sqrt(eigvals(1)));
}

void TrafficLight::plot_estimate() const {
    // compare:
    // http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

    // Plot axes of covariance ellipse
    LOUT(C_est << std::endl);

    Eigen::EigenSolver<Matrix2d> eigsolve(C_est, true);
    Vector2d eigvals = eigsolve.eigenvalues().real();
    Matrix2d eigvecs = eigsolve.eigenvectors().real();

    // Get the 95% confidence interval error ellipse
    const double chisquare_val = 2.4477;
    double a = chisquare_val * std::sqrt(eigvals(0));
    double b = chisquare_val * std::sqrt(eigvals(1));

    // Axes of 95% confidence ellipse
    Vector2d p1 = mean_est + eigvecs.col(0) * a;
    Vector2d p2 = mean_est - eigvecs.col(0) * a;
    Vector2d p3 = mean_est + eigvecs.col(1) * b;
    Vector2d p4 = mean_est - eigvecs.col(1) * b;

    // Plot in AnicarViewer
    std::stringstream plt;
    plt << "thick solid darkBlue line "
    << p1(0) << " " << p1(1) << " "
    << p2(0) << " " << p2(1) << "\n"
    << "thick solid darkBlue line "
    << p3(0) << " " << p3(1) << " "
    << p4(0) << " " << p4(1) << "\n";
    BBOARD->addPlotCommand(plt.str());
}

}


