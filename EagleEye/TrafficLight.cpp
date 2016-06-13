#include "TrafficLight.h"
#include "../Blackboard/Blackboard.h"
#include <string>
#include <sstream>
#include <cmath>


namespace DerWeg {

// -------------------------------------------------------------------------------
// TrafficLight implementation
// -------------------------------------------------------------------------------

TrafficLight::TrafficLight(int transition_observations) :
    potential_state(none), potential_state_observations(0), min_observations(transition_observations) {}

void TrafficLight::observe_state(TrafficLightState signal) {
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

void TrafficLight::update_position(Vec& measurement, State& state, double distance, double confidence) {

    // Set up measurement covariance
    Matrix2d C_measure;

    // Rotate measurement covariance matrix
    double alpha = state.orientation.get_rad_pi() +
                  std::atan2(measurement.y, measurement.x);
    double alpha_sin = std::sin(alpha);
    double alpha_cos = std::cos(alpha);
    Matrix2d R;
    R(0, 0) = alpha_cos;
    R(0, 1) = alpha_sin;
    R(1, 0) = -alpha_sin;
    R(1, 1) = alpha_cos;
    C_measure = R.transpose() * (C_measure * R);

    // Convert measured position to global coordinates
    measurement += state.position;
    Vector2d mean_measure(measurement.x, measurement.y);

    // Update estimate, simple average weighted by the covariances
    Matrix2d C_inv = (C_measure + C_est).inverse();
    mean_est = C_measure * (C_inv * mean_est) +
                      C_est * (C_inv * mean_measure);
    C_est = C_measure * (C_inv * C_est);
}

void TrafficLight::set_position(double x, double y) {
    mean_est(0) = x;
    mean_est(1) = y;
}

void TrafficLight::set_covar(double var_x, double var_y, double covar) {
    C_est(0, 0) = var_x;
    C_est(1, 1) = var_y;
    C_est(0, 1) = covar;
    C_est(1, 0) = covar;
}

Vec TrafficLight::get_position() const {
    return Vec(mean_est(0), mean_est(1));
}

void TrafficLight::plot_estimate(const std::string& color) const {
    // compare:
    // http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

    // Plot estimated position in AnicarViewer
    std::stringstream plt;
    /*
    plt << "thick " << color << " plus "
        << mean_est(0) << " " << mean_est(1) << "\n";
    */

    // Plot axes of covariance ellipse
    Eigen::EigenSolver<Matrix2d> eigsolve(C_est, true);
    Vector2d eigvals = eigsolve.eigenvalues().real();
    Matrix2d eigvecs = eigsolve.eigenvectors().real();
    int ind_max_eigval;
    double max_eigval = eigvals.maxCoeff(&ind_max_eigval);
    int ind_min_eigval;
    double min_eigval = eigvals.minCoeff(&ind_min_eigval);

    // Get the 95% confidence interval error ellipse
    double chisquare_val = 2.4477;
    double a = chisquare_val * std::sqrt(max_eigval);
    double b = chisquare_val * std::sqrt(min_eigval);

    /*
    double phi = std::atan2(eigvecs(1, ind_max_eigval), eigvecs(0, ind_max_eigval));
    double phi_sin = std::sin(phi);
    double phi_cos = std::cos(phi);
    Matrix2d R;
    R(0, 0) = phi_cos;
    R(0, 1) = phi_sin;
    R(1, 0) = -phi_sin;
    R(1, 1) = phi_cos;
    */

    Vector2d p1 = mean_est + eigvecs.col(0);
    Vector2d p2 = mean_est - eigvecs.col(0);
    Vector2d p3 = mean_est + eigvecs.col(0);
    Vector2d p4 = mean_est - eigvecs.col(0);

    plt << "thick solid darkBlue line "
    << p1(0) << " " << p1(1) << " "
    << p2(0) << " " << p2(1) << " "
    << p3(0) << " " << p3(1) << " "
    << p4(0) << " " << p4(1) << "\n";

    BBOARD->addPlotCommand(plt.str());
}



// -------------------------------------------------------------------------------
// TrafficLightBehaviour implementation
// -------------------------------------------------------------------------------

void TrafficLightBehaviour::calculate_curve_distances(TrafficLight& tlight, Segment seg) {
    halt_point_distance = 1;
    projected_distance = 1;
}

void TrafficLightBehaviour::process_state(TrafficLight& tlight, double current_velocity) {
    if (last_known_state == tlight.state) {
        return;
    }

    if (last_known_state == green && tlight.state == red) {
        // All calculations are based on the assumption of braking with constant deceleration
        // The formulas are simple physics (integration)

        // puffer distance to drive on and pass traffic light before it switches from yellow to red
        double puffer_drive = current_velocity * yellow_phase - projected_distance;
        // puffer distance to break before stop line with the default deceleration
        double puffer_brake = std::pow(current_velocity, 2) / (2 * default_deceleration) - halt_point_distance;
        //pass traffic light before it goes to red only if this has a larger puffer distance
        // and a positive buffer
        if (puffer_drive > 0 && puffer_drive > puffer_brake) {
            mode = drive_on;
        } else {
            mode = stop;
        }

    } else if (last_known_state == none && tlight.state == red) {
        mode = stop;
    } else if (tlight.state == green) {
        mode = drive_on;
    }

    last_known_state = tlight.state;

}

double TrafficLightBehaviour::calculate_velocity(TrafficLight& tlight, double current_velocity, Segment seg) {
    calculate_curve_distances(tlight, seg);
    process_state(tlight, current_velocity);

    if (mode == drive_on) {
        return current_velocity;
    } else if (mode == stop) {
        // All calculations are based on the assumption of braking with constant deceleration
        // The formulas are simple physics (integration)
        double emergency_brake_deceleration = std::pow(current_velocity, 2) / (2 * halt_point_distance);
        double max_deceleration = std::max(emergency_brake_deceleration, default_deceleration);
        return std::pow(2 * max_deceleration * halt_point_distance, 0.5);
    }

}

}


