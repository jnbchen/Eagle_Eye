#include "TrafficLight.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>
#include <cmath>

using namespace std;

namespace DerWeg {

// -------------------------------------------------------------------------------
// TrafficLight implementation
// -------------------------------------------------------------------------------

TrafficLight::TrafficLight(int transition_observations, double ccoeff_x, double ccoeff_y) :
    potential_state(none), potential_state_observations(0), min_observations(transition_observations),
    covarcoeff_x(ccoeff_x), covarcoeff_y(ccoeff_y) {}

TrafficLightState TrafficLight::getState() {
    return state;
}

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

void TrafficLight::update_position(cv::Mat& measurement, double distance, State state) {
    Vector2d mean_measure(measurement.at<double>(0,0), measurement.at<double>(1,0));

    // Convert to millimetres
    distance *= 1000;

    // Set up measurement covariance
    Matrix2d C_measure;
    C_measure(0, 0) = pow(distance, 2) * covarcoeff_x;
    C_measure(1, 1) = distance * covarcoeff_y;
    C_measure(0, 1) = 0;
    C_measure(1, 0) = 0;

    // Rotate measurement covariance matrix
    double x = mean_measure(0) - state.position.x;
    double y = mean_measure(1) - state.position.y;
    double alpha = state.orientation.get_rad_pi() + std::atan2(y, x);
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

void TrafficLight::plot_estimate() const {
    // compare:
    // http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/

    // Plot estimated position in AnicarViewer
    std::stringstream plt;

    // Plot axes of covariance ellipse
    Eigen::EigenSolver<Matrix2d> eigsolve(C_est, true);
    // Vector2d eigvals = eigsolve.eigenvalues().real();
    Matrix2d eigvecs = eigsolve.eigenvectors().real();
    // int ind_max_eigval;
    // double max_eigval = eigvals.maxCoeff(&ind_max_eigval);
    // int ind_min_eigval;
    // double min_eigval = eigvals.minCoeff(&ind_min_eigval);

    // Get the 95% confidence interval error ellipse
    // double chisquare_val = 2.4477;
    // double a = chisquare_val * std::sqrt(max_eigval);
    // double b = chisquare_val * std::sqrt(min_eigval);

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
    Vector2d p3 = mean_est + eigvecs.col(1);
    Vector2d p4 = mean_est - eigvecs.col(1);

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

TrafficLightBehaviour::TrafficLightBehaviour(const ConfigReader& cfg) {
    cfg.get("TrafficLightBehaviour::yellow_phase", yellow_phase);
    cfg.get("TrafficLightBehaviour::newton_tolerance", newton_tol);
    cfg.get("TrafficLightBehaviour::newton_max_iter", newton_steps);
    cfg.get("TrafficLightBehaviour::sppm", sppm);
    cfg.get("TrafficLightBehaviour::qf_N", qf_N);
    cfg.get("TrafficLightBehaviour::tl_max_distance_to_curve", tl_max_distance_to_curve);
    cfg.get("TrafficLightBehaviour::stopping_distance", stopping_distance);
    cfg.get("TrafficLightBehaviour::halt_point_radius", halt_point_radius);
    cfg.get("TrafficLightBehaviour::default_deceleration", default_deceleration);
    default_acceleration = default_deceleration;

    last_known_state = none;
    mode = drive_on;
    halt_point_distance = 100000;
    tl_projected_distance = 100000;
}

void TrafficLightBehaviour::calculate_curve_distances(const TrafficLightData& tlight, Segment tl_seg,
                                                        SegmentPosition current_pos) {
    if (tlight.state == none) {
    // if no traffic light detected, set values very high to not stop anywhere
        tl_projected_distance = 100000;
        halt_point_distance = 100000;
        return;
    }

    // seed the segment the traffic light is on to find the curve closest to the traffic light
    SegmentPosition tl_pos = tl_seg.find_segment_position(tlight.position, sppm, qf_N);
    if (tl_pos.min_distance > tl_max_distance_to_curve) {
        EOUT("Error in calculate_curve_distances, possibly the traffic light is not on the given segment" << endl);
    }
    // Calculate the projected parameter on the curve closest to the traffic light
    double tl_curve_param = tl_seg.get(tl_pos.curve_id).project(
                    tlight.position, tl_pos.curve_parameter, newton_tol, newton_steps);

    // Calculate the distance in METRES to the projection point on the traffic light onto the curve
    double length = tl_seg.arc_length(current_pos.curve_id, current_pos.curve_parameter,
                                            tl_pos.curve_id, tl_curve_param, qf_N);
    if (length >= 0) {
        tl_projected_distance = 0.001 * ( current_pos.min_distance + length);
    } else {
        EOUT("Error, invalid arguments in Segment.arc_length(). Call from calculate_curve_distances" << endl);
        // If error, estimate arc length by euclidean distance
        // This should only occur if the car passed the traffic light, hence the negative sign
        tl_projected_distance = -0.001 * ((tlight.position - BBOARD->getState().position).length());
    }
    // Calculate the distance in METRES to the stopping point
    halt_point_distance = tl_projected_distance - 0.001 * stopping_distance;

}

void TrafficLightBehaviour::process_state(const TrafficLightData& tlight, double current_velocity) {
    if (last_known_state == tlight.state) {
        return;
    }

    if (last_known_state == green && tlight.state == red) {
        // All calculations are based on the assumption of braking with constant deceleration
        // The formulas are simple physics (integration)

        // puffer distance to drive on and pass traffic light before it switches from yellow to red
        double puffer_drive = current_velocity * yellow_phase - tl_projected_distance;
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

    LOUT("tl_state: " << tlight.state << "   mode: " << mode << std::endl);

}

double TrafficLightBehaviour::calculate_max_velocity(const TrafficLightData& tlight, double current_velocity,
                                                        Segment tl_seg, SegmentPosition current_pos) {
    calculate_curve_distances(tlight, tl_seg, current_pos);
    process_state(tlight, current_velocity);

    if (mode == drive_on) {
        return current_velocity;
    } else if (mode == stop) {
        // All calculations are based on the assumption of braking with constant deceleration
        // The formulas are simple physics (integration)
        // If the car is far enough away, the returned velocity will be higher than the car can drive
        // because this is the velocity the car can have to stop at the given distance, using max_deceleration

        if (halt_point_distance < halt_point_radius) {
            // Close enough to stop!
            // filters all negative halt point distances, and zero (prevents division by zero)
            return 0;
        }
        double emergency_brake_deceleration = std::pow(current_velocity, 2) / (2 * halt_point_distance);
        double max_deceleration = std::max(emergency_brake_deceleration, default_deceleration);
        return std::pow(2 * max_deceleration * halt_point_distance, 0.5);
    }

}

}


