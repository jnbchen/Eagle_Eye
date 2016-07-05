#include "TrafficLightBehaviour.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>
#include <cmath>

using namespace std;

namespace DerWeg {

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
    cfg.get("LongitudinalControl::v_max", v_max);
    default_acceleration = default_deceleration;

    last_known_state = none;
    mode = drive_on;
    halt_point_distance = 100000;
    tl_projected_distance = 100000;
}

void TrafficLightBehaviour::calculate_curve_distances(const TrafficLightData& tlight,  Segment& tl_seg,
                                                        const SegmentPosition& current_pos) {
    if (tlight.state == none) {
    // if no traffic light detected, set values very high to not stop anywhere
        tl_projected_distance = 100000;
        halt_point_distance = 100000;
        return;
    }

    // seed the segment the traffic light is on to find the curve closest to the traffic light
    SegmentPosition tl_pos = tl_seg.find_segment_position(tlight.position, sppm, qf_N);
    if (tl_pos.min_distance > tl_max_distance_to_curve) {
        EOUT("Error in calculate_curve_distances, possibly the traffic light is not on the given segment" << endl
             << "Distance of traffic light to curve is "<< tl_pos.min_distance<<endl
             << "Projected position = " << tl_seg.get(tl_pos.curve_id)(tl_pos.curve_parameter) << endl
             << "Traffic light pos = " << tlight.position << endl
             << "tl_state = " << tlight.state << endl);

    }
    // Calculate the projected parameter on the curve closest to the traffic light
    double tl_curve_param = tl_seg.get(tl_pos.curve_id).project(
                    tlight.position, tl_pos.curve_parameter, newton_tol, newton_steps);

    // Calculate the distance in METRES to the projection point of the traffic light onto the curve
    // first arclength to traffic light in millimetres
    //LOUT("CTRL: car = " << current_pos.curve_id << ", " << current_pos.curve_parameter << "\t"
    //    << "tl = " << tl_pos.curve_id <<", "<<tl_curve_param<<std::endl);
    double length = tl_seg.arc_length(current_pos.curve_id, current_pos.curve_parameter,
                                            tl_pos.curve_id, tl_curve_param, qf_N);
    if (length >= 0) {
        tl_projected_distance = 0.001 * ( current_pos.min_distance + length);
    } else {
        EOUT("Error, invalid arguments in Segment.arc_length(). Call from calculate_curve_distances. Returned "<< length << endl);
        // If error, estimate arc length by euclidean distance
        // This should only occur if the car passed the traffic light, hence the negative sign
        tl_projected_distance = 0.001 * ((tlight.position - BBOARD->getState().control_position).length());
    }
    // Calculate the distance in METRES to the stopping point
    halt_point_distance = tl_projected_distance - 0.001 * stopping_distance;
    //LOUT("Halt_pt_dist = " << halt_point_distance << endl);

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
                                                        Segment& tl_seg, const SegmentPosition& current_pos) {
    if (tlight.state == none) {
        return v_max;
    }
    calculate_curve_distances(tlight, tl_seg, current_pos);
    process_state(tlight, current_velocity);

    if (mode == drive_on) {
        //LOUT("Current vel " << current_velocity << "\n");
        return min(v_max, current_velocity + 0.3);
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
        double v = std::pow(2 * max_deceleration * halt_point_distance, 0.5);
        //LOUT("case stop, v = " << v << "\n");
        return v;
    } else {
        //LOUT("Case else v_max = " << v_max <<"\n");
        return v_max;
    }

}

}
