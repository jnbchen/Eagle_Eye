#include "TrafficLight.h"

using namespace DerWeg;

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

void TrafficLight::observe_position(Vec pos, double distance, double confidence) {
    position = pos;
    this->distance = distance;
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


