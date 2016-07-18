
#include "VehicleKinematics.h"
#include <cmath>

using namespace std;
using namespace DerWeg;

VehicleKinematics::VehicleKinematics (double f, double r) :
  distance_front_axle (f),
  distance_rear_axle (r) {;}

void VehicleKinematics::euler_step (Vec& position, Angle& heading, double velocity, Angle steering_angle, double time_interval) {
  if (steering_angle==Angle::zero) {
    // Bewegung Geradeaus
    position=position+time_interval*velocity*Vec::unit_vector(heading);   // mm + ms * mm/ms = mm
  } else {
    // Kurvenfahrt
    double distance = distance_front_axle+distance_rear_axle;
    double da = tan(steering_angle.get_rad())*velocity/distance*time_interval;   // (m/s) / mm * ms = rad
    Vec h=position-distance_rear_axle*Vec::unit_vector(heading);
    h=h+distance/tan(steering_angle)*Vec(sin(heading)*(-1+cos(da))+cos(heading)*sin(da), cos(heading)*(1-cos(da))+sin(heading)*sin(da));
    heading=heading+Angle::rad_angle(da);
    position=h+distance_rear_axle*Vec::unit_vector(heading);
  }
}

void VehicleKinematics::heun_step (Vec& position, Angle heading, double velocity1, double velocity2, Angle steering_angle1, Angle steering_angle2, double time_interval) {
  Vec p1=position;
  Vec p2=position;
  Angle h1=heading;
  Angle h2=heading;
  euler_step (p1, h1, velocity1, steering_angle1, time_interval);
  euler_step (p2, h2, velocity2, steering_angle2, time_interval);
  position=0.5*(p1+p2);
  Vec v=Vec::unit_vector(h1)+Vec::unit_vector(h2);
  if (v.squared_length()==0)
    heading = h1;
  else
    heading=v.angle();
}
