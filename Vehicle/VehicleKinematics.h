
#ifndef _DerWeg_VehocleKinematics_h_
#define _DerWeg_VehocleKinematics_h_

#include "../Elementary/Vec.h"

namespace DerWeg {

  /** VehicleKinematics implements a kinematic model of the vehicle used for dead reckoning */
  class VehicleKinematics {
    double distance_front_axle;
    double distance_rear_axle;
  public:
    /** constructor takes distance between vehicle reference point and front axle (in mm) 
      and distance between vehicle reference point and rear axle (in mm) */
    VehicleKinematics (double f, double r);
    /** explicit Euler step for dead reckoning:
      \arg position: initial position of vehicle (in mm) and final position of vehicle (returned by method)
      \arg heading: initial orientation of vehicle and final position of vehicle (returned by method)
      \arg velocity: velocity (in m/s)
      \arg steering_angle: steering angle, positive angles means turning left
      \arg time_interval: time interval in ms for integration */
    void euler_step (Vec& position, Angle& heading, double velocity, Angle steering_angle, double time_interval);
    /** Heun step for dead reckoning:
      \arg position: initial position of vehicle (in mm) and final position of vehicle (returned by method)
      \arg heading: initial orientation of vehicle and final position of vehicle (returned by method)
      \arg velocity1: velocity (in m/s) at the begining of the time interval
      \arg velocity2: velocity (in m/s) at the end of the time interval
      \arg steering_angle1: steering angle at the begining of the time interval, positive angles means turning left
      \arg steering_angle2: steering angle at the end of the time interval, positive angles means turning left
      \arg time_interval: time interval in ms for integration */
    void heun_step (Vec& position, Angle& heading, double velocity1, double velocity2, Angle steering_angle1, Angle steering_angle2, double time_interval);
  };

}

#endif
