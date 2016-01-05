// this header contains blackboard
#include "Elementary/Blackboard.h"

void f () {
  ...
  // get vehicle pose:
  Pose p = BBOARD->getVehiclePose ();
  ...
  // set desired velocity:
  Velocity v;
  // desired velocity of 1.0m/s
  v.velocity = 1.0;
  // desired steering angle of 10 degree
  v.steer = Angle::deg_angle (10);
  // write desired velocity and steering angle to blackboard
  BBOARD->setDesiredVelocity (v);
  ...
}