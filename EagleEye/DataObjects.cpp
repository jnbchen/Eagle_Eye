
#include "DataObjects.h"

using namespace DerWeg;


// State
bool State::operator== (const State& s) const throw () {
  return position==s.position && stddev==s.stddev && orientation==s.orientation &&
          velocity==s.velocity && yawrate==s.yawrate && timestamp==s.timestamp;
}
bool State::operator!= (const State& s) const throw () {
  return position!=s.position || stddev!=s.stddev || orientation!=s.orientation ||
          velocity!=s.velocity || yawrate!=s.yawrate || timestamp!=s.timestamp;
}


// ReferenceTrajectory
bool ReferenceTrajectory::operator== (const ReferenceTrajectory& rt) const throw () {
  return path == rt.path;
}
bool ReferenceTrajectory::operator!= (const ReferenceTrajectory& rt) const throw () {
  return path != rt.path;
}

/*
// DrivingMode
bool DrivingMode::operator== (const DrivingMode& dm) const throw () {
  return current_mode==dm.current_mode && next_mode==dm.next_mode;
}
bool DrivingMode::operator!= (const DrivingMode& dm) const throw () {
  return current_mode!=dm.current_mode || next_mode!=dm.next_mode;
}
*/
