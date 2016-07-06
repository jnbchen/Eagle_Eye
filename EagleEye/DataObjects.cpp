
#include "DataObjects.h"

using namespace DerWeg;


// State
bool State::operator== (const State& s) const throw () {
  return sg_position==s.sg_position && stddev==s.stddev && orientation==s.orientation &&
          velocity==s.velocity && yawrate==s.yawrate && timestamp==s.timestamp;
}
bool State::operator!= (const State& s) const throw () {
  return sg_position!=s.sg_position || stddev!=s.stddev || orientation!=s.orientation ||
          velocity!=s.velocity || yawrate!=s.yawrate || timestamp!=s.timestamp;
}


// ReferenceTrajectory
bool ReferenceTrajectory::operator== (const ReferenceTrajectory& rt) const throw () {
  return path == rt.path;
}
bool ReferenceTrajectory::operator!= (const ReferenceTrajectory& rt) const throw () {
  return path != rt.path;
}
