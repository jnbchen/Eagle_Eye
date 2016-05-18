
#include "DataObjects.h"

using namespace DerWeg;


// State
/*
State::State () throw () : position(0,0), stddev(0), velocity(0), yawrate(0) {;}
State::State (const State& s) throw () { operator= (s); }
State::~State () throw () {;}
const State& State::operator= (const State& s) throw () {
  position=s.position;
  stddev=s.stddev;
  orientation=p.orientation;
  velocity=p.velocity;
  yawrate=p.yawrate;
  timestamp=p.timestamp;
  return *this;
  ...
}
*/
bool State::operator== (const State& s) const throw () {
  return position==s.position && stddev==s.stddev && orientation==s.orientation && velocity==s.velocity && yawrate==s.yawrate && timestamp==s.timestamp;
}
bool State::operator!= (const State& s) const throw () {
  return position!=s.position || stddev!=s.stddev || orientation!=s.orientation || velocity!=s.velocity || yawrate!=s.yawrate || timestamp!=s.timestamp;
}


// ReferenceCurve
/*
Constructor
Destructor
...
*/
bool ReferenceCurve::operator== (const ReferenceCurve& rc) const throw () {
  return x1==rc.x1 && x2==rc.x2 && x3==rc.x3 && x4==rc.x4 && y1==rc.y1 && y2==rc.y2 && y3==rc.y3 && y4==rc.y4;
}
bool ReferenceCurve::operator!= (const ReferenceCurve& rc) const throw () {
  return x1!=rc.x1 || x2!=rc.x2 || x3!=rc.x3 || x4!=rc.x4 || y1!=rc.y1 || y2!=rc.y2 || y3!=rc.y3 || y4!=rc.y4;
}


// DrivingMode

/*
DrivingMode::DrivingMode () throw () : current_mode(0), next_mode(0) {;}
DrivingMode::DrivingMode (const DrivingMode& dm) throw () { operator= (dm); }
DrivingMode::~DrivingMode () throw () {;}
const DrivingMode& DrivingMode::operator= (const DrivingMode& dm) throw () {
  current_mode=dm.current_mode;
  next_mode=dm.next_mode;
  return *this;
}
*/
bool DrivingMode::operator== (const DrivingMode& dm) const throw () {
  return current_mode==dm.current_mode && next_mode==dm.next_mode;
}
bool DrivingMode::operator!= (const DrivingMode& dm) const throw () {
  return current_mode!=dm.current_mode || next_mode!=dm.next_mode;
}
