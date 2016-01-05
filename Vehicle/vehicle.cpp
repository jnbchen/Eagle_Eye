
#include <cmath>
#include "vehicle.h"

using namespace DerWeg;

MotorCommand::MotorCommand () throw () : rpm(0), steer(0) {;}
MotorCommand::MotorCommand (const MotorCommand& m) throw () { operator= (m); }
MotorCommand::~MotorCommand () throw () {;}
const MotorCommand& MotorCommand::operator= (const MotorCommand& m) throw () {
  rpm=m.rpm;
  steer=m.steer;
  return *this;
}

/** For old motor */
MotorCommand_oldcar::MotorCommand_oldcar () throw () : pwmspeed(128), steer(0) {;}
MotorCommand_oldcar::MotorCommand_oldcar (const MotorCommand_oldcar& m_oldcar) throw () { operator= (m_oldcar); }
MotorCommand_oldcar::~MotorCommand_oldcar () throw () {;}
const MotorCommand_oldcar& MotorCommand_oldcar::operator= (const MotorCommand_oldcar& m_oldcar) throw () {
  pwmspeed=m_oldcar.pwmspeed;
  steer=m_oldcar.steer;
  return *this;
}

MotorFeedback::MotorFeedback () throw () : rpm(0), steer(0) {;}
MotorFeedback::MotorFeedback (const MotorFeedback& m) throw () { operator= (m); }
MotorFeedback::~MotorFeedback () throw () {;}
const MotorFeedback& MotorFeedback::operator= (const MotorFeedback& m) throw () {
  rpm=m.rpm;
  steer=m.steer;
  return *this;
}

/** For old motor */
MotorFeedback_oldcar::MotorFeedback_oldcar () throw () : pwmspeed(128), steer(0) {;}
MotorFeedback_oldcar::MotorFeedback_oldcar (const MotorFeedback_oldcar& m_oldcar) throw () { operator= (m_oldcar); }
MotorFeedback_oldcar::~MotorFeedback_oldcar () throw () {;}
const MotorFeedback_oldcar& MotorFeedback_oldcar::operator= (const MotorFeedback_oldcar& m_oldcar) throw () {
  pwmspeed=m_oldcar.pwmspeed;
  steer=m_oldcar.steer;
  return *this;
}

Velocity::~Velocity() throw () {;}
Velocity::Velocity() throw (): velocity(0) {;}
Velocity::Velocity(const Velocity& o) throw () { operator= (o); }
const Velocity& Velocity::operator= (const Velocity& o) throw () {
  velocity = o.velocity;
  steer = o.steer;
  return *this;
}
bool Velocity::operator== (const Velocity& v) const throw () {
  return velocity==v.velocity && steer==v.steer;
}
bool Velocity::operator!= (const Velocity& v) const throw () {
  return velocity!=v.velocity || steer!=v.steer;
}

Odometry::~Odometry() throw () {;}
Odometry::Odometry() throw (): velocity(0) {;}
Odometry::Odometry(const Odometry& o) throw () { operator= (o); }
const Odometry& Odometry::operator= (const Odometry& o) throw () {
  velocity = o.velocity;
  steer = o.steer;
  return *this;
}
bool Odometry::operator== (const Odometry& v) const throw () {
  return velocity==v.velocity && steer==v.steer;
}
bool Odometry::operator!= (const Odometry& v) const throw () {
  return velocity!=v.velocity || steer!=v.steer;
}

Pose::Pose () throw () : position(0,0), stddev(0), velocity(0), yawrate(0) {;}
Pose::Pose (const Pose& p) throw () { operator= (p); }
Pose::~Pose () throw () {;}
const Pose& Pose::operator= (const Pose& p) throw () {
  position=p.position;
  stddev=p.stddev;
  orientation=p.orientation;
  velocity=p.velocity;
  yawrate=p.yawrate;
  timestamp=p.timestamp;
  return *this;
}
bool Pose::operator== (const Pose& p) const throw () {
  return position==p.position && stddev==p.stddev && orientation==p.orientation && velocity==p.velocity && yawrate==p.yawrate && timestamp==p.timestamp;
}
bool Pose::operator!= (const Pose& p) const throw () {
  return position!=p.position || stddev!=p.stddev || orientation!=p.orientation || velocity!=p.velocity || yawrate!=p.yawrate || timestamp!=p.timestamp;
}
