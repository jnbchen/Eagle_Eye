
#include "KogmolaborCommunication.h"
#include <cstring>
#include <sstream>
#include <iostream>

using namespace DerWeg;
using namespace std;

DerWeg::CommMessage::CommMessage () : index (0) {;}

void DerWeg::CommMessage::putFloat (float f) {
  for (unsigned int i=0; i<sizeof(float); ++i)
    msg+=*(reinterpret_cast<char*>(&f)+i);
}

void DerWeg::CommMessage::putString (std::string s) {
  msg+=s+'\0';
}

void DerWeg::CommMessage::putStringList (const std::vector<std::string>& sl) {
  for (unsigned int i=0; i<sl.size(); ++i) {
    msg+='N';
    putString (sl[i]);
  }
  msg+='E';
}

void DerWeg::CommMessage::putBool (bool b) {
  msg+=b;
}

void DerWeg::CommMessage::putUChar (unsigned char c) {
  msg+=c;
}

void DerWeg::CommMessage::putLInt (long int i) {
  unsigned long int u = i;
  putUChar (u & 255);
  u = u>>8;
  putUChar (u & 255);
  u = u>>8;
  putUChar (u & 255);
  u = u>>8;
  putUChar (u & 255);
}

void DerWeg::CommMessage::putTimestamp (Timestamp t) {
  putLInt (t.get_msec());
}

void DerWeg::CommMessage::putAngle (Angle a) {
  putFloat (a.get_deg());
}

void DerWeg::CommMessage::putVec (Vec xy) {
  putFloat (xy.x);
  putFloat (xy.y);
}

float DerWeg::CommMessage::getFloat () {
  float r = *reinterpret_cast<const float*>(msg.c_str()+index);
  index+=sizeof(float);
  return r;
}

std::string DerWeg::CommMessage::getString () {
  std::string r;
  while (msg[index++]!=0) {
    r+=msg[index-1];
  }
  return r;
}

std::vector<std::string> DerWeg::CommMessage::getStringList () {
  vector<string> r;
  while (msg[index++]=='N') {
    string s = getString ();
    r.push_back (s);
  }
  return r;
}

bool DerWeg::CommMessage::getBool () {
  return static_cast<bool>(msg[index++]);
}

unsigned char DerWeg::CommMessage::getUChar () {
  return static_cast<unsigned char>(msg[index++]);
}

long int DerWeg::CommMessage::getLInt () {
  unsigned long int u = 0;
  u = getUChar ();
  u |= (static_cast<unsigned long int>(getUChar ())<<8);
  u |= (static_cast<unsigned long int>(getUChar ())<<16);
  u |= (static_cast<unsigned long int>(getUChar ())<<24);
  return static_cast<long int>(u);
}

Timestamp DerWeg::CommMessage::getTimestamp () {
  Timestamp t;
  t.set_msec (getLInt());
  return t;
}

Angle DerWeg::CommMessage::getAngle () {
  return Angle::deg_angle (getFloat());
}

Vec DerWeg::CommMessage::getVec () {
  Vec xy;
  xy.x = getFloat();
  xy.y = getFloat();
  return xy;
}

const char* DerWeg::CommMessage::getMsg () const {
  return msg.c_str();
}

unsigned int DerWeg::CommMessage::getMsgLen () const {
  return msg.length();
}

void DerWeg::CommMessage::setMsg (const char* m, unsigned int l) {
  msg = string (m, l);
  index = 0;
}


DerWeg::KogmolaborCommunication::KogmolaborCommunication () : NonspecificTaggedUDPCommunication("ac", 8189) {};

DerWeg::KogmolaborCommunication::~KogmolaborCommunication ()  throw () {};

bool DerWeg::KogmolaborCommunication::putCommMessage (unsigned char key, const CommMessage& cm) {
  return socket.put (key, cm.getMsg(), cm.getMsgLen());
}

bool DerWeg::KogmolaborCommunication::getCommMessage (unsigned char key, CommMessage& cm) {
  const char* msg;
  unsigned int size;
  bool success = socket.get(key, msg, size);
  cm.setMsg (msg, size);
  return success;
}

// Uebersicht verwendeter Schluessel:
// 0: goodbye
// 1: ping
// 2: initialization
// 5: saveImage
// 101: motorCommand
// 102: motorFeedback
// 103: velocity
// 104: odometry
// 105: pose
// 106: addModule
// 107: removeModule
// 124: aktive
// 127: allModules
// 128: askForAllModules
// 129: presentModules
// 130: askForPresentModules
// 131: messages
// 132: plotCommands
// 133: motorCommand_oldcar
// 134: motorFeedback_oldcar

bool DerWeg::KogmolaborCommunication::sayGoodbye() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (0, cm);
}

bool DerWeg::KogmolaborCommunication::goodbye() {
  CommMessage cm;
  return getCommMessage (0, cm);
}

bool DerWeg::KogmolaborCommunication::putPing() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (1, cm);
}

bool DerWeg::KogmolaborCommunication::getPing() {
  CommMessage cm;
  return getCommMessage (1, cm);
}

bool DerWeg::KogmolaborCommunication::admitInitialization() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (2, cm);
}

bool DerWeg::KogmolaborCommunication::getAdmitInitialization() {
  CommMessage cm;
  return getCommMessage (2, cm);
}

bool DerWeg::KogmolaborCommunication::putSaveImage() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (5, cm);
}

bool DerWeg::KogmolaborCommunication::getSaveImage() {
  CommMessage cm;
  return getCommMessage (5, cm);
}

bool KogmolaborCommunication::putMotorCommand (const MotorCommand& m) {
  CommMessage cm;
  cm.putFloat (m.rpm);
  cm.putUChar (m.steer);
  return putCommMessage (101, cm);
}

bool KogmolaborCommunication::getMotorCommand (MotorCommand& m) {
  CommMessage cm;
  bool succ = getCommMessage (101,cm);
  if (succ) {
    m.rpm = cm.getFloat();
    m.steer = cm.getUChar();
  }
  return succ;
}

bool KogmolaborCommunication::putMotorFeedback (const MotorFeedback& m) {
  CommMessage cm;
  cm.putFloat (m.rpm);
  cm.putUChar (m.steer);
  return putCommMessage (102, cm);
}

bool KogmolaborCommunication::getMotorFeedback (MotorFeedback& m) {
  CommMessage cm;
  bool succ = getCommMessage (102,cm);
  if (succ) {
    m.rpm = cm.getFloat();
    m.steer = cm.getUChar();
  }
  return succ;
}

bool KogmolaborCommunication::putVelocity (const Velocity& m) {
  CommMessage cm;
  cm.putFloat (m.velocity);
  cm.putAngle (m.steer);
  return putCommMessage (103, cm);
}

bool KogmolaborCommunication::getVelocity (Velocity& m) {
  CommMessage cm;
  bool succ = getCommMessage (103,cm);
  if (succ) {
    m.velocity = cm.getFloat();
    m.steer = cm.getAngle();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putOdometry(const DerWeg::Odometry& m) {
  CommMessage cm;
  cm.putFloat (m.velocity);
  cm.putAngle (m.steer);
  return putCommMessage (104, cm);
}

bool DerWeg::KogmolaborCommunication::getOdometry(DerWeg::Odometry& m) {
  CommMessage cm;
  bool succ = getCommMessage (104,cm);
  if (succ) {
    m.velocity = cm.getFloat();
    m.steer = cm.getAngle();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putPose(const DerWeg::Pose& p) {
  CommMessage cm;
  cm.putVec (p.position);
  cm.putFloat (p.stddev);
  cm.putAngle (p.orientation);
  cm.putFloat (p.velocity);
  cm.putFloat (p.yawrate);
  cm.putTimestamp (p.timestamp);
  return putCommMessage (105, cm);
}

bool DerWeg::KogmolaborCommunication::getPose(DerWeg::Pose& p) {
  CommMessage cm;
  bool succ = getCommMessage (105,cm);
  if (succ) {
    p.position = cm.getVec ();
    p.stddev = cm.getFloat ();
    p.orientation = cm.getAngle ();
    p.velocity = cm.getFloat ();
    p.yawrate = cm.getFloat ();
    p.timestamp = cm.getTimestamp ();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putAddModule (const std::string& s) {
  CommMessage cm;
  cm.putString (s);
  return putCommMessage (106, cm);
}

bool DerWeg::KogmolaborCommunication::getAddModule (std::string& s) {
  CommMessage cm;
  bool succ = getCommMessage (106,cm);
  if (succ) {
    s = cm.getString();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putRemoveModule (const std::string& s) {
  CommMessage cm;
  cm.putString (s);
  return putCommMessage (107, cm);
}

bool DerWeg::KogmolaborCommunication::getRemoveModule (std::string& s) {
  CommMessage cm;
  bool succ = getCommMessage (107,cm);
  if (succ) {
    s = cm.getString();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putActive(const bool a) {
  CommMessage cm;
  cm.putBool (a);
  return putCommMessage (124, cm);
}

bool DerWeg::KogmolaborCommunication::getActive(bool& a) {
  CommMessage cm;
  bool succ = getCommMessage (124,cm);
  if (succ) {
    a = cm.getBool();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putAllModules(const std::vector<std::string>& f) {
  CommMessage cm;
  cm.putStringList (f);
  return putCommMessage (127, cm);
}

bool DerWeg::KogmolaborCommunication::getAllModules(std::vector<std::string>& f){
  CommMessage cm;
  bool succ = getCommMessage (127,cm);
  if (succ) {
    f = cm.getStringList();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::askForAllModules() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (128, cm);
}

bool DerWeg::KogmolaborCommunication::askedForAllModules() {
  CommMessage cm;
  return getCommMessage (128,cm);
}

bool DerWeg::KogmolaborCommunication::putPresentModules(const std::vector<std::string>& f) {
  CommMessage cm;
  cm.putStringList (f);
  return putCommMessage (129,cm);
}

bool DerWeg::KogmolaborCommunication::getPresentModules(std::vector<std::string>& f){
  CommMessage cm;
  bool succ = getCommMessage (129,cm);
  if (succ) {
    f = cm.getStringList();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::askForPresentModules() {
  CommMessage cm;
  cm.putBool (true);
  return putCommMessage (130,cm);
}

bool DerWeg::KogmolaborCommunication::askedForPresentModules() {
  CommMessage cm;
  return getCommMessage (130,cm);
}

bool DerWeg::KogmolaborCommunication::putMessages(const std::string& m) {
  CommMessage cm;
  cm.putString (m);
  return putCommMessage (131,cm);
}

bool DerWeg::KogmolaborCommunication::getMessages(std::string& m) {
  CommMessage cm;
  bool succ = getCommMessage (131,cm);
  if (succ) {
    m = cm.getString();
  }
  return succ;
}

bool DerWeg::KogmolaborCommunication::putPlotCommand(const std::string& m) {
  CommMessage cm;
  cm.putString (m);
  return putCommMessage (132,cm);
}

bool DerWeg::KogmolaborCommunication::getPlotCommand(std::string& m) {
  CommMessage cm;
  bool succ = getCommMessage (132,cm);
  if (succ) {
    m = cm.getString();
  }
  return succ;
}

/** For old motor */

bool KogmolaborCommunication::putMotorCommand_oldcar (const MotorCommand_oldcar& m_oldcar) {
  CommMessage cm;
  cm.putUChar (m_oldcar.pwmspeed);
  cm.putUChar (m_oldcar.steer);
  return putCommMessage (133,cm);
}

bool KogmolaborCommunication::getMotorCommand_oldcar (MotorCommand_oldcar& m_oldcar) {
  CommMessage cm;
  bool succ = getCommMessage (133,cm);
  if (succ) {
    m_oldcar.pwmspeed = cm.getUChar();
    m_oldcar.steer = cm.getUChar();
  }
  return succ;
}

bool KogmolaborCommunication::putMotorFeedback_oldcar (const MotorFeedback_oldcar& m_oldcar) {
  CommMessage cm;
  cm.putUChar (m_oldcar.pwmspeed);
  cm.putUChar (m_oldcar.steer);
  return putCommMessage (134,cm);
}

bool KogmolaborCommunication::getMotorFeedback_oldcar (MotorFeedback_oldcar& m_oldcar) {
  CommMessage cm;
  bool succ = getCommMessage (134,cm);
  if (succ) {
    m_oldcar.pwmspeed = cm.getUChar();
    m_oldcar.steer = cm.getUChar();
  }
  return succ;
}
