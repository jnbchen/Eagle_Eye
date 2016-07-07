

#include <iostream>
#include "Blackboard.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>

using namespace DerWeg;
using namespace std;

Blackboard::Blackboard() : exitProgram(false), active(false) {};

Blackboard::~Blackboard() {};

Blackboard* Blackboard::instance = 0;

Blackboard* Blackboard::getInstance() {
  if ( !instance )
    instance = new Blackboard;
  return instance;
}

void Blackboard::setExitProgram() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  exitProgram = true;
}
bool Blackboard::getExitProgram() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return exitProgram;
}


void Blackboard::setActive(const bool b) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  active = b;
  condActive.notify_all();
  condDesiredVelocity.notify_all();
}
bool Blackboard::getActive() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return active;
}
bool Blackboard::waitForActive (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condActive.timed_wait(lock, timeout);
}

ImageBuffer Blackboard::getImage() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return img.clone();
}
void Blackboard::setImage(const ImageBuffer& image) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  img=image;
  condImage.notify_all();
}
bool Blackboard::waitForImage (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condImage.timed_wait(lock, timeout);
}


ImageBuffer Blackboard::getTopImage() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return top_img.clone();
}
void Blackboard::setTopImage(const ImageBuffer& image) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  top_img=image;
  condTopImage.notify_all();
}
bool Blackboard::waitForTopImage (boost::posix_time::time_duration timeout) {
    boost::unique_lock<boost::mutex> lock (bbmutex);
  return condTopImage.timed_wait(lock, timeout);
}


Velocity Blackboard::getDesiredVelocity () {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return desiredVelocity;
}
void Blackboard::setDesiredVelocity (const Velocity& v) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  desiredVelocity=v;
  condDesiredVelocity.notify_all();
}
bool Blackboard::waitForDesiredVelocity (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condDesiredVelocity.timed_wait(lock, timeout);
}


Pose Blackboard::getVehiclePose () {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return vehiclePose;
}
void Blackboard::setVehiclePose (const Pose& v) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  vehiclePose=v;
  condPose.notify_all();
}
bool Blackboard::waitForVehiclePose (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condPose.timed_wait(lock, timeout);
}


Odometry Blackboard::getOdometry() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return odometry;
}
void Blackboard::setOdometry (const Odometry& o) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  odometry=o;
  condOdometry.notify_all();
}
bool Blackboard::waitForOdometry (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condOdometry.timed_wait(lock, timeout);
}

string Blackboard::getMessage() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  message << flush;
  string m = message.str();
  message.str("");
  return m;
}
void Blackboard::addMessage (const string& m) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  message << m;
  if (m.length()>0 && m[m.length()-1]!='\n')
    message << '\n';
  condMessage.notify_all();
}

bool Blackboard::waitForMessage (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condMessage.timed_wait(lock, timeout);
}

string Blackboard::getPlotCommand() {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  plotcmd << flush;
  string m = plotcmd.str();
  plotcmd.str("");
  return m;
}
void Blackboard::addPlotCommand (const string& m) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  plotcmd << m;
  if (m.length()>0 && m[m.length()-1]!='\n')
    plotcmd << '\n';
  condPlotcmd.notify_all();
}

bool Blackboard::waitForPlotCommand (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condPlotcmd.timed_wait(lock, timeout);
}


//////////////////////////////////////////////////////
// Eagle Eye Data

// State
State Blackboard::getState () {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return state;
}
void Blackboard::setState (const State& s) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  state=s;
  condState.notify_all();
}
bool Blackboard::waitForState (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condState.timed_wait(lock, timeout);
}


// ReferenceTrajectory
ReferenceTrajectory Blackboard::getReferenceTrajectory () {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return reference_trajectory;
}
void Blackboard::setReferenceTrajectory (const ReferenceTrajectory& rt) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  reference_trajectory=rt;
  condReferenceTrajectory.notify_all();
}
bool Blackboard::waitForReferenceTrajectory (boost::posix_time::time_duration timeout) {
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condReferenceTrajectory.timed_wait(lock, timeout);
}


// TrafficLightData
TrafficLightData Blackboard::getTrafficLight() {
    boost::unique_lock<boost::mutex> lock (bbmutex);
    return traffic_light;
}
void Blackboard::setTrafficLight(const TrafficLightData & tl){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  traffic_light=tl;
  condTrafficLight.notify_all();
}
bool Blackboard::waitForTrafficLight(boost::posix_time::time_duration timeout){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condTrafficLight.timed_wait(lock, timeout);
}


// RectImages
RectImages Blackboard::getRectImages() {
    boost::unique_lock<boost::mutex> lock (bbmutex);
    return rect_images;
}
void Blackboard::setRectImages(const RectImages & ri){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  rect_images=ri;
  condRectImages.notify_all();
}
bool Blackboard::waitForRectImages(boost::posix_time::time_duration timeout){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condRectImages.timed_wait(lock, timeout);
}


// PylonMap
PylonMap Blackboard::getPylonMap() {
    boost::unique_lock<boost::mutex> lock (bbmutex);
    return pylon_map;
}
void Blackboard::setPylonMap(const PylonMap & pm){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  pylon_map=pm;
  condPylonMap.notify_all();
}
bool Blackboard::waitForPylonMap(boost::posix_time::time_duration timeout){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condPylonMap.timed_wait(lock, timeout);
}


// PylonMeasurements
PylonMeasurements Blackboard::getPylonMeasurements() {
    boost::unique_lock<boost::mutex> lock (bbmutex);
    PylonMeasurements temp(pylon_measurements);
    pylon_measurements.measurements.clear();
    return temp;
}
void Blackboard::setPylonMeasurements(const PylonMeasurements & pm){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  pylon_measurements=pm;
  condPylonMeasurements.notify_all();
}
bool Blackboard::waitForPylonMeasurements(boost::posix_time::time_duration timeout){
  boost::unique_lock<boost::mutex> lock (bbmutex);
  return condPylonMeasurements.timed_wait(lock, timeout);
}
void Blackboard::addPylonMeasurement(const Vec& pos) {
    boost::unique_lock<boost::mutex> lock (bbmutex);
    pylon_measurements.measurements.push_back(pos);
    condPylonMeasurements.notify_all();
}

