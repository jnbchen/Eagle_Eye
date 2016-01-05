
#include "HeadingFilter.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include "../Elementary/ThreadSafeLogging.h"

namespace {
  inline double square(double x) { return x*x; }
  inline double deg2rad(double x) { return x*M_PI/180; }
  inline double normalize_angle(double x, double r=0) { return x+std::floor(0.5+(r-x)/(2*M_PI))*2*M_PI; }
  double string2double (const std::string& s) throw (std::invalid_argument) {
    char* end_char;
    double d = strtod (s.c_str(), &end_char);
    if ((*end_char)!='\0')
      throw std::invalid_argument (std::string("invalid string conversion to double. Original string was: ")+s);
    return d;
  }
}


DerWeg::HeadingFilter::HeadingFilter () : reference_time (boost::posix_time::microsec_clock::universal_time()) {
  state.setZero();
  state_covariance = 1e+10*Eigen::Matrix<double, 3, 3>::Identity ();

  state_transition_matrix = Eigen::Matrix<double, 3, 3>::Identity ();
  state_transition_covariance.setZero();

  gyro_observation_matrix.setZero();
  gyro_observation_matrix(0,1)=1;
  gyro_observation_matrix(0,2)=1;
  gyro_observation_covariance.setZero();
  gyro_observation_covariance(0,0)=square(deg2rad(0.1));

  gyro_nomotion_observation_matrix.setZero();
  gyro_nomotion_observation_matrix(0,2)=1;
  gyro_nomotion_observation_matrix(1,1)=1;
  gyro_nomotion_observation_covariance(0,0)=square(deg2rad(0.3));
  gyro_nomotion_observation_covariance(1,1)=square(deg2rad(0.3));
  gyro_nomotion_observation(1)=0;

  yaw_observation_matrix.setZero();
  yaw_observation_matrix(0,0)=1;
  yaw_observation_covariance.setZero();
}

DerWeg::HeadingFilter::~HeadingFilter () {;}

void DerWeg::HeadingFilter::predict (boost::posix_time::ptime t) {
  double dt = 1e-6*static_cast<double>((t-reference_time).total_microseconds());
  state_transition_matrix(0,1)=dt;
  state_transition_covariance(0,0)=dt*square(deg2rad(0.3));
  state_transition_covariance(1,1)=dt*square(deg2rad(10.0));
  state_transition_covariance(2,2)=dt*square(deg2rad(0.001));

  reference_time = t;
  state = state_transition_matrix*state;
  Eigen::Matrix<double, 3, 3> pp;
  y_x_yt<3,3> (pp, state_covariance, state_transition_matrix);
  state_covariance = pp+state_transition_covariance;
}

void DerWeg::HeadingFilter::observe_gyro (double yaw_rate_observed) {
  gyro_observation(0) = yaw_rate_observed;
  innovation<1> (gyro_observation, gyro_observation_matrix, gyro_observation_covariance);
  state(0)=normalize_angle(state(0),M_PI);
}

void DerWeg::HeadingFilter::observe_gyro_nomotion (double yaw_rate_observed) {
  gyro_nomotion_observation(0) = yaw_rate_observed;
  innovation<2> (gyro_nomotion_observation, gyro_nomotion_observation_matrix, gyro_nomotion_observation_covariance);
  state(0)=normalize_angle(state(0),M_PI);
}

void DerWeg::HeadingFilter::observe_yaw_angle (double yaw_angle_observed, double stddev) {
  yaw_observation(0,0) = normalize_angle (yaw_angle_observed, state(0));
  yaw_observation_covariance(0,0)=square(stddev);
  innovation<1> (yaw_observation, yaw_observation_matrix, yaw_observation_covariance);
  state(0)=normalize_angle(state(0),M_PI);
}


Eigen::Matrix<double, 3, 1> DerWeg::HeadingFilter::get_state () throw () {
  Eigen::Matrix<double, 3, 1> res = state;
  return res;
}

Eigen::Matrix<double, 3, 3> DerWeg::HeadingFilter::get_covariance () throw () {
  Eigen::Matrix<double, 3, 3> res = state_covariance;
  return res;
}

boost::posix_time::ptime DerWeg::HeadingFilter::get_time () throw () {
  boost::posix_time::ptime res = reference_time;
  return res;
}

double DerWeg::HeadingFilter::get_yaw_angle () const throw () {
  return state(0);
}
