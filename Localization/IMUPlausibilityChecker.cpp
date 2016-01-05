
#include "IMUPlausibilityChecker.h"
#include "../Elementary/ThreadSafeLogging.h"
#include <cmath>

namespace {
  Eigen::Vector6d square(const Eigen::Vector6d& v) {
    Eigen::Vector6d res;
    res(0)=v(0)*v(0);
    res(1)=v(1)*v(1);
    res(2)=v(2)*v(2);
    res(3)=v(3)*v(3);
    res(4)=v(4)*v(4);
    res(5)=v(5)*v(5);
    return res;
  }
  Eigen::Vector6d sqrt(const Eigen::Vector6d& v) {
    Eigen::Vector6d res;
    res(0)=std::sqrt(v(0));
    res(1)=std::sqrt(v(1));
    res(2)=std::sqrt(v(2));
    res(3)=std::sqrt(v(3));
    res(4)=std::sqrt(v(4));
    res(5)=std::sqrt(v(5));
    return res;
  }
}

DerWeg::IMUPlausibilityChecker::IMUPlausibilityChecker () {
  sums.setZero();
  sums2.setZero();
  sums(5)=950;
  sums2(5)=950*950;

  lower_boundary_nomotion(0)=lower_boundary_nomotion(1)=lower_boundary_nomotion(2)=-10;
  upper_boundary_nomotion(0)=upper_boundary_nomotion(1)=upper_boundary_nomotion(2)=+10;
  upper_boundary_std_nomotion(0)=upper_boundary_std_nomotion(1)=upper_boundary_std_nomotion(2)=+1;
  lower_boundary_nomotion(3)=lower_boundary_nomotion(4)=-100;
  upper_boundary_nomotion(3)=upper_boundary_nomotion(4)=+100;
  lower_boundary_nomotion(5)=900;
  upper_boundary_nomotion(5)=1000;
  upper_boundary_std_nomotion(3)=upper_boundary_std_nomotion(4)=upper_boundary_std_nomotion(5)=+30;

  lower_boundary_driving(0)=lower_boundary_driving(1)=-100;
  upper_boundary_driving(0)=upper_boundary_driving(1)=+100;
  lower_boundary_driving(2)=-1e300;
  upper_boundary_driving(2)=+1e300;
  lower_boundary_driving(3)=lower_boundary_driving(4)=-1e300;
  upper_boundary_driving(3)=upper_boundary_driving(4)=+1e300;
  lower_boundary_driving(5)=+700;
  upper_boundary_driving(5)=+1e300;
  discount_rate = 0.99;
}

void DerWeg::IMUPlausibilityChecker::add (const CHR6dm::SSensorData& s) {
  Eigen::Vector6d y;
  Eigen::Vector6d y2;
  y(0)=s.gx;
  y(1)=s.gy;
  y(2)=s.gz;
  y(3)=s.ax;
  y(4)=s.ay;
  y(5)=s.az;
  sums = discount_rate*sums+(1-discount_rate)*y;
  sums2 = discount_rate*sums2+(1-discount_rate)*square(y);
}

bool DerWeg::IMUPlausibilityChecker::is_nomotion () const {
  bool is_okay=true;
  Eigen::Vector6d mu = sums;
  Eigen::Vector6d sdev = sqrt(sums2-square(mu));
  for (unsigned int i=0; i<6 && is_okay; ++i) {
    is_okay &= (mu(i)>lower_boundary_nomotion(i)) && (mu(i)<upper_boundary_nomotion(i)) && (sdev(i)<upper_boundary_std_nomotion(i));
  }
  return is_okay;
}

bool DerWeg::IMUPlausibilityChecker::is_driving () const {
  bool is_okay=true;
  Eigen::Vector6d mu = sums;
  for (unsigned int i=0; i<6 && is_okay; ++i) {
    is_okay &= (mu(i)>lower_boundary_driving(i)) && (mu(i)<upper_boundary_driving(i));
  }
  return is_okay;
}
