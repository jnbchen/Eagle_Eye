
#ifndef _DerWeg_IMUPlausibilityChecker_h_
#define _DerWeg_IMUPlausibilityChecker_h_

#include <eigen2/Eigen/Dense>
#include "IMU/CCHR6dm.h"

namespace Eigen {
  typedef Eigen::Matrix<double,6,1> Vector6d;
}

namespace DerWeg {

  /** eine Klasse, um die Plausibilitaet einer Bewegungshypothese anhand der IMU-Werte zu pruefen */
  class IMUPlausibilityChecker {
    Eigen::Vector6d sums;  ///< exponentiell geglaettete Beobachtungen der IMU-Groessen gx,gy,gz,ax,ay,az
    Eigen::Vector6d sums2; ///< quadrate der Groessen
    double discount_rate;  ///< Diskontierungsrate

    Eigen::Vector6d lower_boundary_nomotion;     ///< Untergrenze fuer gemittelte IMU-Groessen bei stehendem Fahrzeug
    Eigen::Vector6d upper_boundary_nomotion;     ///< Obergrenze fuer gemittelte IMU-Groessen bei stehendem Fahrzeug
    Eigen::Vector6d upper_boundary_std_nomotion; ///< Obergrenze fuer Standardabweichung der IMU-Groessen bei stehendem Fahrzeug

    Eigen::Vector6d lower_boundary_driving;      ///< Untergrenze fuer gemittelte IMU-Groessen bei fahrendem Fahrzeug
    Eigen::Vector6d upper_boundary_driving;      ///< Obergrenze fuer gemittelte IMU-Groessen bei fahrendem Fahrzeug

  public:
    IMUPlausibilityChecker ();
    void add (const CHR6dm::SSensorData&);   ///< neue IMU-Beobachtung einfuegen

    bool is_nomotion () const;       ///< liegen die Beobachtungen innerhalb der Grenzen fuer "keine Bewegung"?
    bool is_driving () const;        ///< liegen die Beobachtungen innerhalb der Grenzen fuer "Fahren"?
  };

}

#endif
