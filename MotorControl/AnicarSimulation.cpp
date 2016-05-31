
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Vehicle/VehicleKinematics.h"
#include "../Blackboard/Blackboard.h"
#include "../Localization/SLVelocitySensor.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <sstream>
#include <cmath>
#include <iostream>

namespace DerWeg {

  class AnicarSimulation : public KogmoThread {
    VehicleKinematics kin;
    SLVelocitySensor vsens;

    // internes Simulationsmodell
    Timestamp timestamp;
    Vec position;
    Angle heading;
    double velocity;
    Angle steering_angle;

    // gesetzte (Soll-)Geschwindigkeiten
    Angle des_steering_angle1;
    Angle des_steering_angle2;
    Angle des_steering_angle2opt;
    double des_velocity1;
    double des_velocity2;
  public:

    AnicarSimulation() : kin (0, 527), velocity(0) {;}
    ~AnicarSimulation() {;}

    void init(const ConfigReader& cfg) {
      std::vector<double> x;
      cfg.get ("AnicarSimulation::init", x);
      if (x.size()>=2) {
        position = Vec (x[0], x[1]);
        if (x.size()>=3) {
          heading = Angle::deg_angle (x[2]);
        }
      }

      // Write simulated starting pose to blackboard
      Pose pose;
      pose.position = position;
      pose.orientation = heading;
      BBOARD->setVehiclePose(pose);
    }
    void execute () {
      boost::mt19937 generator;
      boost::normal_distribution<> normal_dist (0,1);
      boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > normal_rnd (generator, normal_dist);
      try {
        while (true) {
          Velocity bbdv = BBOARD->getDesiredVelocity();
          bool bbac = BBOARD->getActive();
          // Teil 1: neuen Lenkwinkel (aus letztem Zyklus -> Latenz) setzen
          steering_angle = des_steering_angle1;
          des_steering_angle1 = des_steering_angle2;

          // Teil 2: neuen Lenkwinkel bestimmen
          Angle ds = bbdv.steer;
          if (ds.in_between(Angle::deg_angle(30),Angle::half))
            ds=Angle::deg_angle(30);
          else if (ds.in_between(Angle::half,Angle::deg_angle(-30)))
            ds=Angle::deg_angle(-30);
          double deltaa = (ds-des_steering_angle2opt).get_deg_180();
          if (deltaa>0.5) {
            des_steering_angle2opt=ds;
            des_steering_angle2=ds-Angle::deg_angle(2)+0.7*Angle::deg_angle(normal_rnd());
          } else if (deltaa<-0.5) {
            des_steering_angle2opt=ds;
            des_steering_angle2=ds+Angle::deg_angle(2)+0.7*Angle::deg_angle(normal_rnd());
          }

          // Teil 3: Schlupf und tatsaechliche Geschwindigkeit bestimmen
          des_velocity1=0.5*des_velocity1+0.5*des_velocity2;
          des_velocity2=bbac ? bbdv.velocity : 0.0;
          velocity=(8*des_velocity1)/(8+des_velocity1);  // magische Formel, um den Schlupf zu beruecksichtigen

          // Teil 5: Bewegung simulieren
          // z. Z. noch vereinfacht mit kinematischem Modell + Stoerung
          Timestamp now;
          Angle old_heading = heading;
          double difftime = 1e-3*now.diff_usec(timestamp);
          kin.euler_step (position, heading, velocity, steering_angle, difftime);
          double diffangle = heading.get_rad()-old_heading.get_rad();
          if (diffangle>=2*M_PI)
            diffangle-=2*M_PI;
          else if (diffangle<=-2*M_PI)
            diffangle+=2*M_PI;
          timestamp=now;

          // Teil 6: Position und Odometrie ins Blackboard schreiben
          vsens.update (position, heading, now);
          Pose vp;
          Vec velvec;
          vsens.get(vp.position, vp.orientation, velvec, vp.yawrate, now);
          vp.velocity=velvec.length()*(velvec.angle().in_between(vp.orientation-Angle::quarter, vp.orientation+Angle::quarter) ? +1 : -1);
          vp.stddev=100;
          // vp.position=position;
          // vp.orientation=heading;
          // vp.velocity=velocity;
          // vp.yawrate=diffangle/difftime*1e3;
          // vp.timestamp=now;
          BBOARD->setVehiclePose (vp);
          Odometry odo;
          odo.velocity=des_velocity1+0.05*normal_rnd();
          odo.steer=des_steering_angle1;
          BBOARD->setOdometry (odo);

          // Teil 7: Warten
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
          boost::this_thread::interruption_point();
        }
      } catch(boost::thread_interrupted&){;}
    }
    void deinit () {;}
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::AnicarSimulation> anicar_simulation ("AnicarSimulation");

} // namespace
