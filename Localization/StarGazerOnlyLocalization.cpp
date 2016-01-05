
#include "StarGazerProxy.h"
#include "SLVelocitySensor.h"
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Vehicle/VehicleKinematics.h"
#include <cmath>
#include <fstream>
#include <boost/bind.hpp>

using namespace std;
using namespace DerWeg;

namespace {
  inline double sq (double x) { return x*x; }
}
#define DOUT(x) ;

namespace DerWeg {

  class StarGazerOnlyLocalization : public KogmoThread {
  private:
    StarGazerProxy starGazer;        ///< Zugriff auf StarGazer
    boost::mutex sgmutex;            ///< Mutex, da intern zwei Threads laufen. Schuetzt Zugriff auf Attribute
    Timestamp latestUpdateOdometry;  ///< Zeitpunkt des letzten Updates durch Dead Reckoning
    Timestamp latestUpdateStarGazer; ///< Zeitpunkt des letzten Updates durch StarGazer
    Vec latestPosition;              ///< die letzte berechnete Position, bezieht sich auf latestUpdateOdometry
    Angle latestOrientation;
    Odometry latestOdometry;         ///< die letzte Odometriegeschwindigkeit

    double stddev_position;          ///< eine Art Standardabweichung fuer die (gefilterte) Position
    double stddev_orientation;       ///< eine Art Standardabweichung fuer die (gefilterte) Orientierung

    VehicleKinematics kin;           ///< das Kinematikmodell
    SLVelocitySensor velfilter;      ///< der Geschwindigkeitsfilter

    void updateDeadReckoning (Timestamp now, Odometry odo); ///< einen Schritt Dead Reckoning durchfuehren (ohne sgmutex zu locken!)
    void writeToBlackboard ();       ///< aktuelle Pose ins Blackboard schreiben (ohne sgmutex zu locken!)
    void executeDeadReckoning ();    ///< ein Thread, um die Odometre zyklisch zu holen und Dead Reckoning auszufuehren
    void executeSGLocalization ();   ///< ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
  public:
    StarGazerOnlyLocalization ();
    void execute ();
    void init (const ConfigReader&);
  };

}


void StarGazerOnlyLocalization::updateDeadReckoning (Timestamp now, Odometry odo) {
  // einen Heunschritt durchfuehren
  // latestPosition, latestOrientation, latestUpdateOdometry, deviation aktualisieren
  // so eine Art Praediktionsschritt des Filters
  // ACHTUNG: lockt nicht sgmutex
  Angle latestOrientation2 = latestOrientation;
  double difftime = 1e-3*now.diff_usec(latestUpdateOdometry);
  Angle driftAngle = Angle::rad_angle(0.8*odo.steer.get_rad_pi());
  kin.heun_step (latestPosition, latestOrientation, latestOdometry.velocity, odo.velocity, latestOdometry.steer, driftAngle, difftime);
  latestOdometry=odo;
  double diffangle = latestOrientation.get_rad()-latestOrientation2.get_rad();
  if (diffangle>=2*M_PI)
    diffangle-=2*M_PI;
  else if (diffangle<=-2*M_PI)
    diffangle+=2*M_PI;
  latestUpdateOdometry=now;
  stddev_position+=0.4*difftime;
  stddev_orientation+=1.309e-4*difftime;
}

void StarGazerOnlyLocalization::writeToBlackboard () {
  // die Pose ins Blackboard schreiben
  // entnimmt Pose von latestPosition, latestOrientation
  // entnimmt Geschwindigkeiten dem VelocityFilter
  // ACHTUNG: lockt nicht sgmutex
  Pose vp;
  vp.position=latestPosition;
  vp.orientation=latestOrientation;
  vp.timestamp=latestUpdateOdometry;
  vp.stddev=stddev_position;

  Vec filterpos;
  Angle filterorientation;
  Timestamp filtertime;
  Vec filtervel;
  velfilter.get (filterpos, filterorientation, filtervel, vp.yawrate, filtertime);

  vp.velocity = filtervel*Vec::unit_vector (vp.orientation);
  DOUT (vp.position.x << ' ' << vp.position.y << ' ' << vp.orientation.get_deg());
  BBOARD->setVehiclePose (vp);
}

void StarGazerOnlyLocalization::executeDeadReckoning () {
  // ein Thread, um die Odometre zyklisch zu holen und Dead Reckoning auszufuehren
  try{
    while (true) {
      sgmutex.lock();
      Odometry odo = BBOARD->getOdometry ();
      Timestamp now;
      updateDeadReckoning (now, odo);
      DOUT (now.get_msec() << " HEUN ");
      writeToBlackboard ();
      DOUT ("\n");
      sgmutex.unlock();

      boost::this_thread::sleep(boost::posix_time::milliseconds(20));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}


void StarGazerOnlyLocalization::executeSGLocalization () {
  // ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
  try{
    StarGazerPose sg_pose;
    Pose vp;
    while (true) {
      sg_pose = starGazer.getPose();
      vp.timestamp =sg_pose.time;
      vp.position = sg_pose.position_global;
      vp.orientation = sg_pose.orientation_global;
      DOUT (vp.timestamp.get_msec() << " STARGAZER " << sg_pose.id << " " << sg_pose.position_relative.x << " " << sg_pose.position_relative.y << " " << sg_pose.orientation_relative.get_deg() << "\n");
      if (sg_pose.valid_measurement && sg_pose.landmark_exists) {
        DOUT (vp.timestamp.get_msec() << " SG-WORLD " << sg_pose.id << " " << vp.position.x << " " << vp.position.y << " " << vp.orientation.get_deg() << "\n");
        boost::unique_lock<boost::mutex> lock (sgmutex);
        updateDeadReckoning (vp.timestamp, BBOARD->getOdometry ());
        if ((latestPosition-vp.position).length()>2*stddev_position) {
          EOUT("ignoring StarGazer measurement from landmark " << sg_pose.id << '\n');
        } else {
          // EOUT("observing landmark " << pd.id << std::endl);
          latestPosition = (sq(sg_pose.stddev_position)*latestPosition+sq(stddev_position)*vp.position)/(sq(stddev_position)+sq(sg_pose.stddev_position));
          latestOrientation = ((sq(sg_pose.stddev_orientation)*Vec::unit_vector(latestOrientation)+sq(stddev_orientation)*Vec::unit_vector(vp.orientation))/(sq(stddev_orientation)+sq(sg_pose.stddev_orientation))).angle();
          stddev_position = sqrt (sq(stddev_position)*sq(sg_pose.stddev_position)/(sq(stddev_position)+sq(sg_pose.stddev_position)));   // Kalman-Filter-artiger update
          stddev_orientation = sqrt (sq(stddev_orientation)*sq(sg_pose.stddev_orientation)/(sq(stddev_orientation)+sq(sg_pose.stddev_orientation)));   // Kalman-Filter-artiger update
          latestUpdateStarGazer=vp.timestamp;

          velfilter.update (latestPosition, latestOrientation, latestUpdateStarGazer);
          DOUT (vp.timestamp.get_msec() << " FUSION ");
          writeToBlackboard();
          DOUT ("\n");
        }
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}

StarGazerOnlyLocalization::StarGazerOnlyLocalization () :
  stddev_position (2000),
  stddev_orientation (15*M_PI/180),
  kin (0,527),
  velfilter (0.5,1)
{;}

void StarGazerOnlyLocalization::init (const ConfigReader& cfg) {
  starGazer.init (cfg);
}

void StarGazerOnlyLocalization::execute () {
  // ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
  boost::thread* odothread = new boost::thread(boost::bind(&DerWeg::StarGazerOnlyLocalization::executeDeadReckoning, this));
  executeSGLocalization ();
  if (odothread) {
    odothread->interrupt();
    odothread->join();
    delete odothread;
    odothread=NULL;
  }
}



namespace {
  // Anmelden bei der Plugin-Factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StarGazerOnlyLocalization> globalposition_stargazer ("StarGazerOnly");
}
