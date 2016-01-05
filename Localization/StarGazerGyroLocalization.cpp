
#include "StarGazerProxy.h"
#include "SLVelocitySensor.h"
#include "IMUPlausibilityChecker.h"
#include "IMU/CCHR6dm.h"
#include "HeadingFilter.h"
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Vehicle/VehicleKinematics.h"
#include <cmath>
#include <sstream>
#include <boost/bind.hpp>

using namespace std;
using namespace DerWeg;

namespace {
  inline double sq (double x) { return x*x; }
}
#define DOUT(x) ;

namespace DerWeg {

  class StarGazerGyroLocalization : public KogmoThread {
  private:
    boost::mutex sgmutex;            ///< Mutex, da intern zwei Threads laufen. Schuetzt Zugriff auf Attribute

    CHR6dm::CCHR6dm* imu;            ///< die IMU
    HeadingFilter filter;            ///< KF fuer Orientierung
    StarGazerProxy starGazer;        ///< Zugriff auf StarGazer
    IMUPlausibilityChecker imu_checker;  ///< Plausibilitaetspruefung fuer IMU-Werte
    Timestamp timestamp_motion;      /// zur Plausibilisierung der letzte Zeitpunkt mit Bewegung

    Vec latestPosition;              ///< die letzte berechnete Position, bezieht sich auf latestUpdateOdometry
    double stddev_position;          ///< eine Art Standardabweichung fuer die (gefilterte) Position
    Odometry latestOdometry;         ///< die letzte Odometriegeschwindigkeit

    VehicleKinematics kin;           ///< das Kinematikmodell
    SLVelocitySensor velfilter;      ///< der Geschwindigkeitsfilter
    unsigned int hamming_max;        ///< maximaler Hammingabstand der Landmarken-IDs, der noch okay ist

    void writeToBlackboard ();       ///< aktuelle Pose ins Blackboard schreiben (ohne sgmutex zu locken!)
    void imu_callback (const CHR6dm::SSensorData data);   ///< Dead Reckoning auf IMU und kinetischem Modell
    void executeSGLocalization ();   ///< ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
    void predict (Timestamp);        ///< praediziere bis zu einem bestimmten Zeitpunkt (lockt nicht sgmutex)

    void debug (char c, double d=0);
  public:
    StarGazerGyroLocalization ();
    ~StarGazerGyroLocalization ();
    void execute ();
    void init (const ConfigReader&);
  };

}



StarGazerGyroLocalization::StarGazerGyroLocalization () :
  stddev_position (20000),
  kin (0,527),
  velfilter (0.5,1)
{
  imu = NULL;
  hamming_max = 0;
}

StarGazerGyroLocalization::~StarGazerGyroLocalization () {
  if (imu) delete imu;
}

void StarGazerGyroLocalization::init (const ConfigReader& cfg) {
  starGazer.init (cfg);
  std::string imu_device = "/dev/ttyUSB1";
  cfg.get ("IMU::device", imu_device);
  unsigned int imu_frequency = 300;
  cfg.get ("IMU::frequency", imu_frequency);
  cfg.get ("StarGazer::id_fault_tolerance", hamming_max);
  CHR6dm::SChannelData chd (false, false, false, false, false, false, false, false, false, true, true, true, true, true, true);
  imu = new CHR6dm::CCHR6dm;
  imu->setCallback (boost::bind( &DerWeg::StarGazerGyroLocalization::imu_callback, this, _1 ));
  imu->open (imu_device);
  imu->sendSetActiveChannels (chd);
  imu->sendSetBroadcastMode (imu_frequency);
}

void StarGazerGyroLocalization::predict (Timestamp t) {
  // Praediktion von Position (mit kinematischem Modell) und Orientierung (mit KF)
  // ACHTUNG: lockt nicht sgmutex
  double difftime = 1e-3*t.diff_usec(Timestamp(filter.get_time ()));
  if (difftime<0) return;
  Angle orientation = Angle::rad_angle(filter.get_yaw_angle());
  double steer_deg = latestOdometry.steer.get_deg_180();
  Angle steer_fake = Angle::deg_angle((1.0-0.1*latestOdometry.velocity*latestOdometry.velocity)*steer_deg); // Heuristik: um ein Bisschen seitliches Rutschen zu modellieren
  double velocity_fake = /*(1.0-0.1*latestOdometry.velocity*latestOdometry.velocity)* */latestOdometry.velocity; // Heuristik: um ein Bisschen longitudinalen Schlupf zu modellieren
  kin.euler_step (latestPosition, orientation, velocity_fake, steer_fake, difftime);
  stddev_position=sqrt(stddev_position*stddev_position+400*difftime);
  filter.predict (t);
}

void StarGazerGyroLocalization::writeToBlackboard () {
  // die Pose ins Blackboard schreiben
  // entnimmt Pose von latestPosition, HeadingFilter
  // entnimmt Geschwindigkeiten dem VelocityFilter
  // ACHTUNG: lockt nicht sgmutex
  Pose vp;
  vp.position=latestPosition;
  vp.orientation=Angle::rad_angle(filter.get_yaw_angle());
  vp.timestamp=filter.get_time();
  vp.stddev=stddev_position;

  Vec filterpos;
  Angle filterorientation;
  Timestamp filtertime;
  Vec filtervel;
  velfilter.get (filterpos, filterorientation, filtervel, vp.yawrate, filtertime);

  vp.velocity = filtervel*Vec::unit_vector (vp.orientation);
  BBOARD->setVehiclePose (vp);
}

void StarGazerGyroLocalization::imu_callback (const CHR6dm::SSensorData data) {
  // eine Callback-Funktion, um die Odometrie zyklisch zu holen und Dead Reckoning auszufuehren
  Timestamp now;
  latestOdometry = BBOARD->getOdometry ();
  imu_checker.add (data);
  bool is_active = BBOARD->getActive();
  bool is_dvel = (std::abs(BBOARD->getDesiredVelocity().velocity)>1e-5);
  bool is_imu_driving = imu_checker.is_driving();
  bool is_imu_nomotion = imu_checker.is_nomotion();
  if ((is_active && is_dvel) || !is_imu_nomotion) {
    timestamp_motion.update();
  }
  if (is_imu_nomotion && (timestamp_motion.elapsed_msec()>1000)) {
    boost::unique_lock<boost::mutex> lock (sgmutex);
    predict (now);
    filter.observe_gyro_nomotion (data.gz*M_PI/180);
    writeToBlackboard ();
//    debug('N');
  } else if (is_imu_driving && is_active) { // TODO: ueberlegen, ob is_active hier sinnvoll ist
    boost::unique_lock<boost::mutex> lock (sgmutex);
    predict (now);
    filter.observe_gyro (data.gz*M_PI/180);
    writeToBlackboard ();
//    debug('G');
  } else {
    // unplausible Beobachtung
  }
}

void StarGazerGyroLocalization::debug (char c, double d) {
  std::stringstream inout;
  inout << c << " " << d;
  BBOARD->addMessage(inout.str());
//  EOUT (inout.str() << std::endl);
}
  

void StarGazerGyroLocalization::executeSGLocalization () {
  // ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
  try{
    std::vector<StarGazerPose> sg_poses;
    while (true) {
      sg_poses = starGazer.getAllPoses();
      if (sg_poses.size()==0) continue;
      boost::unique_lock<boost::mutex> lock (sgmutex);
      predict (sg_poses[0].time);
      
      // suche unter allen moeglichen Posen die mit der geringsten Hammingdistanz (und darunter, falls mehrdeutig) diejenige mit dem kleinsten Abstand zur erwarteten Pose 
      unsigned int best_hamming = sg_poses[0].hamming_id+1;
      unsigned int best_index = 0;
      double best_distance = 0;
      for (unsigned int i=0; i<sg_poses.size(); ++i) {
        if (sg_poses[i].hamming_id<best_hamming) {
          best_hamming = sg_poses[i].hamming_id;
          best_index = i;
          best_distance = (latestPosition-sg_poses[i].position_global).length();
        } else if (sg_poses[i].hamming_id==best_hamming && (latestPosition-sg_poses[i].position_global).length()<best_distance) {
          best_index = i;
          best_distance = (latestPosition-sg_poses[i].position_global).length();
        }
      }
      StarGazerPose& sg_pose (sg_poses[best_index]);
      
      if (sg_pose.valid_measurement && sg_pose.hamming_id<=hamming_max) {
        if (sg_pose.hamming_id>0) {
//          debug('H', sg_pose.hamming_id);
        }
        predict (sg_pose.time);
        if ((latestPosition-sg_pose.position_global).length()>3*stddev_position) {
          EOUT("ignoring StarGazer measurement from landmark " << sg_pose.id << '\n');
//          debug('F', sg_pose.id);
        } else {
          latestPosition = (sq(sg_pose.stddev_position)*latestPosition+sq(stddev_position)*sg_pose.position_global)/(sq(stddev_position)+sq(sg_pose.stddev_position));
          stddev_position = sqrt (sq(stddev_position)*sq(sg_pose.stddev_position)/(sq(stddev_position)+sq(sg_pose.stddev_position)));   // Kalman-Filter-artiger update
          filter.observe_yaw_angle (sg_pose.orientation_global.get_rad(), sg_pose.stddev_orientation);

          velfilter.update (latestPosition, Angle::rad_angle(filter.get_yaw_angle()), sg_pose.time);
          writeToBlackboard();
//          debug('L', sg_pose.id);
        }
      } else {
        if (!sg_pose.valid_measurement) {
//          debug('D');
        } else {
//          debug('U', sg_pose.id);
        }
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}

void StarGazerGyroLocalization::execute () {
  // ein Thread, um StarGazer-Beobachtungen zu holen und einzubinden
  executeSGLocalization ();
}



namespace {
  // Anmelden bei der Plugin-Factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StarGazerGyroLocalization> globalposition_stargazer ("StarGazerGyro");
}
