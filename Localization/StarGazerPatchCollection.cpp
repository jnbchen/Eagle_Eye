
#include "../Elementary/KogmoThread.h"
#include "StarGazerProxy.h"
#include "StarGazerLandmarkList.h"
#include "StarGazerCalibration.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <fstream>
#include <set>

using namespace std;
using namespace DerWeg;

namespace DerWeg {

  /** collects StarGazer measurements and registers the landmarks */
  class StarGazerPatchCollection : public KogmoThread {
    StarGazerProxy starGazer;
    RobotPositionList positions;
    MeasurementList measurements;
  public:
    void execute ();
    void init (const ConfigReader& cfg);
    void deinit ();
  };

} // namespace DerWeg




void StarGazerPatchCollection::execute () {
  try{
    StarGazerPose sg_pose;
    bool was_active=true;
    int pid=-1;
    while (true) {
      sg_pose = starGazer.getPose ();
      bool is_active = BBOARD->getActive ();
      if (!is_active)
        EOUT('\n');
      if (!is_active) {
        if (was_active) {
          positions.push_back (StarGazerRobotPosition (++pid,Vec::zero_vector, Angle::zero));
          EOUT("NewPos-" << pid << ' ');
        }
        if (sg_pose.valid_measurement) {
          int li=starGazer.landmarks.findLandmark (sg_pose.id);
          if (li>=0) {
            measurements.push_back (StarGazerCalibrationMeasurement (sg_pose.position_relative, sg_pose.orientation_relative, sg_pose.id, pid));
            EOUT("M-" << sg_pose.id << "-" << pid << ' ');
          } else {
            EOUT("UnknownLandmark-" << sg_pose.id << ' ');
          }
        } else {
          EOUT("Dead ");
        }
      } else {
        if (!was_active) {
          LOUT("Summary P" << pid << ":");
          set<unsigned int> lids;
          for (unsigned int im=0; im<measurements.size(); ++im) {
            if (static_cast<int>(measurements[im].position_id)==pid) {
              lids.insert (measurements[im].landmark_id);
            }
          }
          for (set<unsigned int>::iterator it=lids.begin(); it!=lids.end(); it++) {
            LOUT(" L" << (*it));
          }
          LOUT('\n');
        }
        if (sg_pose.valid_measurement) {
          EOUT("L-" << sg_pose.id << ' ');
        } else {
          EOUT("Dead ");
        }
      }
      was_active=is_active;

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}

void StarGazerPatchCollection::init (const ConfigReader& cfg) {
  starGazer.init (cfg);
}

void StarGazerPatchCollection::deinit () {
  EOUT("CalibrateStarGazer ");
  StarGazerCalibration cal;
  cal.addAndMerge (starGazer.landmarks, positions, measurements);
  cal.calibrateLandmarks ();
  EOUT("Calibration error(angle,pos)=" << cal.totalAngleError(1.0) << ", " << cal.totalPositionError(1.0) << std::endl);
  EOUT("Use offline calibration tool to improve registration.\n");
  ofstream rf ("star_gazer_calibration.cfg");
  EOUT("SaveStarGazerCalibration ");
  cal.getLandmarks().writeLandmarks (rf);
  cal.getPositions().writePositions (rf);
  cal.getMeasurements().writeMeasurements (rf);
  rf << flush;
}



namespace {
  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StarGazerPatchCollection> globalposition_stargazer ("StarGazerPatchCollection");
} // namespace
