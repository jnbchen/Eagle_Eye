
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../tools/CameraLocalizationMapping/CameraLocalizationMapping.h"
#include "LandmarkFinder.h"
#include <stdexcept>
#include <fstream>
#include <vector>
#include <map>
#include <calibio/CalibIO.hpp>

using namespace std;
using namespace DerWeg;

typedef ::Landmark DLandmark;

namespace DerWeg {

  /** collects measurements of landmarks */
  class CameraLocalizationPatchCollection : public KogmoThread {
    LandmarkList landmarks;
    RobotPositionList positions;
    MeasurementList measurements;
    LandmarkFinder* lm_finder;

    double fx, fy;   // focal length of camera
    double ppx, ppy; // principal point of camera
    map<unsigned int, unsigned int> landmarks_found;
  public:
    void execute ();
    void init (const ConfigReader& cfg);
    void deinit ();
  };

} // namespace DerWeg


void CameraLocalizationPatchCollection::execute () {
  try{
    bool was_active=true;
    Timestamp starttime_inactive;
    int pid=-1;
    while (true) {
      bool is_active = BBOARD->getActive ();
      if (!is_active && was_active) {
        starttime_inactive.update ();
        ++pid;
        positions.push_back (RobotPosition (pid,Vec::zero_vector, Angle::zero));
        EOUT("NewPos-" << pid << '\n');
        landmarks_found.clear ();
      }
      if (!is_active && starttime_inactive.elapsed_msec()>1000) {
        cv::Mat image = BBOARD->getTopImage ().image;
        lm_finder->SetImage (image);
        std::vector<DLandmark> detections;
        if (lm_finder->FindLandmarks (detections)==0) {
          for (unsigned int i=0; i<detections.size(); ++i) {
            int il = landmarks.findLandmark (detections[i].nID);
            if (il>=0) {
              map<unsigned int, unsigned int>::iterator it = landmarks_found.find (detections[i].nID);
              if (it==landmarks_found.end()) {
                landmarks_found[detections[i].nID] = 1;
                EOUT ("NewLandmark-" << detections[i].nID << '\n');
              } else {
                ++(landmarks_found[detections[i].nID]);
              }
              CameraLocalizationMeasurement m;
              m.landmark_id = detections[i].nID;
              m.position_id = pid;
//              m.position = Vec ((detections[i].oPosition.x-ppx)/fx, (detections[i].oPosition.y-ppy)/fy)*landmarks[il].height;  /// TODO: evtl. noch negieren!
              m.position = Vec (-(detections[i].oPosition.y-ppy)/fy, (detections[i].oPosition.x-ppx)/fx)*landmarks[il].height;  /// TODO: evtl. noch negieren!
              measurements.push_back (m);
            } else {  // landmark not in landmark list
              EOUT("NonexistingLandmark-" << detections[i].nID << '\n');
            }
          }
        }
      }
      was_active=is_active;

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}

void CameraLocalizationPatchCollection::init (const ConfigReader& cfg) {
  landmarks.readLandmarks (cfg);
  std::vector<double> focal_length;
  std::vector<double> principle_point;
  cfg.get ("CameraLocalization::focal_length", focal_length);
  cfg.get ("CameraLocalization::principle_point", principle_point);
  if (focal_length.size()!=2) throw std::runtime_error ("Did not find valid entry 'CameraLocalization::focal_length' in configuration");
  if (principle_point.size()!=2) throw std::runtime_error ("Did not find valid entry 'CameraLocalization::principle_point' in configuration");

    CalibIO oCalib;
    if(!oCalib.readCalibFromFile("/home/common/calib_top.txt"))
    {
        throw std::invalid_argument("/home/common/calib_top.txt not found");
    }
    lm_finder = new LandmarkFinder(cfg, oCalib, "/home/common/darkframe.png");


  fx = focal_length [0];
  ppx = principle_point [0];
  fy = focal_length [1];
  ppy = principle_point [1];
}

void CameraLocalizationPatchCollection::deinit () {
  ofstream rf ("landmark_collection.cfg");
  EOUT("save collected landmarks and measurements\n");
  landmarks.writeLandmarks (rf);
  positions.writePositions (rf);
  measurements.writeMeasurements (rf);
  rf << flush;
}



namespace {
  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::CameraLocalizationPatchCollection> builder ("CameraLocalizationPatchCollection");
} // namespace
