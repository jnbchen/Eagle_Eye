
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "./LandmarkFinder.h"
#include "./Localizer.h"
#include <fstream>
#include <calibio/CalibIO.hpp>
//#include "./PLModel.h"
//#include "./Undistortion.h"

#define DOUT(x);

namespace DerWeg {

  class CameraLocalization : public KogmoThread {
    LandmarkFinder* oFinder;
    Localizer* oLocalizer;
    CalibIO oCalib;
    Vec position;
    Angle heading;
    double velocity;
    Timestamp timestamp;
//    Entzerrung::PLModel plm;


  public:
    CameraLocalization() : velocity(0)//, plm(602, 435)
    {

    }
    ~CameraLocalization() {;}
    void init (const ConfigReader& cfg) {
        if(!oCalib.readCalibFromFile("/home/common/calib_top.txt"))
        {
            throw std::invalid_argument("/home/common/calib_top.txt not found");
        }
        oLocalizer = new Localizer (cfg, oCalib);
        oFinder = new LandmarkFinder(cfg, oCalib, "/home/common/darkframe.png");
//      std::ifstream ms("./model.plm");
//      plm.readParametersFromStream(ms);
//      std::vector<double> x;
//      cfg.get ("DeadReckoning::init", x);
//      if (x.size()>=2) {
//        position = Vec (x[0], x[1]);
//        if (x.size()>=3) {
//          heading = Angle::deg_angle (x[2]);
//        }
//      }
    }
    void execute () {
      try{
        while (true) {
          Pose vp;
          //LOUT("Loc: getting image" << std::endl);
          BBOARD->waitForTopImage();
          ImageBuffer Image = BBOARD->getTopImage();
          if(Image.is_empty())
          {
              continue;
          }
          cv::Mat ImageCV = Image.image;
          //cv::imwrite("./InCamLoc.png", ImageCV);
          //    cv::Mat Image3C;
//            ImageCV.assignTo(Image3C, CV_8UC3);
//
//
//
//          LOUT("Loc: undistorting image" << std::endl);
//          cv::Mat_<cv::Vec3b> outputImage(Image3C.rows, Image3C.cols);
//          Entzerrung::Undistortion undist(plm, Image3C.cols, Image3C.rows);
//          undist.undistort(outputImage, Image3C);

//            cv::namedWindow("Peter");
//            cv::imshow("Peter", ImageCV);
          oFinder->SetImage(ImageCV);

          //LOUT("Loc: looking for landmarks" << std::endl);
          std::vector<Landmark> oLandmarks;
          oFinder->FindLandmarks(oLandmarks);
          LOUT("Found landmakrs: " << oLandmarks.size() << std::endl);
          cv::waitKey(1);
          //LOUT("Loc: updating kf" << std::endl);
          //oLocalizer->UpdateKFFromLandmarks(oLandmarks);
          float fDeltaT = 0.1;
          oLocalizer->FindPosition(oLandmarks, fDeltaT);
          cv::Mat oState = oLocalizer->GetState();

          vp.position=Vec(oState.at<float>(0,0), oState.at<float>(1,0));
          Angle Orientation;
          Orientation.set_rad(oState.at<float>(2,0));
          vp.orientation=Orientation;
          vp.velocity=oState.at<float>(3,0);
          vp.yawrate=oState.at<float>(4,0);

//
//          Odometry odo = BBOARD->getOdometry ();
//          Timestamp now;
//          Angle old_heading = heading;
//          double difftime = 1e-3*now.diff_usec(timestamp);
//          velocity=odo.velocity;
//          double diffangle = heading.get_rad()-old_heading.get_rad();
//          if (diffangle>=2*M_PI)
//            diffangle-=2*M_PI;
//          else if (diffangle<=-2*M_PI)
//            diffangle+=2*M_PI;
//          timestamp=now;
//          vp.position=position;
//          vp.orientation=heading;
//          vp.velocity=velocity;
//          vp.yawrate=diffangle/difftime*1e3;

          //LOUT( "Position " << vp.position.x << ' ' << vp.position.y << ' ' << vp.orientation.get_deg() << std::endl);
          Timestamp now;
          vp.timestamp=now;
          BBOARD->setVehiclePose (vp);
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
    void deinit () {
      delete oLocalizer;
      delete oFinder;
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::CameraLocalization> globalposition_cam ("CameraLocalization");

} // namespace
