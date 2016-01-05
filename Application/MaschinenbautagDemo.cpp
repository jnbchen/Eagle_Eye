
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../ImageProcessing/TrafficSignDetector.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <algorithm>

namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die bei einem groesserem roten Objekt im Bild anhaelt */
  class MaschinenbautagDemo : public KogmoThread {
    TrafficSignDetector detector;
    std::string windowname1, windowname2;
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    std::string windownameRect, windownameConf, windownameVisu;
    cv::Mat depth, conf, rect;
  public:
    MaschinenbautagDemo () :
      windowname1 ("Camera Image 1"),
      stereoGPU ("/home/common/calib.txt"),
      windownameVisu ("stereo") {
      cvNamedWindow (windowname1.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameVisu.c_str(), CV_WINDOW_AUTOSIZE);
    }
    ~MaschinenbautagDemo () {
      cvDestroyWindow (windowname1.c_str());
      cvDestroyWindow (windownameVisu.c_str());
    }
    void paint_box (cv::Mat& image, const TrafficSign& sign) {
      cv::Scalar green (50, 100, 0);
      int v1=sign.center_y-sign.height/2;
      int v2=sign.center_y+sign.height/2;
      int u1=sign.center_x-sign.height/2;
      int u2=sign.center_x+sign.height/2;
      cv::rectangle (image, cv::Point(u1,v1), cv::Point(u2,v2), green, 3);
      std::string text = "";
      switch (sign.type) {
        case NO_ENTRY : text = "no-entry"; break;
        case GO_LEFT : text = "go-left"; break;
        case GO_RIGHT : text = "go-right"; break;
        case GO_AHEAD : text = "go-ahead"; break;
      }
      cv::putText (image, text, cv::Point (u1, v1-5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, green, 2);
    }
    void execute () {
      try{
        Velocity dv;
        dv.steer = Angle::deg_angle(0);
        while (true) {
          BBOARD->waitForImage();   // wait for next image
          ib=BBOARD->getImage();    // get next image pair from blackboard
          
          stereoGPU.runStereoMatching (ib.image, ib.image_right); // starte die Tiefenberechnung
          std::vector<DerWeg::TrafficSign> signs_detected;
          if (!ib.image.empty()) {
            signs_detected = detector.process (ib.image);
          }
          stereoGPU.getRectifiedLeftImage(rect);
          stereoGPU.getStereoResults (depth,conf); // warte, bis die Tiefenberechnung abgeschlossen ist und hole die Ergebnisse ab (in depth, conf, disp, rect)
          cv::imshow (windownameVisu.c_str(),depth/10.f);  // zeige Tiefenbild an

          if (ib.image.empty()) {
            EOUT ("Empty-image!\n");
            dv.velocity=0;
          } else {
            dv.velocity=0.3;
            for (unsigned int i=0; i<signs_detected.size(); ++i) {
              if (signs_detected[i].type==DerWeg::NO_ENTRY) {
                EOUT (" --- ");
                dv.velocity=0;
                paint_box (ib.image, signs_detected[i]);
              } else if (signs_detected[i].type==DerWeg::GO_LEFT) {
                EOUT (" <-- ");
                dv.steer = Angle::deg_angle(30);
                paint_box (ib.image, signs_detected[i]);
              } else if (signs_detected[i].type==DerWeg::GO_RIGHT) {
                EOUT (" --> ");
                dv.steer = Angle::deg_angle(-30);
                paint_box (ib.image, signs_detected[i]);
              } else {
                EOUT ("  ^  ");
                dv.steer = Angle::deg_angle(0);
                paint_box (ib.image, signs_detected[i]);
              }
            }
            EOUT ("\n");
          }
          int obstacles = 0;
          for (int u=0; u<depth.cols; ++u) {
            if (conf.at<unsigned char>(300,u)>30 && depth.at<float>(300,u)<1.0) {
              obstacles++;
            }
          }
          if (obstacles > 20) {
            EOUT ("Obstacles!\n");
            dv.velocity=0;
          }
          
          
          BBOARD->setDesiredVelocity(dv);
          if (!ib.is_empty()) {     // display left image, if available
            cv::imshow (windowname1.c_str(), ib.image);
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::MaschinenbautagDemo> application1 ("MaschinenbautagDemo");

}
