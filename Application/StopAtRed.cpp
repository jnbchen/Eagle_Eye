
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../ImageProcessing/TrafficSignDetector.h"

namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die bei einem groesserem roten Objekt im Bild anhaelt */
  class StopAtRed : public KogmoThread {
  public:
    StopAtRed () {;}
    ~StopAtRed () {;}
    void execute () {
      try{
        ImageBuffer ib;
        while (true) {
          ib = BBOARD->getImage();
          if (!ib.image.empty()) {
            unsigned int num_red_pixels=0;
            for (int v=0; v<ib.image.rows; ++v) {
              for (int u=0; u<ib.image.cols; ++u) {
                cv::Vec3b bgr = ib.image.at<cv::Vec3b>(v,u);
                if (bgr[2]>32 && 2*static_cast<unsigned int>(bgr[1])<static_cast<unsigned int>(bgr[2]) && 2*static_cast<unsigned int>(bgr[0])<static_cast<unsigned int>(bgr[2])) {
                  num_red_pixels++;
                }
              }
            }
            Velocity dv;
            dv.steer=Angle::deg_angle(0);
            unsigned int num_pixels = ib.image.rows*ib.image.cols;
            if (num_red_pixels*20>num_pixels) {
              dv.velocity=0;
              EOUT("S " << num_red_pixels << ' ' << num_pixels << std::endl);
            } else {
              dv.velocity=0.5;
              EOUT("  " << num_red_pixels << ' ' << num_pixels << std::endl);
            }
            BBOARD->setDesiredVelocity(dv);
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

  /** eine sehr einfache Beispiel- und Testanwendung, die bei einem Einfahrt-Verboten-Schild anhält */
  class StopAtNoEntry : public KogmoThread {
    TrafficSignDetector detector;
  public:
    StopAtNoEntry () {;}
    ~StopAtNoEntry () {;}
    void execute () {
      try{
        ImageBuffer ib;
        Velocity dv;
        dv.steer = Angle::deg_angle(0);
        while (true) {
          ib = BBOARD->getImage();
          if (ib.image.empty()) {
            dv.velocity=0;
          } else {
            dv.velocity=0.5;
            std::vector<DerWeg::TrafficSign> signs_detected = detector.process (ib.image);
            for (unsigned int i=0; i<signs_detected.size(); ++i) {
              if (signs_detected[i].type==DerWeg::NO_ENTRY) {
                EOUT (" --- ");
                dv.velocity=0;
              } else if (signs_detected[i].type==DerWeg::GO_LEFT) {
                EOUT (" <-- ");
              } else if (signs_detected[i].type==DerWeg::GO_RIGHT) {
                EOUT (" --> ");
              } else {
                EOUT ("  ^  ");
              }
            }
            EOUT ("\n");
          }
          BBOARD->setDesiredVelocity(dv);

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StopAtRed> application1 ("StopAtRed");
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StopAtNoEntry> application2 ("StopAtNoEntry");

}
