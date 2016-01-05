#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {
  /** A very simple example and testing application that stops
      when facing a large red object. */
  class StopAtRed : public KogmoThread {
  public:
    StopAtRed () {;}
    ~StopAtRed () {;}
    void execute () {
      try{
        while (true) {
          ImageBuffer ib = BBOARD->getImage();
          if (!ib.image.empty()) {
            unsigned int num_red_pixels=0;
            for (int v=0; v<ib.image.rows; ++v) {
              for (int u=0; u<ib.image.cols; ++u) {
                cv::Vec3b bgr = ib.image.at<cv::Vec3b>(v,u);
                if (bgr[2]>32 && 2*static_cast<unsigned int>(bgr[1])<
                    static_cast<unsigned int>(bgr[2]) && 
                    2*static_cast<unsigned int>(bgr[0])<
                    static_cast<unsigned int>(bgr[2])) {
                  num_red_pixels++;
                }
              }
            }
            Velocity dv;
            dv.steer=Angle::deg_angle(0);
            unsigned int num_pixels = ib.image.rows*ib.image.cols;
            if (num_red_pixels*20>num_pixels) dv.velocity=0;
            else dv.velocity=0.5;
            BBOARD->setDesiredVelocity(dv);
          }
          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };
} // namespace DerWeg

namespace {
  // Register Plugin in the factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, 
     DerWeg::StopAtRed> application ("StopAtRed");
}