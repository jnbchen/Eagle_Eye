
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die lediglich die Kamerabilder im 500ms-Rythmus anzeigt */
  class ShowCameraImage : public KogmoThread {
    std::string windowname;
    std::string windowname2;
    ImageBuffer ib;
  public:
    ShowCameraImage () : windowname ("Camera Image"), windowname2 ("Camera Image Right") {
      cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windowname2.c_str(), CV_WINDOW_AUTOSIZE);
    }
    ~ShowCameraImage () {
      cvDestroyWindow (windowname.c_str());
      cvDestroyWindow (windowname2.c_str());
    }
    void execute () {
      try{
        while (true) {
          BBOARD->waitForImage();
          ib=BBOARD->getImage();
          if (!ib.image.empty()) {
            cv::imshow (windowname.c_str(), ib.image);
          }
          if (!ib.image_right.empty()) {
            cv::imshow (windowname2.c_str(), ib.image_right);
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(400));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ShowCameraImage> application_show_camera_image ("ShowCameraImages");

}
