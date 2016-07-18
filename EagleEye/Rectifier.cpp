#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>


namespace DerWeg {

    class Rectifier : public KogmoThread {
        private:
            ImageBuffer ib;
            State state;
            ReferenceTrajectory rt;

            DerWeg::StereoGPU stereoGPU;

            cv::Mat rect_left;
            cv::Mat rect_right;

        public:
            Rectifier () : stereoGPU("/home/common/calib.txt") {}

            ~Rectifier () {}

            void execute () {
                try{
                    BBOARD->waitForImage();
                    while (true) {

                        if (!stereoGPU.isRunning()) {

                            ib = BBOARD->getImage();
                            state = BBOARD->getState();
                            rt = BBOARD->getReferenceTrajectory();

                            stereoGPU.runStereoMatching(ib.image, ib.image_right);
                            stereoGPU.getRectifiedLeftImage(rect_left);
                            stereoGPU.getRectifiedRightImage(rect_right);

                            RectImages rect_imgs;
                            rect_imgs.images.image = rect_left;
                            rect_imgs.images.image_right = rect_right;
                            rect_imgs.state = state;
                            rect_imgs.reference_trajectory = rt;

                            BBOARD->setRectImages(rect_imgs);
                        }

                        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
                        boost::this_thread::interruption_point();
                    }
                } catch(boost::thread_interrupted&){;}
            }
    };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::Rectifier> application_save_stereo_camera_image ("Rectifier");

}
