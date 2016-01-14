
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

namespace DerWeg {
  
  class KSOPLab : public KogmoThread {
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    std::string windowname1, windownameVisu;
    cv::Mat depth, conf, disp, rect, visu;
    bool obstacle;
  public:
    KSOPLab () :
    windowname1 ("Camera Image 1"),
    stereoGPU ("/home/common/calib.txt"),
    windownameVisu ("stereo"),
    obstacle(false) {
      cvNamedWindow (windowname1.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameVisu.c_str(), CV_WINDOW_AUTOSIZE);
    }
    ~KSOPLab () {;}

    void detect_obstacle() {
      /*
      \todo: Detect obstacles here

      You can use accessors like depth.cols or depth.rows to get the size
      of the depth image. Also, you can create a region of interest using
      depth(cv::Rect(x, y, width, height)). There are several statistical
      functions that you can find online
      (--> Google: "open cv operations on arrays")
      */

      obstacle = false;
    }

    void set_motion() {
      /*
      Set the vehicle motion according to the obstacle detection.
      You can set the vehicle velocity using the "Velocity" structure:

      v_desired.steer = Angle::deg_angle(0);
      v_desired.velocity = 0;
      */

      Velocity v_desired;

      if(obstacle == true) {
        // Obstacle detected
        LOUT("Obstacle detected" << std::endl);

        /*
        \todo: React on detection
        */
      } else {
        // No detection
      }

      // Set the desired velocity accordingly
      BBOARD->setDesiredVelocity(v_desired);
    }

    void execute () {
      try{

        while (true) {
          BBOARD->waitForImage();   // wait for next image
          ib=BBOARD->getImage();    // get next image pair from blackboard


          if (ib.image.empty()) {
            // Display warning if there is no image
            EOUT ("Empty-image!\n");
          } else {
            // Compute stereo images
            stereoGPU.runStereoMatching (ib.image, ib.image_right);

            // Get the rectified image
            stereoGPU.getRectifiedLeftImage (rect);

            // Get the stereo image
            stereoGPU.getStereoResults (depth, conf);

            // Display rectified and depth image
            cv::imshow (windowname1.c_str(), rect);
            cv::imshow (windownameVisu.c_str(), depth*0.1f);

            // Detect obstacles
            detect_obstacle();
          }

          // Set motion of vehicle accordingly
          set_motion();

          /*
          DO NOT EDIT BELOW!
          This is for thread handling.
          */
          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };
}

namespace {

  // Register plugin
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::KSOPLab> application ("KSOPLab");
}
