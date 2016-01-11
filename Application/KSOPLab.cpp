
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>


namespace DerWeg {

  /** eine sehr einfache Beispielanwendung, die das Fahrzeug zyklisch vorwaerts und rueckwaerts fahren laesst */
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
        obstacle = false;
        int c=0, i, j;
        // ToDo: detect obstacles here
        //depth.cols;
        //depth.rows;

        for(i=depth.rows/3; i<2*depth.rows/3; i++)
        {
            for(j=depth.cols/3; j<=2*depth.cols/3; j++)
            {
                if(depth.at<float>(i, j) < 2)
                {
                    c++;
                }

            }

        }

        if(c > depth.rows*depth.cols/20){
          LOUT(c << std::endl);
          obstacle = true;
        }//LOUT("Depth " << depth.at<float>(200, 300) << std::endl);
        //depth.at<float>(i, j) // access to an element of distance image
        // conf.at<unsigned char>(i,j) // access to an element of confidence image

}

    void set_motion() {
        Velocity dv;
        dv.steer = Angle::deg_angle(0);
        dv.velocity = 0;


        if(obstacle == true) {
            dv.velocity = 0.4;
             dv.steer = Angle::deg_angle(30);


                    // ToDo: react on obstacle here
            LOUT("Detected obstacle!" << std::endl);
        } else {
            dv.velocity = 0.6;  // ToDo: react on no obstacle here
            dv.steer = Angle::deg_angle(0);

        }

        BBOARD->setDesiredVelocity(dv);
    }

    void execute () {
      try{

        while (true) {
            BBOARD->waitForImage();   // wait for next image
            ib=BBOARD->getImage();    // get next image pair from blackboard


            // if there is no image, display warning
            if (ib.image.empty()) {
                EOUT ("Empty-image!\n");
            } else {
            // if there was an image

            // compute stereo images
            stereoGPU.runStereoMatching (ib.image, ib.image_right); // starte die Tiefenberechnung

            // get the rectified image
            stereoGPU.getRectifiedLeftImage (rect);
//            LOUT(img1->Width);
//            LOUT(img1->Height);

            // get the stereo image
            stereoGPU.getStereoResults (depth,conf); // warte, bis die Tiefenberechnung abgeschlossen ist und hole die Ergebnisse ab (in depth, conf, disp, rect)


            // display
            cv::imshow (windowname1.c_str(),rect);  // zeige das rektifizierte Bild an
            cv::imshow (windownameVisu.c_str(),depth*0.1f);  // zeige Tiefenbild an

            // detect obstacles
            detect_obstacle();
            }

            // set motion of vehicle accordingly
            set_motion();


            // DO NOT EDIT BELOW!
            // this is for thread handling.
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::KSOPLab> application ("KSOPLab");

}
