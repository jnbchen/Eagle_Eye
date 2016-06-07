
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>

using namespace std;

namespace DerWeg {

  /** Eine sehr einfache Beispiel- und Testanwendung, die lediglich die
   *  Stereo-Kamera- und Tiefenbilder bilder im 500ms-Rythmus anzeigt */
  class ShowStereoCameraImage : public KogmoThread {
    std::string windowname1, windowname2;
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    std::string windownameRect, windownameConf, windownameVisu;
    cv::Mat depth, conf, rect;
  public:
    /** Konstruktor initialisiert die Opencv-Fenster und den Tiefenschaetzer */
    ShowStereoCameraImage () :
      windowname1 ("Camera Image 1"), windowname2 ("Camera Image 2"),
      stereoGPU ("/home/common/calib.txt"),
      windownameRect ("rectified"), windownameConf ("confidence"), windownameVisu ("stereo") {
      cvNamedWindow (windowname1.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windowname2.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameRect.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameConf.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameVisu.c_str(), CV_WINDOW_AUTOSIZE);
    }
    /** Destruktor schliesst die Opencv-Fenster */
    ~ShowStereoCameraImage () {
      cvDestroyWindow (windowname1.c_str());
      cvDestroyWindow (windowname2.c_str());
      cvDestroyWindow (windownameRect.c_str());
      cvDestroyWindow (windownameConf.c_str());
      cvDestroyWindow (windownameVisu.c_str());
    }
    void execute () {
      try{
        while (true) {
          BBOARD->waitForImage();   // wait for next image
          ib=BBOARD->getImage();    // get next image pair from blackboard
          if (!ib.is_empty()) {     // display left image, if available
            cv::imshow (windowname1.c_str(), ib.image);
          }
          if (ib.is_stereo()) {     // display right image, if available
            cv::imshow (windowname2.c_str(), ib.image_right);
          }
//          DerWeg::Timestamp now;
          stereoGPU.runStereoMatching (ib.image, ib.image_right); // starte die Tiefenberechnung
//          EOUT("Zeit fuer Stereoanstoss: " << now.elapsed_msec() << '\n');
          stereoGPU.getRectifiedLeftImage (rect);
//          EOUT("Zeit fuer Rektifikation: " << now.elapsed_msec() << '\n');
          stereoGPU.getStereoResults (depth,conf); // warte, bis die Tiefenberechnung abgeschlossen ist und hole die Ergebnisse ab (in depth, conf, disp, rect)
//          EOUT("Zeit fuer Stereo: " << now.elapsed_msec() << '\n');
          cv::imshow (windownameRect.c_str(),rect);  // zeige das rektifizierte Bild an
          cv::imshow (windownameConf.c_str(),conf);  // zeige die Sicherheit des Tiefenschaetzers fuer jedes Pixel an
          cv::imshow (windownameVisu.c_str(),depth*0.1f);  // zeige Tiefenbild an



          // TESTING!!!
          /*
int c=0;
            LOUT( "Type CV_8UC1: "<<bool(depth.type() == CV_8UC1) <<endl);
            LOUT( "Type CV_8UC3: "<<bool(depth.type() == CV_8UC3) <<endl);
            LOUT( "Type CV_32F: "<<bool(depth.type() == CV_32F) <<endl);
            LOUT( "Type CV_64F: "<<bool(depth.type() == CV_64F) <<endl);
            LOUT("Type "<<depth.type()<<endl);
            LOUT("Channels "<<depth.channels()<<endl);
            for(int i = 0; i < depth.rows; i++) {
                for(int j = 0; j < depth.cols; j++) {
                    float intensity = depth.at<float>(i,j);

                    if (c%700==0 and conf.at<int>(i,j) > 170) {
                        //LOUT( "i,j = "<< i << ", " <<j<<endl);
                        LOUT( intensity << "   ");


                        //image.at<uchar>(i, j)=255;
                    }
                    c +=1;
                }
            }
            LOUT(endl);
            */


          boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ShowStereoCameraImage> application_show_stereo_camera_image ("ShowStereoCameraImages");

}
