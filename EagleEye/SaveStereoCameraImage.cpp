
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <string.h>

using namespace std;

namespace DerWeg {

  /** Eine sehr einfache Beispiel- und Testanwendung, die lediglich die
   *  Stereo-Kamera- und Tiefenbilder bilder im 500ms-Rythmus anzeigt */
  class SaveStereoCameraImage : public KogmoThread {
    std::string windowname1, windowname2;
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    std::string windownameRect, windownameConf, windownameVisu;
    cv::Mat depth, conf, rect;
  public:
    /** Konstruktor initialisiert die Opencv-Fenster und den Tiefenschaetzer */
    SaveStereoCameraImage () :
      windowname1 ("Camera Image 1"), windowname2 ("Camera Image 2"),
      stereoGPU ("/home/common/calib.txt"),
      windownameRect ("rectified"), windownameConf ("confidence"), windownameVisu ("stereo") {
      /*
      cvNamedWindow (windowname1.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windowname2.c_str(),    CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameRect.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameConf.c_str(), CV_WINDOW_AUTOSIZE);
      cvNamedWindow (windownameVisu.c_str(), CV_WINDOW_AUTOSIZE);
      */
    }
    /** Destruktor schliesst die Opencv-Fenster */
    ~SaveStereoCameraImage () {
    /*
      cvDestroyWindow (windowname1.c_str());
      cvDestroyWindow (windowname2.c_str());
      cvDestroyWindow (windownameRect.c_str());
      cvDestroyWindow (windownameConf.c_str());
      cvDestroyWindow (windownameVisu.c_str());
    */
    }
    void execute () {
      try{
        while (true) {
          BBOARD->waitForImage();   // wait for next image
          ib=BBOARD->getImage();    // get next image pair from blackboard
          if (!ib.is_empty()) {     // display left image, if available
            //cv::imshow (windowname1.c_str(), ib.image);
          }
          if (ib.is_stereo()) {     // display right image, if available
            //cv::imshow (windowname2.c_str(), ib.image_right);
          }
//          DerWeg::Timestamp now;
          stereoGPU.runStereoMatching (ib.image, ib.image_right); // starte die Tiefenberechnung
//          EOUT("Zeit fuer Stereoanstoss: " << now.elapsed_msec() << '\n');
          stereoGPU.getRectifiedLeftImage (rect);
//          EOUT("Zeit fuer Rektifikation: " << now.elapsed_msec() << '\n');
          stereoGPU.getStereoResults (depth,conf); // warte, bis die Tiefenberechnung abgeschlossen ist und hole die Ergebnisse ab (in depth, conf, disp, rect)
//          EOUT("Zeit fuer Stereo: " << now.elapsed_msec() << '\n');
//          cv::imshow (windownameRect.c_str(),rect);  // zeige das rektifizierte Bild an
//          cv::imshow (windownameConf.c_str(),conf);  // zeige die Sicherheit des Tiefenschaetzers fuer jedes Pixel an
//          cv::imshow (windownameVisu.c_str(),depth*0.1f);  // zeige Tiefenbild an




          //Write Images here

        time_t systemzeit;
        systemzeit = time(0);
        //char *asctime(const struct tm *t);
        string timeString;
        //timeString = ctime(&systemzeit);
        tm * localt = localtime(&systemzeit);

        ostringstream oss;
        oss << setfill('0') << setw(4) << localt->tm_year + 1900 << '-' << setw(2) <<  localt->tm_mon +1 << '-' << setw(2) << localt->tm_mday << '_'
            << setw(2) <<  localt->tm_hour << '-' << setw(2) << localt->tm_min << '-' << setw(2) << localt->tm_sec;
        timeString = oss.str();



        char rectFileName[100];
        strcpy(rectFileName, "../data/StereoImages/Stereo_");
        strcat(rectFileName, timeString.c_str());

          cv::imwrite(string(rectFileName) + "_rect.png", rect);

        char depthFileName[100];
        strcpy(depthFileName, "../data/StereoImages/Stereo_");
        strcat(depthFileName, timeString.c_str());

          cv::imwrite(string(depthFileName) + "_depth.png", depth);

        char confFileName[100];
        strcpy(confFileName, "../data/StereoImages/Stereo_");
        strcat(confFileName, timeString.c_str());

          cv::imwrite(string(confFileName) + "_conf.png", conf);

        char leftFileName[100];
        strcpy(leftFileName, "../data/StereoImages/Stereo_");
        strcat(leftFileName, timeString.c_str());

          cv::imwrite(string(leftFileName) + "_left.png", ib.image);





          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::SaveStereoCameraImage> application_save_stereo_camera_image ("SaveStereoCameraImages");

}
