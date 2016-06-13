#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>


using namespace cv;


namespace DerWeg {


  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    Mat depth, conf, rect;

    int erode_size;
    int lower_red_h1; int lower_red_h2; int lower_red_v1; int lower_red_v2;
    int upper_red_h1; int upper_red_h2; int upper_red_v1; int upper_red_v2;
    int yellow_h1; int yellow_h2; int yellow_v1; int yellow_v2;
    int green_h1; int green_h2; int green_v1; int green_v2;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt") {}

    /** Destruktor */
    ~TrafficLightDetection () {}

    void init(const ConfigReader& cfg) {
        cfg.get("TrafficLigthDetection::erode_size", erode_size);
        cfg.get("TrafficLigthDetection::lower_red_h1", lower_red_h1);
        cfg.get("TrafficLigthDetection::lower_red_h2", lower_red_h2);
        cfg.get("TrafficLigthDetection::lower_red_v1", lower_red_v1);
        cfg.get("TrafficLigthDetection::lower_red_v2", lower_red_v2);
        cfg.get("TrafficLigthDetection::upper_red_h1", upper_red_h1);
        cfg.get("TrafficLigthDetection::upper_red_h2", upper_red_h2);
        cfg.get("TrafficLigthDetection::upper_red_v1", upper_red_v1);
        cfg.get("TrafficLigthDetection::upper_red_v2", upper_red_v2);
        cfg.get("TrafficLigthDetection::green_h1", green_h1);
        cfg.get("TrafficLigthDetection::green_h2", green_h2);
        cfg.get("TrafficLigthDetection::green_v1", green_v1);
        cfg.get("TrafficLigthDetection::green_v2", green_v2);
//        cfg.get("TrafficLigthDetection::yellow_h1", yellow_h1);
//        cfg.get("TrafficLigthDetection::yellow_h2", yellow_h2);
//        cfg.get("TrafficLigthDetection::yellow_v1", yellow_v1);
//        cfg.get("TrafficLigthDetection::yellow_v2", yellow_v2);

    }


    void execute () {
      try{
        while (true) {
          BBOARD->waitForImage();
          ib = BBOARD->getImage();

          // starte die Tiefenberechnung
          stereoGPU.runStereoMatching (ib.image, ib.image_right);
          stereoGPU.getRectifiedLeftImage (rect);
          // warte, bis die Tiefenberechnung abgeschlossen ist und
          // hole die Ergebnisse ab (in depth, conf, disp, rect)
          stereoGPU.getStereoResults (depth,conf);

          //==================================================================
          // DETECTION

          // windows
          Mat image(ib.image);
          CvRect rec;
          rec.x=0,rec.y=140,rec.width=659,rec.height=200;
          image = image(rec);
          //blur and hsv
          medianBlur(image, image, 3);
          Mat im_hsv;
          cvtColor(image, im_hsv, cv::COLOR_BGR2HSV);
          //thresholing
          Mat lower_red_hue_range;
          Mat upper_red_hue_range;
          Mat red_hue_range;
//          Mat yellow_hue_range;
          Mat green_hue_range;
          //red
          inRange(im_hsv, cv::Scalar(lower_red_h1,100,lower_red_v1), cv::Scalar(lower_red_h2,255,lower_red_v2), lower_red_hue_range);
          inRange(im_hsv, cv::Scalar(upper_red_h1,100,upper_red_v1), cv::Scalar(upper_red_h2,255,upper_red_v2), upper_red_hue_range);
          addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_range,-1);
          //yellow
//          inRange(im_hsv, cv::Scalar(yellow_h1,100,yellow_v1), cv::Scalar(yellow_h2,255,yellow_v2), yellow_hue_range);
          //green
          inRange(im_hsv, cv::Scalar(green_h1,100,green_v1), cv::Scalar(green_h2,255,green_v2), green_hue_range);

          //erode
          Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*erode_size + 1, 2*erode_size+1 ),Point( erode_size, erode_size ) );
//          erode(red_hue_range, red_hue_range, element);
//          erode(yellow_hue_range, yellow_hue_range, element);
//          erode(green_hue_range, green_hue_range, element);
          morphologyEx(red_hue_range, red_hue_range, MORPH_OPEN, element);
          morphologyEx(green_hue_range, green_hue_range, MORPH_OPEN, element);


          vector<vector<Point> > contours_red;
//          vector<vector<Point> > contours_yellow;
          vector<vector<Point> > contours_green;
          findContours(red_hue_range, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
//          findContours(yellow_hue_range, contours_yellow, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
          findContours(green_hue_range, contours_green, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

          //find ellipse
          //red
          vector<RotatedRect> red_ellipses;
          vector<RotatedRect> green_ellipses;

          for(size_t i = 0; i < contours_red.size(); i++){
              size_t count = contours_red[i].size();
              if( count < 10 || count > 100)
                continue;

              Mat pointsf;
              Mat(contours_red[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
                continue;
              //ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
              box.center.y += rec.y;
              red_ellipses.push_back(box);
          }

//          //yellow
//          for(size_t i = 0; i < contours_yellow.size(); i++){
//              size_t count = contours_yellow[i].size();
//              if( count < 10 || count > 100)
//                continue;
//
//              Mat pointsf;
//              Mat(contours_yellow[i]).convertTo(pointsf, CV_32F);
//              RotatedRect box = fitEllipse(pointsf);
//
//              if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
//                continue;
//              //ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
//              center.push_back(Point(box.center.x,box.center.y+rec.y));
//              x[1] = 1;
//          }

          //green
          for(size_t i = 0; i < contours_green.size(); i++){
              size_t count = contours_green[i].size();
              if( count < 10 || count > 100)
                continue;

              Mat pointsf;
              Mat(contours_green[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
                continue;
              //ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
              box.center.y += rec.y;
              green_ellipses.push_back(box);
          }


          //=====================================================================
          /*
          TODO:
          Tiefeninformation für jede Ellipse speichern
          Validitätscheck für jede Ellipse (passt der Durchmesser und die Höhe im bild zur berechneten Tiefe?)
          Entscheidung für einen Ampelzustand  und update auf dem blackboard
          Positionsberechnung
          */



          //distance calculate
//          float distance = 0.0;
//          for(unsigned int i = 0; i <= red_ellipses.size(); i++){
//              distance += depth.at<float>(red_ellipses[i].x, red_ellipses[i].y);
//          }
//          distance = distance/red_ellipses.size();

          /** get rectified projection matrix e.g. for reconstructing 3D points
         * @param projection_matrix
         *   3x4 projection matrix of the (virtual) rectified camera */
          Mat projection_matrix;
          stereoGPU.getProjectionMatrix(projection_matrix);



          //==================================================================
          // LOCALIZATION

          // Assumed values of detected traffic light in car coordinates
          // COMES FROM ABOVE
          Vec position(1, 0);
          double distance = 0;
          double confidence = 0;

          // Logic for reasoning which traffic light
          // Depends on which point car comes from {1, 2, 3, 4}
          // TO BE IMPLEMENTED
          int tl_index = 1;


          State state = BBOARD->getState();
          //traffic_lights[tl_index].update_position(position, state, distance, confidence);

          // Plot measured positoin as red dot in AnicarViewer
          std::stringstream pos;
          pos << "thick red dot "
              << position.x << " " << position.y << std::endl;

          //==================================================================

          boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TrafficLightDetection> application_save_stereo_camera_image ("TrafficLightDetection");

}
