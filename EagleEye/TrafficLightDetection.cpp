#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ImageProcessingFunctions.h"
#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>


using namespace cv;


namespace DerWeg {


  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    Mat depth, conf, rect;
    CoordinateTransform transformer;

    std::string windowname;
    std::string windowname2;

    int erode_size;
    int lower_red_h1; int lower_red_h2; int lower_red_s1; int lower_red_s2; int lower_red_v1; int lower_red_v2;
    int yellow_h1; int yellow_h2; int yellow_s1; int yellow_s2; int yellow_v1; int yellow_v2;
    int green_h1; int green_h2; int green_s1; int green_s2; int green_v1; int green_v2;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt"), windowname("Processed Image") {
        cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);
        cvNamedWindow (windowname2.c_str(), CV_WINDOW_AUTOSIZE);
      }

    /** Destruktor */
    ~TrafficLightDetection () {}

    void init(const ConfigReader& cfg) {
        LOUT("Exec TrafficLightDetection init()" << std::endl);

        cfg.get("TrafficLightDetection::erode_size", erode_size);
        cfg.get("TrafficLightDetection::lower_red_h1", lower_red_h1);
        cfg.get("TrafficLightDetection::lower_red_h2", lower_red_h2);
        cfg.get("TrafficLightDetection::lower_red_s1", lower_red_s1);
        cfg.get("TrafficLightDetection::lower_red_s2", lower_red_s2);
        cfg.get("TrafficLightDetection::lower_red_v1", lower_red_v1);
        cfg.get("TrafficLightDetection::lower_red_v2", lower_red_v2);
        cfg.get("TrafficLightDetection::green_h1", green_h1);
        cfg.get("TrafficLightDetection::green_h2", green_h2);
        cfg.get("TrafficLightDetection::green_s1", green_s1);
        cfg.get("TrafficLightDetection::green_s2", green_s2);
        cfg.get("TrafficLightDetection::green_v1", green_v1);
        cfg.get("TrafficLightDetection::green_v2", green_v2);

        Mat projection_matrix;
        stereoGPU.getProjectionMatrix(projection_matrix);

        transformer = CoordinateTransform(cfg, projection_matrix);
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
          Rect rec(0, 140, 659, 200);
          //Region of Interest
          Mat ROI_image = image(rec);

          //blur and hsv
          medianBlur(ROI_image, ROI_image, 3);
          Mat im_hsv;
          cvtColor(ROI_image, im_hsv, cv::COLOR_BGR2HSV);

          //thresholing
          Mat red_hue_range;
          Mat green_hue_range;

          //red
          inRange(im_hsv, cv::Scalar(lower_red_h1,lower_red_s1,lower_red_v1), cv::Scalar(lower_red_h2,lower_red_s2,lower_red_v2), red_hue_range);

          //green
          inRange(im_hsv, cv::Scalar(green_h1,green_s1,green_v1), cv::Scalar(green_h2,green_s2,green_v2), green_hue_range);

          //TODO: OPENing / CLOSING anstatt ERODE
          Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*erode_size + 1, 2*erode_size+1 ),Point( -1, -1 ) );
          erode(red_hue_range, red_hue_range, element);
          erode(green_hue_range, green_hue_range, element);

          vector<vector<Point> > contours_red;
          vector<vector<Point> > contours_green;


          cv::imshow (windowname2.c_str(), red_hue_range );

          // Watch out, this call changes the input image !!
          findContours(red_hue_range, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
          findContours(green_hue_range, contours_green, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

          //find ellipse
          vector<RotatedRect> red_ellipses;
          vector<RotatedRect> green_ellipses;

          for(size_t i = 0; i < contours_red.size(); i++){
              size_t count = contours_red[i].size();
              //TODO: Ist 200 als Grenze okay ?? Oder kann eine Kontour auch über 200 pixel liegen?
              if( count < 10 || count > 200)
                continue;

              Mat pointsf;
              Mat(contours_red[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
                continue;

              //draw found ellipses into image
              box.center.y += rec.y;
              ellipse(image, box, Scalar(0,255,255));
              red_ellipses.push_back(box);
          }

          //green
          for(size_t i = 0; i < contours_green.size(); i++){
              size_t count = contours_green[i].size();
              if( count < 10 || count > 200)
                continue;

              Mat pointsf;
              Mat(contours_green[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
                continue;
              //draw found ellipses into image
              box.center.y += rec.y;
              ellipse(image, box, Scalar(0,255,255));
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
          for(size_t i = 0; i < red_ellipses.size(); i++){
            RotatedRect& box = red_ellipses[i];
            //ellipse center
            float u = box.center.x;
            float v = box.center.y;
            //TODO: Also use confidence for depth

            Rect box_rect = box.boundingRect();
            rectangle(image, box_rect, Scalar(255,0,0));
            double distance = median(depth(box_rect));

            Mat image_coords(3, 1, CV_64FC1);
            image_coords.at<double>(0,0) = distance * u;
            image_coords.at<double>(1,0) = distance * v;
            image_coords.at<double>(2,0) = distance;

            //Mat camera_coords = coord_trafo * image_coords;

            //LOUT("image_coords = " << std::endl << " " << image_coords << std::endl);
            //LOUT("camera_coords = " << std::endl << " " << camera_coords << std::endl);



          }

          for(size_t i = 0; i < green_ellipses.size(); i++){
            RotatedRect& box = green_ellipses[i];
            //ellipse center
            float u = box.center.x;
            float v = box.center.y;
            //TODO: Also use confidence for depth

            Rect box_rect = box.boundingRect();
            rectangle(image, box_rect, Scalar(255,0,0));
            double distance = median(depth(box_rect));

            Mat image_coords(3, 1, CV_64FC1);
            image_coords.at<double>(0,0) = distance * u;
            image_coords.at<double>(1,0) = distance * v;
            image_coords.at<double>(2,0) = distance;
            //Mat camera_coords = coord_trafo * image_coords;

            //LOUT("image_coords = " << std::endl << " " << image_coords << std::endl);
            //LOUT("camera_coords = " << std::endl << " " << camera_coords << std::endl);

          }

          // Show results of ellipse fitting
          cv::imshow (windowname.c_str(), image);

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
