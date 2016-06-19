#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "TrafficLight.h"
#include "ImageProcessingFunctions.h"
#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>


using namespace cv;
using namespace std;

namespace DerWeg {

    // models the ellipses detected inthe filtered image, used for decing on traffic light state
    struct DetectedEllipse {
        RotatedRect box;
        TrafficLightState color;
        double distance;
        Mat world_coords;

        DetectedEllipse (RotatedRect rect, TrafficLightState state) : box(rect), color(state), distance(0) {}
    };


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

    double red_height, yellow_height, green_height, height_tol;

    map<int, TrafficLight> traffic_lights;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt"), windowname("Processed Image") {
        cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);
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
        cfg.get("TrafficLightDetection::red_height", red_height);
        cfg.get("TrafficLightDetection::yellow_height", yellow_height);
        cfg.get("TrafficLightDetection::green_height", green_height);
        cfg.get("TrafficLightDetection::height_tol", height_tol);

        Mat projection_matrix;
        stereoGPU.getProjectionMatrix(projection_matrix);

        transformer = CoordinateTransform(cfg, projection_matrix);


        int min_observations;
        double covarcoeff_x, covarcoeff_y;
        std::vector<double> init_mean;
        std::vector<double> init_covariance;
        cfg.get("TrafficLightDetection::min_observations", min_observations);
        cfg.get("TrafficLightDetection::covarcoeff_x", covarcoeff_x);
        cfg.get("TrafficLightDetection::covarcoeff_y", covarcoeff_y);
        cfg.get("TrafficLightDetection::init_mean", init_mean);
        cfg.get("TrafficLightDetection::init_cov", init_covariance);

        for (size_t i = 1; i <= 4; ++i) {
            traffic_lights[i] = TrafficLight(min_observations, covarcoeff_x, covarcoeff_y);
            vector<double>::const_iterator first = init_mean.begin() + 2*(i - 1);
            vector<double>::const_iterator last = init_mean.begin() + 2*(i - 1) + 2;
            std::vector<double> sub_vec(first, last);
            traffic_lights[i].set_position(sub_vec);
            traffic_lights[i].set_covar(init_covariance);
        }


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


          // Watch out, these calls change the input image !!
          findContours(red_hue_range, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
          findContours(green_hue_range, contours_green, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);


          // Filter the detected ellipses
          vector<DetectedEllipse> detectedEllipses;
          // red
          for(size_t i = 0; i < contours_red.size(); i++){
              size_t count = contours_red[i].size();
              //TODO: Ist 200 als Grenze okay ?? Oder kann eine Kontour auch über 200 pixel liegen?
              if( count < 10 || count > 200)
                continue;

              Mat pointsf;
              Mat(contours_red[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              // shift back the cutoff from region of interest
              box.center.y += rec.y;

              detectedEllipses.push_back(DetectedEllipse(box, red));
          }
          // green
          for(size_t i = 0; i < contours_green.size(); i++){
              size_t count = contours_green[i].size();
              //TODO: Ist 200 als Grenze okay ?? Oder kann eine Kontour auch über 200 pixel liegen?
              if( count < 10 || count > 200)
                continue;

              Mat pointsf;
              Mat(contours_green[i]).convertTo(pointsf, CV_32F);
              RotatedRect box = fitEllipse(pointsf);

              // shift back the cutoff from region of interest
              box.center.y += rec.y;

              detectedEllipses.push_back(DetectedEllipse(box, green));
          }


          State state = BBOARD->getState();
          for(int i = detectedEllipses.size() - 1; i >= 0; i--){
            DetectedEllipse dEllipse = detectedEllipses[i];
            RotatedRect& box = dEllipse.box;
            //ellipse center
            float u = box.center.x;
            float v = box.center.y;
            //TODO: Also use confidence for depth

            Rect box_rect = box.boundingRect();

            // draw ellipses into image
            ellipse(image, box, Scalar(0,255,255));
            rectangle(image, box_rect, Scalar(255,0,0));


            if (box_rect.width > 1.5 * box_rect.height || box_rect.height > 2.5 * box_rect.width) {
                detectedEllipses.erase(detectedEllipses.begin() + i);
                continue;
            }

            double distance = median(depth(box_rect));
            dEllipse.distance = distance;

            Mat camera_coords = transformer.image_to_camera_coords(u, v, distance);
            // Convert metres to millimetres
            camera_coords *= 1000;
            Mat world_coords = transformer.camera_to_world_coords(camera_coords, state);

            dEllipse.world_coords = world_coords;

            double height = world_coords.at<double>(2,0);

            //LOUT("image_coords = " << std::endl << " " << image_coords << std::endl);
            //LOUT("camera_coords = " << std::endl << " " << camera_coords << std::endl);

            // erase ellipses if their height does not match the expected height
            if (dEllipse.color == red &&
                    !( abs(height - red_height) < height_tol || abs(height - yellow_height) < height_tol )
                ) {
                   detectedEllipses.erase(detectedEllipses.begin() + i);
                   continue;
            } else if (dEllipse.color == green &&
                    !( abs(height - green_height) < height_tol)
                ) {
                   detectedEllipses.erase(detectedEllipses.begin() + i);
                   continue;
            }

          }

          // Show results of ellipse fitting
          cv::imshow (windowname.c_str(), image);



          int count_red_ellipses = 0, count_green_ellipses = 0;

          for (unsigned int i=0; i < detectedEllipses.size(); i++) {
            if (detectedEllipses[i].color == red) {
                count_red_ellipses++;
            } else if (detectedEllipses[i].color == green) {
                count_green_ellipses++;
            }
          }

          TrafficLightState final_state;

          if (count_red_ellipses == 0 && count_green_ellipses == 0) {
            final_state = none;
          } else if (count_red_ellipses == 0) {
            final_state = green;
          } else {
            final_state = red;
          }


          TrafficLightData tl_data;

          ReferenceTrajectory rt = BBOARD->getReferenceTrajectory();
          int tl_number;
          if (rt.path.behind_intersec) {
            tl_number = (rt.segment_id % 10);
          } else {
            tl_number = (rt.segment_id / 10);
          }

          TrafficLight& tl = traffic_lights[tl_number];

          tl.observe_state(final_state);
          tl_data.state = tl.getState();

          if (final_state != none) {
            Mat tl_pos;
            double distance;
            for (unsigned int i=0; i < detectedEllipses.size(); i++) {
                if (detectedEllipses[i].color == final_state) {
                    tl_pos = detectedEllipses[i].world_coords;
                    distance = detectedEllipses[i].distance;
                    break;
                }
            }
            tl.update_position(tl_pos, distance, state);
            tl_data.position = tl.get_position();
          } else {
            tl_data.position = Vec(0,0);
          }

          BBOARD->setTrafficLight(tl_data);


          // Plot measured position as red dot in AnicarViewer
          std::stringstream pos;
          pos << "thick red dot "
              << tl_data.position.x << " " << tl_data.position.y << std::endl;
          BBOARD->addPlotCommand(pos.str());

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
