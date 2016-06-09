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

    /** Simple implemenation of a 2x2 Matrix */
    class CovMat {
        private:
            double a, b, c, d;
            /* C = (a, b;
                    c, d)
            */

        public:
            CovMat() {}
            CovMat(const double a0, const double b0,
                   const double c0, const double d0) :
                a(a0), b(b0), c(c0), d(d0) {}
            CovMat(std::vector<double> v) :
                a(v[0]), b(v[1]), c(v[2]), d(v[3]) {}
            ~CovMat() {}

            CovMat operator+ (const CovMat& C) const {
                // Matrix-Matrix-Sum
                return CovMat(a+C.a, b+C.b, c+C.c, d+C.d);
            }

            CovMat operator* (const CovMat& C) {
                // Matrix-Matrix-Product
                return CovMat(a*C.a + b*C.c, a*C.b + b*C.d,
                               c*C.a + d*C.c, c*C.b + d*C.d);
            }

            Vec operator* (const Vec& v) const {
                // Matrix-Vector-Product
                return Vec(a*v.x + b*v.y, c*v.x + d*v.y);
            }

            double det() const {
                // Determinant
                return a*d - b*c;
            }

            CovMat inv() const {
                // Inverse
                double det = this->det();
                return CovMat(d/det, -b/det, -c/det, a/det);
                }

            CovMat t() const {
                // Transpose
                return CovMat(a, c, b, d);
            }

        };



  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    cv::Mat depth, conf, rect;

    int sliderPos; int erode_size; int stopdis;
    int lower_red_h1; int lower_red_h2; int lower_red_v1; int lower_red_v2;
    int upper_red_h1; int upper_red_h2; int upper_red_v1; int upper_red_v2;
    int yellow_h1; int yellow_h2; int yellow_v1; int yellow_v2;
    int green_h1; int green_h2; int green_v1; int green_v2;

    // Localization variables
    State state;
    std::vector<Vec> tl_mean;
    std::vector<CovMat> tl_covar;
    CovMat C_measure_prototype;

  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt") {}

    /** Destruktor */
    ~TrafficLightDetection () {}

    void init(const ConfigReader& cfg) {
        cfg.get("Ampelerkennung::sliderPos", sliderPos);
        cfg.get("Ampelerkennung::erode_size", erode_size);
        cfg.get("Ampelerkennung::lower_red_h1", lower_red_h1);
        cfg.get("Ampelerkennung::lower_red_h2", lower_red_h2);
        cfg.get("Ampelerkennung::lower_red_v1", lower_red_v1);
        cfg.get("Ampelerkennung::lower_red_v2", lower_red_v2);
        cfg.get("Ampelerkennung::upper_red_h1", upper_red_h1);
        cfg.get("Ampelerkennung::upper_red_h2", upper_red_h2);
        cfg.get("Ampelerkennung::upper_red_v1", upper_red_v1);
        cfg.get("Ampelerkennung::upper_red_v2", upper_red_v2);
        cfg.get("Ampelerkennung::green_h1", green_h1);
        cfg.get("Ampelerkennung::green_h2", green_h2);
        cfg.get("Ampelerkennung::green_v1", green_v1);
        cfg.get("Ampelerkennung::green_v2", green_v2);
        cfg.get("Ampelerkennung::yellow_h1", yellow_h1);
        cfg.get("Ampelerkennung::yellow_h2", yellow_h2);
        cfg.get("Ampelerkennung::yellow_v1", yellow_v1);
        cfg.get("Ampelerkennung::yellow_v2", yellow_v2);
        cfg.get("Ampelerkennung::stopdistance", stopdis);

        // Localization init
        std::vector<double> init_mean;
        cfg.get("TrafficLightDetection::init_mean", init_mean);
        for (unsigned int i = 0; i < init_mean.size()/2; ++i) {
            tl_mean[i] = Vec(init_mean[i*2], init_mean[i*2 + 1]);
        }

        std::vector<double> init_cov;
        cfg.get("TrafficLightDetection::init_cov", init_cov);
        for (unsigned int i = 0; i < init_mean.size()/2; ++i) {
            tl_covar[i] = CovMat(init_cov);
        }

        std::vector<double> covmeasure;
        cfg.get("TrafficLightDetection::C_measure", covmeasure);
        C_measure_prototype = CovMat(covmeasure);
    }


    void execute () {
      try{
        while (true) {
          BBOARD->waitForImage();
          ib = BBOARD->getImage();
          state = BBOARD->getState();

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
          Mat yellow_hue_range;
          Mat green_hue_range;
          //red
          inRange(im_hsv, cv::Scalar(lower_red_h1,100,lower_red_v1), cv::Scalar(lower_red_h2,255,lower_red_v2), lower_red_hue_range);
          inRange(im_hsv, cv::Scalar(upper_red_h1,100,upper_red_v1), cv::Scalar(upper_red_h2,255,upper_red_v2), upper_red_hue_range);
          addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_range,-1);
          //yellow
          inRange(im_hsv, cv::Scalar(yellow_h1,100,yellow_v1), cv::Scalar(yellow_h2,255,yellow_v2), yellow_hue_range);
          //green
          inRange(im_hsv, cv::Scalar(green_h1,100,green_v1), cv::Scalar(green_h2,255,green_v2), green_hue_range);

          //erode
          Mat element = getStructuringElement( MORPH_ELLIPSE,Size( 2*erode_size + 1, 2*erode_size+1 ),Point( erode_size, erode_size ) );
          erode(red_hue_range, red_hue_range, element);
          erode(yellow_hue_range, yellow_hue_range, element);
          erode(green_hue_range, green_hue_range, element);

          //edge detection
          red_hue_range = red_hue_range >= sliderPos;
          yellow_hue_range = yellow_hue_range >= sliderPos;
          green_hue_range = green_hue_range >= sliderPos;

          vector<vector<Point> > contours_red;
          vector<vector<Point> > contours_yellow;
          vector<vector<Point> > contours_green;
          findContours(red_hue_range, contours_red, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
          findContours(yellow_hue_range, contours_yellow, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
          findContours(green_hue_range, contours_green, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

          //find ellipse
          //red
          vector<Point> center;
          int x[3] = {0,0,0}; //{if_red, if_yellow, if_green}
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
          center.push_back(Point(box.center.x,box.center.y+140));
          x[0] = 1;
          }

          //yellow
          for(size_t i = 0; i < contours_yellow.size(); i++){
          size_t count = contours_yellow[i].size();
          if( count < 10 || count > 100)
          continue;

          Mat pointsf;
          Mat(contours_yellow[i]).convertTo(pointsf, CV_32F);
          RotatedRect box = fitEllipse(pointsf);

          if( MAX(box.size.width, box.size.height) > MIN(box.size.width, box.size.height)*1.5)
          continue;
          //ellipse(cimage, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,255), 1, CV_AA);
          center.push_back(Point(box.center.x,box.center.y+140));
          x[1] = 1;
          }

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
          center.push_back(Point(box.center.x, box.center.y+140));
          x[2] = 1;
          }

          //distance calculate
          float distance = 0.0;
          for(unsigned int i = 0; i <= center.size(); i++){
              distance += depth.at<float>(center[i].x, center[i].y);
          }
          distance = distance/center.size();

          //Velocity control
          Velocity dv;
          if((x[0]+x[1]) >= 1 && distance <= stopdis){
              dv.velocity = 0;
          }else
              dv.velocity = 1;

          BBOARD->setDesiredVelocity(dv);


          //==================================================================
          // LOCALIZATION

          // Assumed postion of detected traffic lights in car coordinates
          // COMES FROM ABOVE
          Vec tl_position(1, 0);

          // Logic for reasoning which traffic light
          // Depends on which point car comes from {1, 2, 3, 4}
          // TO BE IMPLEMENTED
          int tl_index = 1;

          // Scale measurement covariance matrix in car coordinates
          CovMat C_measure = C_measure_prototype;

          // Convert to global coordinates
          tl_position += state.position;
          double alpha = state.orientation.get_rad_pi() +
                          std::atan2(tl_position.y, tl_position.x);
          double alpha_sin = std::sin(alpha);
          double alpha_cos = std::cos(alpha);
          CovMat rotation(alpha_cos, alpha_sin, -alpha_sin, alpha_cos);
          C_measure = rotation.t() * (C_measure * rotation);

          // Update estimate
          CovMat C_inv = (C_measure + tl_covar[tl_index]).inv();
          tl_mean[tl_index] = C_measure * (C_inv * tl_mean[tl_index]) +
                              tl_covar[tl_index] * (C_inv * tl_position);
          tl_covar[tl_index] = C_measure * (C_inv * tl_covar[tl_index]);

          // Plot measured and estimated positions in AnicarViewer
          // measured: red
          // estimated: dark green
          std::stringstream tl_pos;
          tl_pos << "thick red dot "
                 << tl_position.x << " " << tl_position.y << std::endl;
          for (unsigned int i = 0; i < tl_mean.size(); ++i) {
              tl_pos << "thick darkGreen dot "
                     << tl_mean[i].x << " " << tl_mean[i].y << std::endl;
          }
          BBOARD->addPlotCommand(tl_pos.str());

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
