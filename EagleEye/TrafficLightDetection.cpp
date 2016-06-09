
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


namespace DerWeg {

    /** Simple implemenation of a 2x2 Matrix */
    class Mat {
        private:
            double a, b, c, d;
            /* C = (a, b;
                    c, d)
            */

        public:
            Mat() {}
            Mat(const double a0, const double b0,
                   const double c0, const double d0) :
                a(a0), b(b0), c(c0), d(d0) {}
            Mat(std::vector<double> v) :
                a(v[0]), b(v[1]), c(v[2]), d(v[3]) {}
            ~Mat() {}

            Mat operator+ (const Mat& C) const {
                // Matrix-Matrix-Sum
                return Mat(a+C.a, b+C.b, c+C.c, d+C.d);
            }

            Mat operator* (const Mat& C) {
                // Matrix-Matrix-Product
                return Mat(a*C.a + b*C.c, a*C.b + b*C.d,
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

            Mat inv() const {
                // Inverse
                double det = this->det();
                return Mat(d/det, -b/det, -c/det, a/det);
                }

            Mat t() const {
                // Transpose
                return Mat(a, c, b, d);
            }

        };



  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    ImageBuffer ib;
    DerWeg::StereoGPU stereoGPU;
    cv::Mat depth, conf, rect;

    // Localization variables
    State state;
    std::vector<Vec> tl_mean;
    std::vector<Mat> tl_covar;
    Mat C_measure_prototype;

  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt") {}

    /** Destruktor */
    ~TrafficLightDetection () {}


    void init (const ConfigReader& cfg) {


        // Localization init
        std::vector<double> init_mean;
        cfg.get("TrafficLightDetection::init_mean", init_mean);
        for (unsigned int i = 0; i < init_mean.size()/2; ++i) {
            tl_mean[i] = Vec(init_mean[i*2], init_mean[i*2 + 1]);
        }

        std::vector<double> init_cov;
        cfg.get("TrafficLightDetection::init_cov", init_cov);
        for (unsigned int i = 0; i < init_mean.size()/2; ++i) {
            tl_covar[i] = Mat(init_cov);
        }

        std::vector<double> covmeasure;
        cfg.get("TrafficLightDetection::C_measure", covmeasure);
        C_measure_prototype = Mat(covmeasure);
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
          // IMPLEMENTATION HERE






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
          Mat C_measure = C_measure_prototype;

          // Convert to global coordinates
          tl_position += state.position;
          double alpha = state.orientation.get_rad_pi() +
                          std::atan2(tl_position.y, tl_position.x);
          double alpha_sin = std::sin(alpha);
          double alpha_cos = std::cos(alpha);
          Mat rotation(alpha_cos, alpha_sin, -alpha_sin, alpha_cos);
          C_measure = rotation.t() * (C_measure * rotation);

          // Update estimate
          Mat C_inv = (C_measure + tl_covar[tl_index]).inv();
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
