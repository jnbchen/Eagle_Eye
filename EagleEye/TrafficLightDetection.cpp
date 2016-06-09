#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace cv;

namespace DerWeg {

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
          // windows
          Mat image = ib.image.copy();
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
          Mat red_hue_range = red_hue_range >= sliderPos;
          Mat yellow_hue_range = yellow_hue_range >= sliderPos;
          Mat green_hue_range = green_hue_range >= sliderPos;

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
          for(i = 0; i <= center.size(); i++){
              distance += depth.at<float>(center[i].x, center[i].y);
          }
          float distance = distance/center.size();

          //Velocity control
          Velocity dv;
          if((x[0]+x[1]) >= 1 && distance <= stopdis){
              dv.velocity = 0;
          }else
              dv.velocity = 1;

          BBOARD->setDesiredVelocity(dv);
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
