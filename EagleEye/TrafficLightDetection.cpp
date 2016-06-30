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


    typedef std::vector<DetectedEllipse> EllipseVector;

    class EllipseDetector {
        private:
            Rect roi;

            int median_filter_size;

            int lower_red_h1, lower_red_h2;
            int lower_red_s1, lower_red_s2;
            int lower_red_v1, lower_red_v2;
            int upper_red_h1, upper_red_h2;
            int upper_red_s1, upper_red_s2;
            int upper_red_v1, upper_red_v2;
            int yellow_h1, yellow_h2;
            int yellow_s1, yellow_s2;
            int yellow_v1, yellow_v2;
            int green_h1, green_h2;
            int green_s1, green_s2;
            int green_v1, green_v2;

            int erode_size, dilate_size;

            int min_contour_count, max_contour_count;

            double max_size_ratio;
            double min_bbox_hw_ratio, max_bbox_hw_ratio;
            int max_ellipse_size;


        public:
            void init(const ConfigReader& cfg) {

                int roi_border_top, roi_border_bot, roi_width;
                cfg.get("TrafficLightDetection::roi_border_top", roi_border_top);
                cfg.get("TrafficLightDetection::roi_border_bot", roi_border_bot);
                cfg.get("TrafficLightDetection::roi_width", roi_width);
                roi = Rect(0, roi_border_top, roi_width, roi_border_bot);

                cfg.get("TrafficLightDetection::median_filter_size", median_filter_size);

                cfg.get("TrafficLightDetection::lower_red_h1", lower_red_h1);
                cfg.get("TrafficLightDetection::lower_red_h2", lower_red_h2);
                cfg.get("TrafficLightDetection::lower_red_s1", lower_red_s1);
                cfg.get("TrafficLightDetection::lower_red_s2", lower_red_s2);
                cfg.get("TrafficLightDetection::lower_red_v1", lower_red_v1);
                cfg.get("TrafficLightDetection::lower_red_v2", lower_red_v2);
                cfg.get("TrafficLightDetection::upper_red_h1", upper_red_h1);
                cfg.get("TrafficLightDetection::upper_red_h2", upper_red_h2);
                cfg.get("TrafficLightDetection::upper_red_s1", upper_red_s1);
                cfg.get("TrafficLightDetection::upper_red_s2", upper_red_s2);
                cfg.get("TrafficLightDetection::upper_red_v1", upper_red_v1);
                cfg.get("TrafficLightDetection::upper_red_v2", upper_red_v2);
                cfg.get("TrafficLightDetection::green_h1", green_h1);
                cfg.get("TrafficLightDetection::green_h2", green_h2);
                cfg.get("TrafficLightDetection::green_s1", green_s1);
                cfg.get("TrafficLightDetection::green_s2", green_s2);
                cfg.get("TrafficLightDetection::green_v1", green_v1);
                cfg.get("TrafficLightDetection::green_v2", green_v2);

                cfg.get("TrafficLightDetection::erode_size", erode_size);
                cfg.get("TrafficLightDetection::dilate_size", dilate_size);

                cfg.get("TrafficLightDetection::min_contour_count", min_contour_count);
                cfg.get("TrafficLightDetection::max_contour_count", max_contour_count);

                cfg.get("TrafficLightDetection::max_size_ratio", max_size_ratio);
                cfg.get("TrafficLightDetection::min_bbox_hw_ratio", min_bbox_hw_ratio);
                cfg.get("TrafficLightDetection::max_bbox_hw_ratio", max_bbox_hw_ratio);
                cfg.get("TrafficLightDetection::max_ellipse_size", max_ellipse_size);
            }


            EllipseVector detect(const cv::Mat& im_input) {

                // Region of Interest
                Mat im_roi = im_input(roi);


                // Blur and HSV
                medianBlur(im_roi, im_roi, median_filter_size);
                Mat im_hsv;
                cvtColor(im_roi, im_hsv, COLOR_BGR2HSV);


                // Color thresholing
                Mat lower_red_hue_range;
                Mat upper_red_hue_range;
                Mat red_hue_range;
                Mat green_hue_range;

                inRange(im_hsv, cv::Scalar(lower_red_h1,lower_red_s1,lower_red_v1),
                        cv::Scalar(lower_red_h2,lower_red_s2,lower_red_v2), lower_red_hue_range);
                inRange(im_hsv, cv::Scalar(upper_red_h1,upper_red_s1,upper_red_v1),
                        cv::Scalar(upper_red_h2,upper_red_s2,upper_red_v2), upper_red_hue_range);
                addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_range);

                inRange(im_hsv, cv::Scalar(green_h1,green_s1,green_v1),
                        cv::Scalar(green_h2,green_s2,green_v2), green_hue_range);


                // Opening/Closing
                Mat element_erode = getStructuringElement(MORPH_ELLIPSE,
                                                          Size(2*erode_size + 1, 2*erode_size+1),
                                                          Point(-1, -1));
                Mat element_dilate = getStructuringElement(MORPH_ELLIPSE,
                                                           Size(2*dilate_size + 1, 2*dilate_size+1),
                                                           Point(-1, -1));
                // für red dilate dann erode, da der Rotkreis so dünn ist ( = closing)
                dilate(red_hue_range, red_hue_range, element_dilate);
                erode(red_hue_range, red_hue_range, element_erode);
                // für green nur erode, ansonsten werden die Blätter von Baum vergrößt
                erode(green_hue_range, green_hue_range, element_erode);


                // Get contour points (changes input image)
                vector<vector<Point> > contours_red;
                vector<vector<Point> > contours_green;
                findContours(red_hue_range, contours_red, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                findContours(green_hue_range, contours_green, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


                // Filter the detected ellipses
                EllipseVector detected_ellipses;
                // red
                for(size_t i = 0; i < contours_red.size(); i++){
                  int count = contours_red[i].size();
                  if( count < min_contour_count || count > max_contour_count)
                    continue;

                  Mat pointsf;
                  Mat(contours_red[i]).convertTo(pointsf, CV_32F);
                  RotatedRect box = fitEllipse(pointsf);

                  // shift back the cutoff from region of interest
                  box.center.y += roi.y;

                  detected_ellipses.push_back(DetectedEllipse(box, red));
                }

                // green
                for(size_t i = 0; i < contours_green.size(); i++){
                  int count = contours_green[i].size();
                  if( count < min_contour_count || count > max_contour_count)
                    continue;

                  Mat pointsf;
                  Mat(contours_green[i]).convertTo(pointsf, CV_32F);
                  RotatedRect box = fitEllipse(pointsf);

                  // shift back the cutoff from region of interest
                  box.center.y += roi.y;

                  detected_ellipses.push_back(DetectedEllipse(box, green));
                }

                LOUT("Detected ellipses count = " << detected_ellipses.size() << endl);

                // Sort out ellipses with uncorrect size ratio and too large height
                for(int i = detected_ellipses.size() - 1; i >= 0; --i) {
                    double el_width = detected_ellipses[i].box.size.width;
                    double el_height = detected_ellipses[i].box.size.height;
                    Rect bbox = detected_ellipses[i].box.boundingRect();
                    double bbox_hw_ratio = (double)bbox.height / bbox.width;

                    if (max(el_width, el_height) / min(el_width, el_height) > max_size_ratio) {
                        LOUT("Ellipse kicked because of maxsize/minsize ratio\n");
                        detected_ellipses.erase(detected_ellipses.begin() + i);
                    }
                    else if (bbox_hw_ratio < min_bbox_hw_ratio ||
                              bbox_hw_ratio > max_bbox_hw_ratio) {
                        LOUT("Ellipse kicked because of height/width ratio of bounding box\n");
                        detected_ellipses.erase(detected_ellipses.begin() + i);
                    }
                    else if (bbox.height > max_ellipse_size ||
                              bbox.width > max_ellipse_size) {
                        LOUT("Ellipse kicked because the overall size is too large\n"
                             <<"bbox height = " << bbox.height << "\n"
                             <<"bbox width = " << bbox.width << "\n");
                        detected_ellipses.erase(detected_ellipses.begin() + i);
                    }
                    else {
                        // do nothing, size ratio of ellipse is ok
                        continue;
                    }
                }

                return detected_ellipses;
          }
    };




  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    ImageBuffer ib;
    State state;
    DerWeg::StereoGPU stereoGPU;
    std::string windowname;
    std::string windowname2;
    Mat left, right;

    EllipseDetector ellipse_detector;
    CoordinateTransform transformer;

    double red_height, yellow_height, green_height, height_tol, light_diameter;
    double camera_height, v0, focus_length, base_length;

    map<int, TrafficLight> traffic_lights;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
      stereoGPU ("/home/common/calib.txt"), windowname("Processed Image"), windowname2("Right Image") {
        cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);
        cvNamedWindow (windowname2.c_str(), CV_WINDOW_AUTOSIZE);
      }

    /** Destruktor */
    ~TrafficLightDetection () {}

    void init(const ConfigReader& cfg) {
        LOUT("Exec TrafficLightDetection init()" << std::endl);

        ellipse_detector.init(cfg);

        cfg.get("TrafficLightDetection::red_height", red_height);
        cfg.get("TrafficLightDetection::yellow_height", yellow_height);
        cfg.get("TrafficLightDetection::green_height", green_height);
        cfg.get("TrafficLightDetection::height_tol", height_tol);
        cfg.get("TrafficLightDetection::light_diameter", light_diameter);

        cfg.get("TrafficLightDetection::base_length", base_length);

        vector<double> tmp;
        cfg.get("CoordinateTransform::cam_to_stargazer", tmp);
        camera_height = abs(tmp[2]); // z-coordinate

        Mat projection_matrix;
        stereoGPU.getProjectionMatrix(projection_matrix);
        v0 = projection_matrix.at<double>(1,2);
        focus_length = projection_matrix.at<double>(0,0);

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
        LOUT("Enter TLD execute()\n");
        BBOARD->waitForReferenceTrajectory();
      try{
        while (true) {
          BBOARD->waitForImage();
          ib = BBOARD->getImage();
          state = BBOARD->getState();

          stereoGPU.runStereoMatching(ib.image, ib.image_right);
          stereoGPU.getRectifiedLeftImage(left);
          stereoGPU.getRectifiedRightImage(right);

          Mat image(left);
          Mat r_image(right);


          //==================================================================
          // DETECTION

          EllipseVector ellipses_left = ellipse_detector.detect(left);
          EllipseVector ellipses_right = ellipse_detector.detect(right);
          EllipseVector detectedEllipses = ellipses_left;


          for (int i = detectedEllipses.size() - 1; i >= 0; i--) {
            DetectedEllipse& dEllipse = detectedEllipses[i];

            //ellipse center
            float u = dEllipse.box.center.x;
            float v = dEllipse.box.center.y;

            // draw into image
            ellipse(image, dEllipse.box, Scalar(0,255,255));

            //
            double disparity = 0;
            double h0 = camera_height;
            if (dEllipse.color == red) {
                double approx_distance = (red_height - h0) * focus_length / (v0 - v);
                double approx_disparity = focus_length * base_length / approx_distance;
                double tolerance = std::pow(focus_length * light_diameter/2 / approx_distance, 2);

                LOUT("Approx distance = " << approx_distance << "\n");
                LOUT("Approx disparity = " << approx_disparity << "\n");

                circle(r_image, Point(u-approx_disparity, v), std::sqrt(tolerance), Scalar(255,0,255));

                for (int j = ellipses_right.size() - 1; j >= 0; --j) {
                    if (ellipses_right[j].color == red &&
                        std::pow(u - approx_disparity - ellipses_right[j].box.center.x, 2) +
                        std::pow(v - ellipses_right[j].box.center.y, 2) <= tolerance) {
                        // Found matching ellipse
                        disparity = u - ellipses_right[j].box.center.x;
                        ellipses_right.erase(ellipses_right.begin() + j);
                        break;
                    }
                    else {
                        // Ellipses not matching
                        continue;
                    }
                }

                if (disparity == 0) {
                    double approx_distance = (yellow_height - h0) * focus_length / (v - v0);
                    double approx_disparity = focus_length * base_length / approx_distance;
                    double tolerance = std::pow(focus_length * light_diameter/2 / approx_distance, 2);

                    for (int j = ellipses_right.size() - 1; j >= 0; --j) {
                        if (ellipses_right[j].color == red &&
                            std::pow(u - approx_disparity - ellipses_right[j].box.center.x, 2) +
                            std::pow(v - ellipses_right[j].box.center.y, 2) <= tolerance) {
                            // Found matching ellipse
                            disparity = u - ellipses_right[j].box.center.x;
                            ellipses_right.erase(ellipses_right.begin() + j);
                            break;
                        }
                        else {
                            // Ellipses not matching
                            continue;
                        }
                    }
                }

            }
            else if (dEllipse.color == green) {
                double approx_distance = (green_height - h0) * focus_length / (v - v0);
                double approx_disparity = focus_length * base_length / approx_distance;
                double tolerance = std::pow(focus_length * light_diameter/2 / approx_distance, 2);

                for (int j = ellipses_right.size() - 1; j >= 0; --j) {
                    if (ellipses_right[j].color == green &&
                        std::pow(u - approx_disparity - ellipses_right[j].box.center.x, 2) +
                        std::pow(v - ellipses_right[j].box.center.y, 2) <= tolerance) {
                        // Found matching ellipse
                        disparity = u - ellipses_right[j].box.center.x;
                        ellipses_right.erase(ellipses_right.begin() + j);
                        break;
                    }
                    else {
                        // Ellipses not matching
                        continue;
                    }
                }

            }
            else {
                // Error
            }

            if (disparity == 0) {
                LOUT("Ellipse kicked because no matching ellipse was found\n");
                detectedEllipses.erase(detectedEllipses.begin() + i);
                continue;
            }

            // Distance in Millimetres
            double distance = focus_length * base_length / disparity;
            dEllipse.distance = distance;

            // Debugging depth information
            LOUT("Distance = "<<distance<<endl);

            // Transform image to world coordinates
            Mat camera_coords = transformer.image_to_camera_coords(u, v, distance);
            Mat world_coords = transformer.camera_to_world_coords(camera_coords, state);
            dEllipse.world_coords = world_coords;

            //LOUT("image_coords = " << std::endl << " " << image_coords << std::endl);
            //LOUT("camera_coords = " << std::endl << " " << camera_coords << std::endl);

            // erase ellipses if their height does not match the expected height
            double height = world_coords.at<double>(2,0);
            if (dEllipse.color == red &&
                    !( abs(height - red_height) < height_tol || abs(height - yellow_height) < height_tol )
                ) {
                   detectedEllipses.erase(detectedEllipses.begin() + i);
                   LOUT("RED Ellipse kicked out because of height tolerance\n");
                   continue;
            } else if (dEllipse.color == green &&
                    !( abs(height - green_height) < height_tol)
                ) {
                   detectedEllipses.erase(detectedEllipses.begin() + i);
                   LOUT("GREEN Ellipse kicked out because of height tolerance\n");
                   continue;
            }

          }

          // Show results of ellipse fitting
          cv::imshow (windowname.c_str(), image);
          cv::imshow (windowname2.c_str(), r_image);

          int count_red_ellipses = 0, count_green_ellipses = 0;

          for (unsigned int i=0; i < detectedEllipses.size(); i++) {
            LOUT("ellipse " << i << " world coordinates: "<< detectedEllipses[i].world_coords << std::endl);
            if (detectedEllipses[i].color == red) {
                count_red_ellipses++;
            } else if (detectedEllipses[i].color == green) {
                count_green_ellipses++;
            }
          }

            LOUT("red: "<< count_red_ellipses << ", green: "<<count_green_ellipses<<"\n");

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
          //LOUT("tl_number = " << tl_number << std::endl);
          TrafficLight& tl = traffic_lights[tl_number];

          //LOUT("Final state = " << final_state << std::endl);
          tl.observe_state(final_state);
          tl_data.state = tl.getState();
          //LOUT("Final state copied = " << tl_data.state << std::endl);


          if (tl_data.state != none) {
            Mat tl_pos;
            double distance = 0;
            bool found_correct_ellipse = false;
            for (unsigned int i=0; i<detectedEllipses.size(); i++) {
                if (detectedEllipses[i].color == tl_data.state) {
                    tl_pos = detectedEllipses[i].world_coords;
                    distance = detectedEllipses[i].distance;
                    found_correct_ellipse = true;

                    double u = detectedEllipses[i].box.center.x;
                    LOUT("Distance of TL to picture boundary: " << min (u, 659 - u) << " pixels \n");
                    break;
                }
            }
            if (found_correct_ellipse) {
                LOUT("tl_pos = "<<tl_pos <<"\n");

                traffic_lights[tl_number].update_position(tl_pos, distance, state);
                tl_data.position = traffic_lights[tl_number].get_position();
            } else {
                LOUT("Error in tl_data.state/finding the correct ellipse \n");
            }

          } else {
            tl_data.position = Vec(0,0);
          }

          //LOUT("Write tl_data to blackboard, state = " << tl_data.state <<std::endl);
          BBOARD->setTrafficLight(tl_data);


          // Plot measured position as red dot in AnicarViewer
          std::stringstream pos;
          pos << "thick red dot "
              << tl_data.position.x << " " << tl_data.position.y << std::endl;
          BBOARD->addPlotCommand(pos.str());

          // Plot estimated positions of traffic lights (dark blue cross)
          for (int i = 1; i <= 4; ++i) {
              traffic_lights[i].plot_estimate();
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
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
