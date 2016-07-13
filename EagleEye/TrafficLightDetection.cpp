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
            std::string windowname;

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
            double max_fitting_error;

            double max_size_ratio;
            double min_bbox_hw_ratio, max_bbox_hw_ratio;
            int max_ellipse_size;


        public:
            void init(const ConfigReader& cfg) {
                windowname = "Preprocessing";
                cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);

                int roi_border_top, roi_border_bot, roi_width;
                cfg.get("TrafficLightDetection::roi_border_top", roi_border_top);
                cfg.get("TrafficLightDetection::roi_border_bot", roi_border_bot);
                cfg.get("TrafficLightDetection::roi_width", roi_width);
                roi = Rect(0, roi_border_top, roi_width, roi_border_bot - roi_border_top);

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
                cfg.get("TrafficLightDetection::max_fitting_error", max_fitting_error);

                cfg.get("TrafficLightDetection::max_size_ratio", max_size_ratio);
                cfg.get("TrafficLightDetection::min_bbox_hw_ratio", min_bbox_hw_ratio);
                cfg.get("TrafficLightDetection::max_bbox_hw_ratio", max_bbox_hw_ratio);
                cfg.get("TrafficLightDetection::max_ellipse_size", max_ellipse_size);
            }

            void extractEllipsesFromContour(EllipseVector& ellipse_vec, const vector<vector<Point> > & contours, const TrafficLightState c) {
                for(size_t i = 0; i < contours.size(); i++){
                    int count = contours[i].size();
                    if( count < min_contour_count || count > max_contour_count)
                    continue;

                    Mat pointsf;
                    Mat(contours[i]).convertTo(pointsf, CV_32F);
                    RotatedRect box = fitEllipse(pointsf);

                    double fitting_error = ellipseFittingError(box , contours[i]);
                    //LOUT("Ellipse fitting error = " << fitting_error << "\n");

                    if (fitting_error < max_fitting_error) {
                        // shift back the cutoff from region of interest
                        box.center.y += roi.y;

                        ellipse_vec.push_back(DetectedEllipse(box, c));
                    }
                }
            }

            double ellipseFittingError(RotatedRect box, const vector<Point>& contour) {
                double err = 0;
                for (size_t i=0; i < contour.size(); i++) {
                    Point p = contour[i];
                    double a = min(box.size.width, box.size.height)/2;
                    double b = max(box.size.width, box.size.height)/2;
                    err += abs( pow( (p.x - box.center.x)/a , 2) + pow( (p.y - box.center.y)/b , 2) - 1);
                }
                return err/contour.size();
            }


            EllipseVector detect(const cv::Mat& im_input) {
                LOUT("im_input.cols = " << im_input.cols << "\n");
                LOUT("im_input.rows = " << im_input.rows << "\n");

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
                // Opening
                erode(red_hue_range, red_hue_range, element_erode);
                dilate(red_hue_range, red_hue_range, element_dilate);
                // für green nur erode, ansonsten werden die Blätter von Baum vergrößt
                erode(green_hue_range, green_hue_range, element_erode);


                // Show filter results
                cv::imshow (windowname.c_str(), red_hue_range);
                cv::imshow ("green_thresholding", green_hue_range);



                // Get contour points (changes input image)
                vector<vector<Point> > contours_red;
                vector<vector<Point> > contours_green;
                findContours(red_hue_range, contours_red, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
                findContours(green_hue_range, contours_green, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);


                // Fit ellipses from contours
                EllipseVector detected_ellipses;
                // red
                extractEllipsesFromContour(detected_ellipses, contours_red, red);
                //green
                extractEllipsesFromContour(detected_ellipses, contours_green, green);


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


                LOUT("Detected ellipses count = " << detected_ellipses.size() << endl);


                return detected_ellipses;
          }
    };









  /** TrafficLightDetection */
  class TrafficLightDetection : public KogmoThread {
  private:
    RectImages rect_images;
    State state;
    ReferenceTrajectory rt;

    // needed to get the projection matrix
    DerWeg::StereoGPU stereoGPU;

    std::string windowname;
    std::string windowname2;
    Mat left, right, vis_left, vis_right;

    EllipseDetector ellipse_detector;
    CoordinateTransform transformer;

    double max_shape_diff;

    double red_height, yellow_height, green_height, height_tol, light_diameter;
    double camera_height, v0, focus_length, base_length;

    map<int, TrafficLight> traffic_lights;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    TrafficLightDetection () :
        stereoGPU("/home/common/calib.txt"),  // needed to get the projection matrix
        windowname("Processed Image"), windowname2("Right Image") {
            cvNamedWindow (windowname.c_str(), CV_WINDOW_AUTOSIZE);
            cvNamedWindow (windowname2.c_str(), CV_WINDOW_AUTOSIZE);
      }

    /** Destruktor */
    ~TrafficLightDetection () {}

    void init(const ConfigReader& cfg) {

        LOUT("Exec TrafficLightDetection init()" << std::endl);

        ellipse_detector.init(cfg);

        cfg.get("TrafficLightDetection::max_shape_diff", max_shape_diff);
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


    void findMatchingEllipses(const DetectedEllipse& ellipse, double expected_height, const EllipseVector& ellipses_right, vector<int>& potentialMatchesIndices) {
        //ellipse center
        float u = ellipse.box.center.x;
        float v = ellipse.box.center.y;

        double approx_distance = (expected_height - camera_height) * focus_length / (v0 - v);
        double approx_disparity = focus_length * base_length / approx_distance;
        double tolerance = std::pow(focus_length * light_diameter/2 / approx_distance, 2);

        if (approx_distance < 0 ) {
            return;
        }

        LOUT("Approx distance = " << approx_distance << "\n");
        LOUT("Approx disparity = " << approx_disparity << "\n");
        circle(vis_right, Point(u - approx_disparity, v), std::sqrt(tolerance), Scalar(255,0,0));


        for (int j = ellipses_right.size() - 1; j >= 0; --j) {
            if (ellipses_right[j].color == ellipse.color &&
                std::pow(u - approx_disparity - ellipses_right[j].box.center.x, 2) +
                std::pow(v - ellipses_right[j].box.center.y, 2) <= tolerance) {
                // Found matching ellipse in tolerance area
                potentialMatchesIndices.push_back(j);
            }
        }
    }

    // Returns disparity
    // returns 0 if no matching ellipse was found
    // Also deletes the matching ellipse out of right_ellipses!!
    double matchEllipse(const DetectedEllipse& ellipse, EllipseVector& right_ellipses) {
        // draw into image
        cv::ellipse(vis_left, ellipse.box, Scalar(0,255,255));

        // Get all indices of ellipses in right image within tolerance window
        vector<int> potentialMatchesIndices;
        if (ellipse.color == red) {
            findMatchingEllipses(ellipse, red_height, right_ellipses, potentialMatchesIndices);
            findMatchingEllipses(ellipse, yellow_height, right_ellipses, potentialMatchesIndices);
        } else if (ellipse.color == green) {
            findMatchingEllipses(ellipse, green_height, right_ellipses, potentialMatchesIndices);
        } else {
            //Error!
            EOUT("Unknown ellipse color " << ellipse.color << "\n");
        }

        // Find ellipse with the closest matching shape which is in a close row of the image
        double min_shape_difference = 1000000000;
        int arg_min = -1; // shape difference minimizing index in right_ellipses
        for (size_t i=0; i<potentialMatchesIndices.size(); i++) {
            DetectedEllipse& r_ellipse = right_ellipses[potentialMatchesIndices[i]];
            double shape_diff = pow( (ellipse.box.center.y - r_ellipse.box.center.y), 2)
                            + pow( (ellipse.box.boundingRect().height - r_ellipse.box.boundingRect().height), 2)
                            + pow( (ellipse.box.boundingRect().width - r_ellipse.box.boundingRect().width), 2);
            if (shape_diff < min_shape_difference || arg_min < 0) {
                arg_min = potentialMatchesIndices[i];
                min_shape_difference = shape_diff;
            }
        }

        LOUT("Shape difference "<< min_shape_difference << "\n");

        if (arg_min >= 0 && min_shape_difference < max_shape_diff) {
            float u = ellipse.box.center.x;
            double disparity = u - right_ellipses[arg_min].box.center.x;
            right_ellipses.erase(right_ellipses.begin() + arg_min);
            return disparity;
        } else {
            // if no potential matches were found, return 0
            return 0;
        }
    }

    // Decision logic if there are seen many ellipses
    // returns index of ellipse that should be used
    // returns negative value if the traffic light state is none
    // (should not occur since this method is only called on ellipse vectors with nonvanishing size)
    int processValidatedEllipses(const EllipseVector& ellipses) {

        int count_red_ellipses = 0, count_green_ellipses = 0;
        for (unsigned int i=0; i < ellipses.size(); i++) {
            //LOUT("ellipse " << i << " world coordinates: "<< detectedEllipses[i].world_coords << std::endl);
            if (ellipses[i].color == red) {
                count_red_ellipses++;
            } else if (ellipses[i].color == green) {
                count_green_ellipses++;
            }
        }
        LOUT("Red ellipses : "<< count_red_ellipses << ", green ellipses: "<<count_green_ellipses<<"\n");

        // Interpret color frequencies for final state decision
        TrafficLightState final_state;
        if (count_red_ellipses == 0 && count_green_ellipses == 0) {
            final_state = none;
            return -1;
        } else if (count_red_ellipses == 0) {
            final_state = green;
        } else {
            final_state = red;
        }

        for (unsigned int i=0; i<ellipses.size(); i++) {
            if (ellipses[i].color == final_state) {
                double u = ellipses[i].box.center.x;
                LOUT("Distance of TL to picture boundary: " << min (u, 659 - u) << " pixels \n");
                return i;
            }
        }
        EOUT("Error in processValidatedEllipses/finding the correct ellipse \n");
        return -1;

    } // end processValidatedEllipses


    void execute () {
        LOUT("Enter TLD execute()\n");
        BBOARD->waitForReferenceTrajectory();
      try{
        while (true) {
          LOUT("TL DETECTION EXECUTE LOOP \n");

          BBOARD->waitForRectImages();
          rect_images = BBOARD->getRectImages();

          left = rect_images.images.image;
          right = rect_images.images.image_right;
          state = rect_images.state;
          rt = rect_images.reference_trajectory;

          vis_left = left.clone();
          vis_right = right.clone();


          //==================================================================
          // Get the traffic light we see next

          int tl_number;
          if (rt.path.behind_intersec) {
            tl_number = (rt.segment_id % 10);
          } else {
            tl_number = (rt.segment_id / 10);
          }
          //LOUT("tl_number = " << tl_number << std::endl);
          TrafficLight& tl = traffic_lights[tl_number];

          // Get position and covariance of curren traffic light
          Vec tl_est_pos = tl.get_position();
          Vec tl_est_stddev = tl.get_stddev();
          double stddev = max(tl_est_stddev.x, tl_est_stddev.y);


          //==================================================================
          // DETECTION

          EllipseVector ellipses_left = ellipse_detector.detect(left);
          EllipseVector ellipses_right = ellipse_detector.detect(right);
          EllipseVector unmatched_left;


          //==================================================================
          // Matching and Validation by calculated height

          for (int i = ellipses_left.size() - 1; i >= 0; i--) {
            DetectedEllipse& ellipse = ellipses_left[i];

            // draw into image
            cv::ellipse(vis_left, ellipse.box, Scalar(0,255,255));

            double disparity = matchEllipse(ellipse, ellipses_right);

            if (disparity == 0) {
                ellipses_left.erase(ellipses_left.begin() + i);
                unmatched_left.push_back(ellipse);
                continue;
            }

            //ellipse center
            float u = ellipse.box.center.x;
            float v = ellipse.box.center.y;

            // Distance in Millimetres
            double distance = focus_length * base_length / disparity;
            ellipse.distance = distance;

            // Transform image to world coordinates
            Mat camera_coords = transformer.image_to_camera_coords(u, v, distance);
            Mat world_coords = transformer.camera_to_world_coords(camera_coords, state);
            ellipse.world_coords = world_coords;

            // Debugging depth information
            LOUT("Distance = "<<distance<<endl);
            //LOUT("image_coords = " << std::endl << " " << image_coords << std::endl);
            //LOUT("camera_coords = " << std::endl << " " << camera_coords << std::endl);

            // erase ellipses if their height does not match the expected height
            double height = world_coords.at<double>(2,0);
            if (ellipse.color == red &&
                    !( abs(height - red_height) < height_tol || abs(height - yellow_height) < height_tol )
                ) {
                   ellipses_left.erase(ellipses_left.begin() + i);
                   LOUT("RED Ellipse kicked out because of height tolerance\n");
                   continue;
            } else if (ellipse.color == green &&
                    !( abs(height - green_height) < height_tol)
                ) {
                   ellipses_left.erase(ellipses_left.begin() + i);
                   LOUT("GREEN Ellipse kicked out because of height tolerance\n");
                   continue;
            }

            Vec ellipse_pos(ellipse.world_coords.at<double>(0,0), ellipse.world_coords.at<double>(1,0));

            //TODO: Diese Bedingung verbessern, Kovarianzmatrix nutzen!
            // Als parameter umschreiben
            if( ( (tl_est_pos - ellipse_pos).length() > 1500 && stddev < 500 )
               || ellipse_pos.x < 0 || ellipse_pos.y < 0 || ellipse_pos.x > 12000 || ellipse_pos.y > 6000 ) {
                ellipses_left.erase(ellipses_left.begin() + i);
               LOUT("Ellipse kicked out because of wrong position\n");
               continue;
            }

          } // End of validation loop

          // Show results of ellipse fitting
          cv::imshow (windowname.c_str(), vis_left);
          cv::imshow (windowname2.c_str(), vis_right);


            // At this point there are three sets of ellipses:
            // Left ellipses contains matched ellipses including distance and world coordinates
            // Right ellipses contains the unmatched right ellipses
            // unmatched left contains unmatched ellipses from the left image

          //===================================================================
          // Choose an ellipse for final state and position of observed traffic light
          TrafficLightState final_state;

          // If there is an ellipse which is in both images, use this one
          if (ellipses_left.size() > 0) {
            int ellipse_index = processValidatedEllipses(ellipses_left);
            if (ellipse_index < 0) {
                final_state = none;
            } else {
                final_state = ellipses_left[ellipse_index].color;
                // update position estimate
                Mat world_pos = ellipses_left[ellipse_index].world_coords;
                Vec tl_pos = Vec(world_pos.at<double>(0,0),world_pos.at<double>(1,0));
                double distance = ellipses_left[ellipse_index].distance;
                traffic_lights[tl_number].update_position(tl_pos, distance, state);
            }
          }
          // If there are only ellipses which occur in only either one of the images,
          // use the so far known position of the traffic light
          // But only if the covanriance is already small enough
          //TODO als parameter
          else if (unmatched_left.size() > 0 && (tl_est_pos - state.sg_position).length() < 1000 && stddev < 500 ) {
            int ellipse_index = processValidatedEllipses(unmatched_left);
            if (ellipse_index < 0) {
                final_state = none;
            } else {
                final_state = unmatched_left[ellipse_index].color;
            }
          }
          // TOdo als parameter
          else if (ellipses_right.size() > 0 && (tl_est_pos - state.sg_position).length() < 1000 && stddev < 500 ) {
            int ellipse_index = processValidatedEllipses(ellipses_right);
            if (ellipse_index < 0) {
                final_state = none;
            } else {
                final_state = ellipses_right[ellipse_index].color;
            }
          } else {
            final_state = none;
          }


          //===================================================================
          // Fill tl_data object and write it to blackboard

          TrafficLightData tl_data;
          //LOUT("Final state = " << final_state << std::endl);
          tl.observe_state(final_state);

          tl_data.state = tl.getState();
          //LOUT("Final state copied = " << tl_data.state << std::endl);
          tl_data.position = traffic_lights[tl_number].get_position();

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

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
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
