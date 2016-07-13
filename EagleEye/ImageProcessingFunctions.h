#ifndef _DerWeg_IMAGEPROCESSINGFUNCTIONS_H__
#define _DerWeg_IMAGEPROCESSINGFUNCTIONS_H__

#include "../Elementary/ConfigReader.h"
#include "../Elementary/ThreadSafeLogging.h"

#include <opencv/cv.h>
#include <vector>
#include <cmath>
#include "DataObjects.h"
#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"

using namespace cv;
using namespace std;

namespace DerWeg{

/*
float median(const Mat& mat){
    std::vector<float> array;
    if (mat.isContinuous()) {
      array.assign((float*)mat.datastart, (float*)mat.dataend);
    } else {
      for (int i = 0; i < mat.rows; ++i) {
        array.insert(array.end(), (float*)mat.ptr<uchar>(i), (float*)mat.ptr<uchar>(i)+mat.cols);
      }
    }

    // Delete zeros out of array before median is computed
    for (int i = array.size() - 1; i >= 0; --i) {
        if (array[i] == 0) {
            array.erase(array.begin() + i);
        }
    }

    std::vector<float>::iterator first = array.begin();
    std::vector<float>::iterator last = array.end();
    std::vector<float>::iterator middle = first + (last - first) / 2;
    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
    float median = *middle;
    return median;
}
*/

class CoordinateTransform {
private:
    // vector for the transotion from the left camera to the stargazer
    // the z-coordinate of the stargazer is moved to the ground
    Mat left_cam_to_stargazer;
    Mat image_to_camera;
    Mat camera_to_image;

public:

    CoordinateTransform() {}

    CoordinateTransform(const ConfigReader& cfg, Mat projection_matrix) {
        camera_to_image = projection_matrix(Rect(0, 0, 3, 3)).clone();
        //LOUT("camera_to_image = " << std::endl << " " << camera_to_image << std::endl);
        image_to_camera = camera_to_image.inv();

        left_cam_to_stargazer = Mat(3, 1, CV_64FC1);
        vector<float> temp_offset;
        cfg.get("CoordinateTransform::cam_to_stargazer", temp_offset);
        left_cam_to_stargazer.at<double>(0,0) = temp_offset[0];
        left_cam_to_stargazer.at<double>(1,0) = temp_offset[1];
        left_cam_to_stargazer.at<double>(2,0) = temp_offset[2];
    }

    Mat image_to_camera_coords(double u, double v, double distance) const {
        Mat scaled_image_coords(3, 1, CV_64FC1);
        scaled_image_coords.at<double>(0,0) = distance * u;
        scaled_image_coords.at<double>(1,0) = distance * v;
        scaled_image_coords.at<double>(2,0) = distance;
        //LOUT("scaled_image_coords = " << scaled_image_coords << "\n");

        return image_to_camera * scaled_image_coords;
    }

    Mat image_to_camera_coords(double u, double v) {
        return image_to_camera_coords(u, v, 1);
    }

    // Return u, v as a pair
    pair<double,double> camera_to_image_coords(const Mat camera_coords) const {
        Mat image = camera_to_image * camera_coords;
        if (image.at<double>(2,0) == 0) {
            return pair<double,double>(0,0);
        } else {
            pair<double,double> result;
            result.first = image.at<double>(0,0) / image.at<double>(2,0);
            result.second = image.at<double>(1,0) / image.at<double>(2,0);
            return result;
        }
    }

    Mat camera_to_world_coords(const Mat cam_coords, const State state) const {
        Mat result = cam_coords.clone();
        //rotate camera coordinate system
        result.at<double>(0,0) =   cam_coords.at<double>(2,0);
        result.at<double>(1,0) = - cam_coords.at<double>(0,0);
        result.at<double>(2,0) = - cam_coords.at<double>(1,0);

        // move rotated camera coordinate system to stargazer coordinate system
        result -= left_cam_to_stargazer;

        // rotate vehicle coordintes and shift coordinate system to get x,y in world coordinates
        Vec vehicle_coords(result.at<double>(0,0), result.at<double>(1,0));
        Vec world = vehicle_coords.rotate(state.orientation) + state.sg_position;

        // z coordinate doesn't change in this transformation
        result.at<double>(0,0) = world.x;
        result.at<double>(1,0) = world.y;
        //LOUT("result = " << result << "\n");
        return result;
    }

    // Reverse Transformation
    Mat world_to_camera_coords(const Mat world_coords, const State state) const {
        Mat tmp = world_coords.clone();

        Vec world_pos(tmp.at<double>(0,0), tmp.at<double>(1,0));
        Vec vehicle_coords = (world_pos - state.sg_position).rotate(- state.orientation);

        tmp.at<double>(0,0) = vehicle_coords.x;
        tmp.at<double>(1,0) = vehicle_coords.y;

        tmp += left_cam_to_stargazer;

        Mat cam_coords = tmp.clone();
        cam_coords.at<double>(0,0) = - tmp.at<double>(1,0);
        cam_coords.at<double>(1,0) = - tmp.at<double>(2,0);
        cam_coords.at<double>(2,0) =   tmp.at<double>(0,0);
        return cam_coords;
    }

};

} //namespace

#endif

