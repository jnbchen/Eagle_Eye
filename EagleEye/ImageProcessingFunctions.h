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

float median(const Mat& mat){
    std::vector<float> array;
    if (mat.isContinuous()) {
      array.assign((float*)mat.datastart, (float*)mat.dataend);
    } else {
      for (int i = 0; i < mat.rows; ++i) {
        array.insert(array.end(), (float*)mat.ptr<uchar>(i), (float*)mat.ptr<uchar>(i)+mat.cols);
      }
    }

    std::vector<float>::iterator first = array.begin();
    std::vector<float>::iterator last = array.end();
    std::vector<float>::iterator middle = first + (last - first) / 2;
    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
    float median = *middle;
    return median;
}

class CoordinateTransform {
private:
    // vector for the transotion from the left camera to the stargazer
    // the z-coordinate of the stargazer is moved to the ground
    Mat left_cam_to_stargazer;
    Mat image_to_camera;

public:

    CoordinateTransform() {}

    CoordinateTransform(const ConfigReader& cfg, Mat projection_matrix) {
        image_to_camera = projection_matrix(Rect(0, 0, 3, 3)).clone();
        //LOUT("image_to_camera = " << std::endl << " " << image_to_camera << std::endl);
        image_to_camera = image_to_camera.inv();

        left_cam_to_stargazer = Mat(3, 1, CV_64FC1);
        vector<float> temp_offset;
        cfg.get("CoordinateTransform::cam_to_stargazer", temp_offset);
        left_cam_to_stargazer.at<double>(0,0) = temp_offset[0];
        left_cam_to_stargazer.at<double>(1,0) = temp_offset[1];
        left_cam_to_stargazer.at<double>(2,0) = temp_offset[2];
    }

    Mat image_to_camera_coords(double u, double v, double distance) {
        Mat scaled_image_coords(3, 1, CV_64FC1);
        scaled_image_coords.at<double>(0,0) = distance * u;
        scaled_image_coords.at<double>(1,0) = distance * v;
        scaled_image_coords.at<double>(2,0) = distance;
        LOUT("scaled_image_coords = " << scaled_image_coords << "\n");

        return image_to_camera * scaled_image_coords;
    }

    Mat camera_to_world_coords(Mat cam_coords, const State state) {
        Mat result = cam_coords.clone();
        //rotate camera coordinate system
        result.at<double>(0,0) =   cam_coords.at<double>(2,0);
        result.at<double>(1,0) = - cam_coords.at<double>(0,0);
        result.at<double>(2,0) = - cam_coords.at<double>(1,0);

        // move rotated camera coordinate system to stargazer coordinate system
        result -= left_cam_to_stargazer;

        // rotate vehicle coordintes and shift coordinate system to get x,y in world coordinates
        Vec vehicle_coords(result.at<double>(0,0), result.at<double>(1,0));
        Vec world = vehicle_coords.rotate(state.orientation) + state.position;

        // z coordinate doesn't change in this transformation
        result.at<double>(0,0) = world.x;
        result.at<double>(1,0) = world.y;
        LOUT("result = " << result << "\n");
        return result;
    }

};

} //namespace

#endif

