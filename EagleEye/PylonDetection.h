#ifndef _DerWeg_w8ing_PylonDetection_H_
#define _DerWeg_w8ing_PylonDetection_H_

#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "StereoTools.h"
#include "toast2/stereo/kogmolabor.h"

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace DerWeg {

    class PylonDetection {

    public:

        PylonDetection (int i_p_u_area, int i_p_v_area, int i_p_v_shift);

        std::vector<std::vector<cv::Point>> applyDetection (cv::Mat i_rectified_L);
        void bgr2hsv ();
        void filterForRed ();
        void closing ();
        void opening ();
        void slicendice ();
        void gaussian ();
        void erode ();
        void findPeaks ();

        void findCanny ();

        void showImages ();
        void saveImages (int i_Counter);

    private:
    cv::Mat i_rect,
            i_hsv,
            i_red,
            i_open,
            i_close,
            i_canny,
            i_gauss,
            i_errode,
            i_proc;

    cv::Size Size_structEl_1, Size_structEl_2;
    cv::Size img_Size;
    double d_upper, d_lower, d_height;

    int i_u_area,
        i_v_area,
        i_v_shift;

    std::vector<std::vector<cv::Point>> PylonPointClouds;

    };

}; // namespace

#endif
