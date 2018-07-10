#include "PylonDetection.h"
#include <cstdio>

using namespace DerWeg;

PylonDetection::PylonDetection (int i_p_u_area, int i_p_v_area, int i_p_v_shift) {
    Size_structEl_1 = cv::Size(11, 11);
    Size_structEl_2 = cv::Size(15, 15);

    d_upper = 3./10.;
    d_lower = 4./10.;
    d_height = 1. - d_upper - d_lower;
    // LOUT("up: " << d_upper << " low: " << d_lower << " height: " << d_height);

    i_u_area = i_p_u_area;
    i_v_area = i_p_v_area;
    i_v_shift = i_p_v_shift;
}

std::vector<std::vector<cv::Point>> PylonDetection::applyDetection (cv::Mat i_rectified_L) {
    PylonPointClouds.clear();

    i_proc = i_rectified_L.clone();
    i_rect = i_rectified_L.clone();

    img_Size = i_rectified_L.size();
    this->slicendice();

    // filter for color
    this->bgr2hsv();
    this->filterForRed();

    // noisecancelling
    this->closing();
    this->opening();

    // look for points
    this->gaussian();
    this->erode();
    this->findPeaks();

    //this->showImages();
    //this->saveImages();

    return PylonPointClouds;
}

void PylonDetection::bgr2hsv() {
        cv::cvtColor(i_proc, i_proc, CV_BGR2HSV);

        //i_hsv = i_proc.clone();
}

void PylonDetection::filterForRed(){
    cv::Mat lower_red, upper_red;
    cv::inRange(i_proc, cv::Scalar(2,150,8), cv::Scalar(9,255,255), i_proc);
    //cv::inRange(i_proc, cv::Scalar(160,150,100), cv::Scalar(180,255,255), upper_red);
    //cv::addWeighted(lower_red, 1.0, upper_red, 1.0, 0.0, i_proc);

    //i_red = i_proc.clone();
}

void PylonDetection::closing() {
    cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size_structEl_1);
    cv::morphologyEx(i_proc, i_proc, cv::MORPH_CLOSE, sel);

    //i_close = i_proc.clone();
}
void PylonDetection::opening() {
    cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size_structEl_1);
    cv::morphologyEx(i_proc, i_proc, cv::MORPH_OPEN, sel);

    //i_open = i_proc.clone();
}

void PylonDetection::findCanny() {
    cv::Canny(i_proc, i_proc, 100, 200, 3);

    //i_canny = i_proc.clone();
}

void PylonDetection::slicendice() {
    int s_x = 0;
    int s_y = img_Size.height*d_upper;
    int s_width = img_Size.width;
    int s_height = img_Size.height*d_height;

    img_Size.height = s_height;
    i_proc = cv::Mat(i_proc, cv::Rect(s_x, s_y, s_width, s_height)).clone();

    // LOUT(img_Size << "  " << i_proc.size() << "  [" << img_Size.width << " x " << s_height << "]\n");
}

void PylonDetection::gaussian() {
    cv::GaussianBlur(i_proc, i_proc, Size_structEl_2, 0, 0);

    //i_gauss = i_proc.clone();
}

void PylonDetection::erode() {
    cv::Mat sel = cv::getStructuringElement(cv::MORPH_ELLIPSE, Size_structEl_2);
    cv::morphologyEx(i_proc, i_proc, cv::MORPH_ERODE, sel);

    i_errode = i_proc.clone();
}

void PylonDetection::findPeaks() {
    int n = 0;
    int iEdge = 0;
    bool bClimb = false;

    for (int m = 0; m < img_Size.width; m++) {
        //LOUT((int)i_proc.at<uchar>(img_Size.height-1-n, m) << "\n");

        while((int)i_proc.at<uchar>(img_Size.height-1-n, m) != 0) {
            n++;
            iEdge = 0;
            bClimb = true;
        }

        if ((int)i_proc.at<uchar>(img_Size.height-1-n, m) == 0 && n != 0) {
            if((int)i_proc.at<uchar>(img_Size.height-n, m) != 0) {
                iEdge++;
            }
            else {
                if (bClimb && iEdge > 0) {
                    int u = m - iEdge/2 - 1;
                    int v = img_Size.height*(1 + d_upper/d_height) - n;

                    std::vector<cv::Point> newPointVector;
                    PylonPointClouds.push_back(newPointVector);

                    PylonPointClouds[PylonPointClouds.size() - 1].push_back(cv::Point(u, v)); // First point at (0 | 0)

                    for (int ui = - i_u_area; ui < i_u_area; ui++) {
                        for (int vi = 0; vi < i_v_area; vi++) {
                            if (vi != 0 && ui != 0 && n > vi && (int)i_proc.at<uchar>(img_Size.height - n + vi, u + ui) != 0) {
                                PylonPointClouds[PylonPointClouds.size() - 1].push_back(cv::Point(u + ui, v + vi + i_v_shift));
                            }
                        }
                    }

                    // LOUT("u: " << u << " | v: " << v << "\n");

                    iEdge = 0;
                    bClimb = false;
                }

                while ((int)i_proc.at<uchar>(img_Size.height-n, m) == 0 && n > 0) {
                    n --;
                }

                n++;
            }
        }
    }
}

void PylonDetection::showImages(){
    cv::Mat i_plot = i_rect.clone();

    if(PylonPointClouds.size() != 0) {
        for(int i = 0; i < PylonPointClouds.size(); i++) {
            for (int j = 0; j < PylonPointClouds[i].size(); j++) {
                cv::circle(i_plot, PylonPointClouds[i][j], 3, cv::Scalar(0, 255, 0), 3);
            }
        }
    }

    cv::imshow("Pylons", i_plot );
}

void PylonDetection::saveImages(int i_Counter){
    cv::Mat i_plot = i_rect.clone();

    if(PylonPointClouds.size() != 0) {
        for(int i = 0; i < PylonPointClouds.size(); i++) {
            cv::circle(i_plot, PylonPointClouds[i][0], 5, cv::Scalar(0, 255, 0), 2);
        }
    }

    char ch[10];
    sprintf(ch, "%03d", i_Counter);
    //std::string path = "/home/kal3_mrt219/U/Christian/images_out/";
    std::string path = "/home/kal3/source/Christian/images_out/pylons/";

    std::ostringstream imgProc_1;
    imgProc_1 << path << "SegmentedCones_" << ch << ".png";
    std::string v_i_proc_1 = imgProc_1.str();

    cv::imwrite(v_i_proc_1, i_errode);

    std::ostringstream imgProc_2;
    imgProc_2 << path << "DetectedCones_" << ch << ".png";
    std::string v_i_proc_2 = imgProc_2.str();

    cv::imwrite(v_i_proc_2, i_plot);
}
