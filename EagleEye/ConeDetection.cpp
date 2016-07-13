#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "toast2/stereo/kogmolabor.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include "ImageProcessingFunctions.h"



using namespace cv;
using namespace std;

namespace DerWeg {

    struct EdgeData{
        double sum_v;       //sum of all v-coordinates of one edge
        double sum_u;
        double n;           //number of found edge pixels

        std::vector<double> c_list; //only needed if median should be calculated

        int start;          // start point of edge (beg or end of cone top side)

        double apex_u;      // estimated cone apex u-coordinate (saved in left-edge EdgeData struct)
        double apex_v;
        double apex_disp;   // estimated disparity (only saved in left-edge and left image)
    };


  /** ConeDetection */
  class ConeDetection : public KogmoThread {
  private:
        RectImages rect_images;
        State state;

        // needed to get the projection matrix
        DerWeg::StereoGPU stereoGPU;

        std::string windowname_l;
        std::string windowname_r;
        Mat left, right, vis_left, vis_right;

        cv::Mat img[3], imgR[3];                       //arrays with H,S,V(maybe normalized V) channels of left and right stereo image
        cv::Mat out0, out1, out2, out0R,out1R,out2R, outMAP;   //output channels

        //current values, overwritten each loop
        char current_camera;                            //left 'L' or right 'R' camera image

        int height;                                     // height of region of interest for edge search
        int width;                                      // cone top side width, depending on row number
        double u, v;                                    // coordinates of the last estimated cone apex

        // variables to get from config-file:

        int Hmax, Smin,Vmin;     // HSV values for thresholding in function isOrange()
        int Hdes, Sdes, Vdes;   // HSV values of perfect cone color, used to compute difference image

        int u0, v0;          //v0= image row of camera horizont (estimated from images)
        double f;   // focal length in pixel
        double b;     // distance between cameras
        double h;      // distance cone top side to camera horizont
        double w;      // width of cone top side
        double H;        // distance cone top side to ground

        double m_cone;   //slope of cone flanks

        int v_search_beg,v_search_end,u_search_beg;   //region of interest for search of cone top side

        //minimum number of pixels to accept cone top side / // width minus tolerance
        //width_min= width_min_prop_to_width*width+width_min_const;
        double width_min_prop_to_width;
        int width_min_const;

        //width plus tolerance
        //width_max=width_max_prop_to_width*width+width_max_const;
        double width_max_prop_to_width;
        int width_max_const;

        // critical pixel distance, when to assume that pixels ahead would no more belong to the current cone
        double delta_prop_to_width;

        // min. number of edge pixels to accept as valid edge:
        // int n_min = n_min_prop_to_height *height + n_min_const ;
        double n_min_prop_to_height;
        int n_min_const;


        //half width of region where to search corresponnding cone top side in right image
        double tol_prop_to_width;
        int tol_const;
        int tol_height;

        //accepted width of cone top side in right image
        double width_min_prop_to_widthR;
        int width_min_constR;
        double width_max_prop_to_widthR;
        int width_max_constR;

        //accepted difference between v-coordinate of apex estimation in left and right image:
        //int accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
        double ac_v_diff_prop_to_height;
        int ac_v_diff_const;

        //half width of region of interest for edge detection
        int ROI_half_width;


        CoordinateTransform transformer;
        size_t frame_counter;

        // parameters for sanity check on cone positions
        double min_height_tol, max_height_tol;
        double min_x_value, max_x_value;
        double min_y_value, max_y_value;


  public:
    /** Konstruktor initialisiert den Tiefenschaetzer */
    ConeDetection () :
      stereoGPU ("/home/common/calib.txt"),
      windowname_l("Left Rectified Image"),
      windowname_r("Right Rectified Image") {
        cvNamedWindow (windowname_l.c_str(), CV_WINDOW_AUTOSIZE);
        cvNamedWindow (windowname_r.c_str(), CV_WINDOW_AUTOSIZE);
      }

    /** Destruktor */
    ~ConeDetection () {}

    void init(const ConfigReader& cfg) {
        LOUT("Exec ConeDetection init()" << std::endl);

        Hmax=20; Smin=25;Vmin=90;     // HSV values for thresholding in function isOrange()
        Hdes=7; Sdes=185; Vdes=240;   // HSV values of perfect cone color, used to compute difference image

        u0=332; //???
        v0=260;          // image row of camera horizont (estimated from images)
        b=0.1473;     // distance between cameras
        h=0.056;      // distance cone top side to camera horizont
        w=0.033;      // width of cone top side
        H=0.35;//0.3;        // distance cone top side to ground

        m_cone=5.35;   //slope of cone flanks

        v_search_beg=190;v_search_end=258;   //region of interest for search of cone top side

        //minimum number of pixels to accept cone top side / // width minus tolerance
        //width_min= width_min_prop_to_width*width+width_min_const;
        width_min_prop_to_width=0.7;
        width_min_const=0;

        //width plus tolerance
        //width_max=width_max_prop_to_width*width+width_max_const;
        width_max_prop_to_width=1.3;
        width_max_const=2;

        // critical pixel distance, when to assume that pixels ahead would no more belong to the current cone
        delta_prop_to_width=0.5;

        // min. number of edge pixels to accept as valid edge:
        // int n_min = n_min_prop_to_height *height + n_min_const ;
        n_min_prop_to_height=0.02;
        n_min_const=1;


        //half width of region where to search corresponnding cone top side in right image
        tol_prop_to_width=2;
        tol_const=30;
        tol_height=25;

        //accepted width of cone top side in right image
        width_min_prop_to_widthR=1;
        width_min_constR=0;
        width_max_prop_to_widthR=1.3;
        width_max_constR=2;




        //accepted difference between v-coordinates of apex estimation in left and right image:
        //int accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
        ac_v_diff_prop_to_height=0.05;
        ac_v_diff_const=1;

        //half width of region of interest for edge detection
        ROI_half_width=7;  //(int)(5+0.5*width);

        Mat projection_matrix;
        stereoGPU.getProjectionMatrix(projection_matrix);
        f = projection_matrix.at<double>(0,0);

        transformer = CoordinateTransform(cfg, projection_matrix);

        frame_counter = 0;

        cfg.get("ConeDetection::min_height_tol", min_height_tol);
        cfg.get("ConeDetection::max_height_tol", max_height_tol);
        cfg.get("ConeDetection::min_x_value", min_x_value);
        cfg.get("ConeDetection::max_x_value", max_x_value);
        cfg.get("ConeDetection::min_y_value", min_y_value);
        cfg.get("ConeDetection::max_y_value", max_y_value);
    }

    /*
    Mat getThresholded(const Mat& input) {
        // Region of Interest
        Rect roi(0, 0, inout.rows, input.cols);
        Mat im_roi = input(roi);


        // Blur and HSV
        int median_filter_size = 3;
        medianBlur(im_roi, im_roi, median_filter_size);
        Mat im_hsv;
        cvtColor(im_roi, im_hsv, COLOR_BGR2HSV);


        // Color thresholing
        Mat hue_range;
        int hue_low = 0
        int hue_high = 20;
        int sat_low = 25;
        int sat_high = 255;
        int val_low = 90;
        int val_high = 255;

        inRange(im_hsv, cv::Scalar(hue_low, sat_low, val_low),
                cv::Scalar(hue_high, sat_high, val_high), hue_range);


        // Opening/Closing
        int erode_size = 2;
        int dilate_size = 2;
        Mat element_erode = getStructuringElement(MORPH_ELLIPSE,
                                                  Size(2*erode_size + 1, 2*erode_size+1),
                                                  Point(-1, -1));
        Mat element_dilate = getStructuringElement(MORPH_ELLIPSE,
                                                   Size(2*dilate_size + 1, 2*dilate_size+1),
                                                   Point(-1, -1));
        // Opening
        erode(hue_range, hue_range, element_erode);
        dilate(hue_range, hue_range, element_dilate);

        return hue_range;
    }
    */

//======function to show results=========================================================

void show_res(const cv::Mat& ch012, const cv::Mat& ch0, const cv::Mat& ch1, const cv::Mat& ch2, const std::string windowname){
    cv::Mat to_show [3];
    double b=0.5;
    to_show[0]=ch0+b*ch012;
    to_show[1]=ch1+b*ch012;
    to_show[2]=ch2+b*ch012;
    cv::Mat img_to_show;
    cv::merge(to_show,3,img_to_show);
    cv::imshow(windowname, img_to_show);
}

//======function to test wheather pixel is orange (only used for searching top sides, not for edge detection)=====

bool isOrange(int H, int S, int V){
    return ( H<=Hmax && S>=Smin && V>=Vmin);
}


//=====function to create image with isOrange function (only for visualization)========
void test_isOrange (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
    img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
    for (int i=0; i<img_in[0].rows;i++) {
        const uchar* H_in=img_in[0].ptr<uchar>(i);
        const uchar* S_in=img_in[1].ptr<uchar>(i);
        const uchar* V_in=img_in[2].ptr<uchar>(i);
        uchar* D_out=img_out.ptr<uchar>(i);
        for (int j=0; j<img_in[0].cols;j++) {
            *D_out=255*isOrange(H_in[j],S_in[j],V_in[j]);
            D_out++; // move pointer forwards
        }
    }
}

//======function to test wheather a possible cone top side in this place would be hidden by a previously detected cone

bool isHidden(int u, int v, std::vector<int> beg,  std::vector<int> end,   std::vector<int> row){
    for (size_t i=0; i<beg.size();i++) {
        if(u>= (beg[i]-1/m_cone*(v-row[i]) ) && u<= ( end[i]+1/m_cone*(v-row[i]))  ){
            return 1;
        }
    }
    return 0;
}


//======function to get a scalar value from H,S,V values, defined by the differences to the desired orange color=========

int diff_to_Orange(int H, int S, int V){
    if (H<30 && V>20 && S>20) {
        return (255- std::min(255, abs(H-Hdes) +  abs(S-Sdes) + abs(V-Vdes) ) );
    }
    else {
        return 0;
    }
}

//=====function to create image with diff_to_Orange function (only for visualization)========
void test_diff_to_Orange (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
    img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
    for (int i=0; i<img_in[0].rows;i++) {
        const uchar* H_in=img_in[0].ptr<uchar>(i);
        const uchar* S_in=img_in[1].ptr<uchar>(i);
        const uchar* V_in=img_in[2].ptr<uchar>(i);
        uchar* D_out=img_out.ptr<uchar>(i);
        for (int j=0; j<img_in[0].cols;j++) {
            *D_out=diff_to_Orange(H_in[j],S_in[j],V_in[j]);
            D_out++; // move pointer forwards
        }
    }
}

//=====function to create image with threshold o  diff_to_Orange function (only for visualization)========
void test_diff_to_Orange_threshold (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
    img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
    for (int i=0; i<img_in[0].rows;i++) {
        const uchar* H_in=img_in[0].ptr<uchar>(i);
        const uchar* S_in=img_in[1].ptr<uchar>(i);
        const uchar* V_in=img_in[2].ptr<uchar>(i);
        uchar* D_out=img_out.ptr<uchar>(i);
        for (int j=0; j<img_in[0].cols;j++) {
            if(diff_to_Orange(H_in[j],S_in[j],V_in[j])>50){
                *D_out=255;
            }
            D_out++; // move pointer forwards
        }
    }
}

//======function to compute a length in image pixels based on the length in meter and the distance of the object. This distance is defined by the quotient of a known length in world and the corresponding pixel number in the image  ====================

int meter2pixel(double length, double ref_world, int ref_image) {
    //e.g. ref_world = height over horizont, ref_image=v0-v with row number v, and image horizont v0
    return std::round(length *ref_image/ref_world);
}

double pixel2meter(int pixel, double ref_world, int ref_image) {
    return pixel * ref_world/ref_image;
}


//========== visualize where cone in the right camera image is expected to be
void visualizeDispExpectation(int beg, int end, int i) {
    if (current_camera=='L') {

        uchar* output2R=out2R.ptr<uchar>(i);            //pointer to i-th row of third out-channel

        int disp=meter2pixel(b,h,v0-i);          //expected disparity

        int j0=beg-disp;
        if (j0>0) {
            output2R+=j0;
            for (int j=j0; j<beg;j++) {
                *output2R=255;
                output2R++;
            }
        }
    }
}

//calculate cone position in vehicle fixed coordinate system from estimated apex column and disparity
void determinePosition(const EdgeData & EDl){
    // f/x=disp/b => x=f*b/disp
    double distance = pixel2meter(f,b,EDl.apex_disp) * 1000;
    // u/y=disp/b

    if (distance < 0) {
        EOUT("Drop cone measurement: distance can not be negative\n");
    }
    else {
        double y=pixel2meter(-(EDl.apex_u-u0),b,EDl.apex_disp);
        //std::cout<<"found cone at x="<<x<<" y="<<y<<std::endl;

        cv::Mat camera_coords = transformer.image_to_camera_coords(EDl.apex_u, EDl.apex_v, distance);
        cv::Mat world_coords = transformer.camera_to_world_coords(camera_coords, state);

        if (world_coords.at<double>(2,0) < min_height_tol || world_coords.at<double>(2,0) > max_height_tol) {
            EOUT("Drop cone measurement: peak height out of bounds\n");
        }
        else if (world_coords.at<double>(0,0) < min_x_value || world_coords.at<double>(0,0) > max_x_value ||
            world_coords.at<double>(1,0) < min_y_value || world_coords.at<double>(1,0) > max_y_value) {
            EOUT("Drop cone measurement: position not within area\n");
        }
        else {
            PylonMeasurement pm;
            pm.position = Vec(world_coords.at<double>(0,0), world_coords.at<double>(1,0));
            pm.distance = distance;
            Vec diff = pm.position - state.sg_position;
            pm.view_angle = std::atan2(diff.y, diff.x);
            pm.frame_number = frame_counter;

            BBOARD->addPylonMeasurement(pm);

            //visualize:
            cv::circle(outMAP,cv::Point(250-100*y,500-100*distance),15,255,1,8); //img, center, radius, color, thickness
            std::stringstream coordinate_sstream;
            coordinate_sstream<<distance<<"|"<<y;
            std::string coordinate_string;
            coordinate_sstream>>coordinate_string;
            cv::putText(outMAP,coordinate_string, cv::Point(250-100*y,500-100*distance),cv::FONT_HERSHEY_SIMPLEX,0.5,255,1,8);
        }
    }
}

/////////////////////////////////////////////////////////////////////////


//=========function to  estimate cone apex ================================================

void estimateApexMono(const char camera, EdgeData& EDl,EdgeData& EDr){

    // either least squares / arithmetic mean value
    double cl=(EDl.sum_v+m_cone*EDl.sum_u)/EDl.n;
    double cr=(EDr.sum_v-m_cone*EDr.sum_u)/EDr.n;

    // or median
    // int nl=EDl.c_list.size()/2;
    // std::nth_element(EDl.c_list.begin(), EDl.c_list.begin()+nl, EDl.c_list.end());
    // int nr=EDr.c_list.size()/2;
    // std::nth_element(EDr.c_list.begin(), EDr.c_list.begin()+nr, EDr.c_list.end());

    u=(cl-cr)/(2*m_cone);
    v=(cl+cr)/2;
    int uc= std::min(std::max(0,(int)u),img[0].cols); // pixel number
    int vc=std::min(std::max(0,(int)v),img[0].rows);

    EDl.apex_u=u;
    EDl.apex_v=v;

    if (camera=='L') {
        out2.at<uchar>(vc,uc)=255; out2.at<uchar>(vc-1,uc)=255;out2.at<uchar>(vc+1,uc)=255;out2.at<uchar>(vc,uc-1)=255;out2.at<uchar>(vc,uc+1)=255;                                                                                 //PLOT point in channel 2
        cv::line(out2, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
        cv::line(out2, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
    }
    else {
        out2R.at<uchar>(vc,uc)=255; out2R.at<uchar>(vc-1,uc)=255;out2R.at<uchar>(vc+1,uc)=255;out2R.at<uchar>(vc,uc-1)=255;out2R.at<uchar>(vc,uc+1)=255;
        cv::line(out2R, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
        cv::line(out2R, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
    }
    // std::cout<<"estimated apex in u= "<<u<<"; v= "<<v << " based on " <<EDl.n << " left and "<<EDr.n<<" right pixels in a ROI with a height of "<<height<<" pixels" <<std::endl;

}


//============= estimate apex from both stereo cameras with least squares: estimated parameters are uK,vK (column and row of apex in left camera image and disparity b_
void estimateApexStereo(EdgeData& EDl, EdgeData& EDr, EdgeData& EDlR, EdgeData& EDrR){
    estimateApexMono('L', EDl,EDr) ;
    estimateApexMono('R', EDlR,EDrR);
    EDl.apex_disp=EDl.apex_u-EDlR.apex_u;
}


/////////////////////////////////////////////////////////////////////////


//=====core function =======================================================================

void detect (char camera){

    std::vector<cv::Point3d> peak_image_positions;

    if (camera=='S') {
        current_camera='L';
    }
    else {
        current_camera=camera; //set global variable to current camera left (L) or right(R)
    }

    //------local variables---------------------------------------------------------------
    int beg, end;                                 // begin and end of current truncated cone top side
    int delta;                                    // critical pixel distance, when to assume that pixels ahead would no more belong to the current cone
    int width_min;                                // width minus tolerance
    int width_max;                                // width plus tolerance
    std::vector<int> hidden_beg, hidden_end, hidden_row;     //vectors with left point of cone top side, with corresponding right point and with the row number of the detected cone top side
    EdgeData EDl, EDr, EDlR,EDrR; // e.g. right edge (r) in right camera (R)
    bool valid_right_cone;

    int n_min; // min number of edge pixels to accept edge
    int accepted_v_diff; //accepted difference between v-coordinate of apex estimation

    const uchar* currentH;
    const uchar* currentS;
    const uchar* currentV;  //pointer to i-th row of ...-channel
    uchar* output0;
    uchar* output1;
    uchar* output2;

    // plot horizont
    cv::line(out2, cv::Point(0,v0),cv::Point(img[0].cols,v0),100); //PLOT line in channel 2

    //------iterate through rows----------------------------------------------------------
    for (int i=v_search_beg; i<v_search_end;i++) {


    //set pointers to i-th row
    if(camera=='L' || camera=='S'){
      currentH=img[0].ptr<uchar>(i); //pointer to i-th row of H-channel
      currentS=img[1].ptr<uchar>(i); //pointer to i-th row of S-channel
      currentV=img[2].ptr<uchar>(i); //pointer to i-th row of V-channel
      output0=out0.ptr<uchar>(i);            //pointer to i-th row of first out-channel
      output1=out1.ptr<uchar>(i);            //pointer to i-th row of second out-channel
      output2=out2.ptr<uchar>(i);            //pointer to i-th row of third out-channel
    }
    else{
      currentH=imgR[0].ptr<uchar>(i); //pointer to i-th row of H-channel
      currentS=imgR[1].ptr<uchar>(i); //pointer to i-th row of S-channel
      currentV=imgR[2].ptr<uchar>(i); //pointer to i-th row of V-channel
      output0=out0R.ptr<uchar>(i);            //pointer to i-th row of first out-channel
      output1=out1R.ptr<uchar>(i);            //pointer to i-th row of second out-channel
      output2=out2R.ptr<uchar>(i);            //pointer to i-th row of third out-channel
    }


    //optional: left rand not to scan (because not in right stereo image available, or because of edges from rectification)
    u_search_beg=30;
    output0+=u_search_beg;                       //move pointer to column
    output1+=u_search_beg;
    output2+=u_search_beg;

    //initialize beg and end counter
    beg=-2;                            // -2 means that no cone begin was detected, and there must be a non-orange pixel in front of a valid cone (in order to have no cone begin next to the left rand or next to a hidden region
    end=-2;                            // no active cone

    //evaluate row-dependent parameters
    width=meter2pixel(w,h,v0-i);  // width of top side if truncated cone top side  would be in this row.
    width_min= width_min_prop_to_width*width+width_min_const;
    width_max=width_max_prop_to_width*width+width_max_const; //width_max=1.3*width+2;
    delta= delta_prop_to_width*width;        //critical distance to last orange pixel

    height=meter2pixel(H,h,v0-i); //// height of region of interest, dependent on row, global variable !!!


    //----iterate through columns---------------------------------------------------------------------
    for (int j=u_search_beg; j<img[0].cols-1; j++){

        if (isHidden(j,i,hidden_beg,hidden_end,hidden_row)) {
            //PLOT hidden regions in Channel 0
            *output0+=100;
        }

        //......either pixel is orange...................................................................
        if (isOrange(currentH[j],currentS[j],currentV[j])) {

            //either  new cone
            if(beg==-1){
                beg=j;
                end=j;
            }
            //or not relevant: cone with top side here would be hidden by already detected cone
            else if (isHidden(j,i,hidden_beg,hidden_end,hidden_row)) {
                beg=-2;
                end=-2;
            }
            //no active cone, and there was no non-orange pixel between the left rand or a hidden region and the current pixel => nothing to do
            else if(beg==-2 || end==-2){
                // do nothing
            }
            //or  pixel belongs to current cone
            else{
                end=j;
            }

            //PLOT all orange Pixels in ROI in Channel 1
            *output1=100;
        }
        //........or pixel is not orange...............................................................
        else {

            //******* critical distance to last orange pixel  => cone finished **************************
            if (j-end==delta) {

                //::::::::::::: either suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
                if (end-beg>=width_min && end-beg<=width_max && beg!=u_search_beg) {

                    //plot line from cone begin to cone end
                    for (int z=beg; z<=end; z++) {
                        *(output1-j+z)+=150;
                    }

                    // define hidden region
                    hidden_beg.push_back(beg-10);  //10
                    hidden_end.push_back(end+10);
                    hidden_row.push_back(i);

                    EDl.start=beg;
                    EDr.start=end;

                    //detect edges
                    detectEdge(i,-1,current_camera, EDl);      //detect left edge
                    detectEdge(i,+1,current_camera, EDr);      //detect right edge


                    n_min= n_min_prop_to_height *height + n_min_const ; //min. number of edge pixels to accept as valid edge

                    if (EDl.n>n_min && EDr.n>n_min) {

                        //estimate cone apex from left or right camera image
                        if (camera!='S') {
                            estimateApexMono(current_camera,EDl,EDr);
                            // visualize expected disparity
                            visualizeDispExpectation(beg, end, i);
                        }

                        //if camera mode is 'S' (Stereo): estimate cone apex and disparity from both images
                        if (camera=='S') {

                            // search for cone in right stereo image based on the current result in left camera image
                            valid_right_cone=0;
                            searchRightImage(beg, end, i, EDlR, EDrR, valid_right_cone);

                            //if corresponding right cone top side found
                            if (valid_right_cone ==1) {

                                //search for left and right cone edge
                                detectEdge(i,-1,'R', EDlR);
                                detectEdge(i,1,'R', EDrR);

                                //if enough edge pixels found
                                if (EDlR.n>n_min && EDrR.n>n_min) {

                                    //estimate apex in left and right image
                                    estimateApexStereo(EDl,EDr,EDlR,EDrR);

                                    //if not too large difference in apex height: calculate x,y (vehicle-fixed coordinate system) coordinates
                                    accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
                                    if(std::abs( EDl.apex_v-EDlR.apex_v)<accepted_v_diff){
                                        cv::line(out2, cv::Point(EDl.apex_u,EDl.apex_v),cv::Point(EDlR.apex_u,EDlR.apex_v),100);

                                        determinePosition(EDl);
                                    }
                                    // else{
                                    // 	std::cout<<"vertical distance of apex estimations: "<< std::abs( EDl.apex_v-EDlR.apex_v)<<" > accepted distance "<< accepted_v_diff <<std::endl;
                                    // }
                                }
                                // else{
                                // 		std::cout<<"not enough edge pixel found in right image: n_min= "<<n_min<< " [right cam - left flank]: "<< EDlR.n<<" [right cam - right flank]: "<<EDrR.n<<std::endl;
                                // }

                            }

                        }

                    }
                    // else{
                    // 	std::cout<<"not enough edge pixel found in left image: n_min= "<<n_min<< " [left cam,  left flank]: "<< EDl.n<<" [left cam, right flank]: "<<EDr.n<<std::endl;
                    // }

                }


                //::::::::::: or no suitable width  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                //   => nothing to do


                //cone finished => reset beg and end counter to not active (-1)
                beg=-1;
                end=-1;
            }

            //*************else:  critical distance to last orange pixel not reached *********************
            // non-orange pixel detected => can remove "-2" blocker, because there has been a non orange pixel after the rand or hidden region.
            if (beg==-2 || end==-2) {
                beg=-1;
                end=-1;
            }



        }

        //plot width
        if (j==200) {
            *output1=100;
            *(output1-width)=100;
        }

        output0++; //move pointer to next column
        output1++;
        output2++;
        }
    }
}


/////////////////////////////////////////////////////////////////////////


//======== function to detect edge in region of interest defined by corner coordinates (row,col) of the cone =====================

void detectEdge(int row, int left_right, char cam, EdgeData& EdgeData_res){     //left edge (col=beg) => left_right=-1 <->  right edge (col=end) => left_right=1

    int col= EdgeData_res.start; //set column number: start point of edge (=begin or end point of cone top side)

    //initialize counter
    EdgeData_res.sum_u=0;
    EdgeData_res.sum_v=0;
    EdgeData_res.n=0;

    //declare pointers
    const uchar* EdgeH;
    const uchar* EdgeS;
    const uchar* EdgeV;
    uchar* EdgeOut;


    //----------iterate through rows------------------------------------------
    for (int iE=row; iE<std::min(img[0].rows, row+height);iE++) {

        if(cam=='L'){
            EdgeH=img[0].ptr<uchar>(iE); //pointer to i-th row
            EdgeS=img[1].ptr<uchar>(iE);
            EdgeV=img[2].ptr<uchar>(iE);
            EdgeOut=out0.ptr<uchar>(iE);
        }
        else{
            EdgeH=imgR[0].ptr<uchar>(iE); //pointer to i-th row
            EdgeS=imgR[1].ptr<uchar>(iE);
            EdgeV=imgR[2].ptr<uchar>(iE);
            EdgeOut=out0R.ptr<uchar>(iE);   //define channel where to plot ROI and detected pixels is channel 0
        }


        int jEdge=col+left_right*(iE-row)*1/m_cone;      //ROI center: column number dependent on row number ( line starting in col (col=left or right cone top side vertex)).

        int jE0=std::max(1,jEdge-ROI_half_width);        //where to begin (a few  points in front of the ROI center)
        EdgeOut+=jE0;                                    //move pointer from first column in the current row to ROI begin

        //--------------iterate through columns------------------------------------------
        for (int jE=jE0; jE<std::min(jEdge+ROI_half_width, img[0].cols-20); jE++) { //consider filter mask width in what to choose as minimum column number
            //std::cout<<"jE "<<jE<<std::endl;

            *EdgeOut+=50;                                  //PLOT all considered pixels (=region of interest) in Edge Channel

            if( //left edge and filter mask [-2,1] with threshold 40
            (left_right==-1 &&
             std::max(0, diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE]) - 2*diff_to_Orange(EdgeH[jE-1],EdgeS[jE-1],EdgeV[jE-1])) >40) ||
            //or right edge and filter mask [1,-2] with threshold 40
            (left_right==+1 &&
             std::max(0, diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE]) - 2*diff_to_Orange(EdgeH[jE+1],EdgeS[jE+1],EdgeV[jE+1])) >40)) {
                EdgeData_res.sum_u+=jE;
                EdgeData_res.sum_v+=iE;
                EdgeData_res.n++;

                EdgeData_res.c_list.push_back(iE+left_right*m_cone*jE); //y-offset of line with m_cone going through point u=jE, v=iE; add to list (only needed if median should be calculated)

                *EdgeOut=255; //PLOT detected edge pixels
            }
            // :::::::::::: or no edge point ::::::::::::::::::::::
            // nothing to do


            // move pointer to next column
            EdgeOut++;
        }

    }
}


/////////////////////////////////////////////////////////////////////////

//========== search in right stereo image for corresponding cone
void searchRightImage(int beg, int end, int i, EdgeData &EDlR, EdgeData & EDrR, bool & valid_right_cone) {
    //search only in current row; begin in beg-disp-tolerance (see visualizeDispExpectation() ), search for orange region with suitable with (see detect() )
    const uchar* currentH; //pointer to i-th row of H-channel
    const uchar* currentS; //pointer to i-th row of S-channel
    const uchar* currentV; //pointer to i-th row of V-channel
    //uchar* output2R;            //pointer to i-th row of third out-channel
    uchar* output1R;            //pointer to i-th row of third out-channel
    uchar* output0R;            //pointer to i-th row of third out-channel
    int widthR=end-beg;
    int disp=meter2pixel(b,h,v0-i);        //expected disparity
    int j0=beg-disp;

    //int delta=0.5*widthR;

    int tol=tol_prop_to_width*widthR+tol_const;//2*widthR+30; // half width of region where to search
    // int width_min=0.5*(end-beg)-1;
    //int width_max=1.5*(end-beg)+1;

    //evaluate row-dependent parameters
    int width=meter2pixel(w,h,v0-i);  // width of top side if truncated cone top side  would be in this row.
    int width_min=width_min_prop_to_widthR*width+width_min_constR;//width; //0.7*width-2;
    int width_max=width_max_prop_to_widthR*width+width_max_constR;//1.3*width+2;
    int delta=delta_prop_to_width*width;        //critical distance to last orange pixel

    //j0>0

    bool done=0;
    int v_cur=i-2;

    while (done==0 && v_cur-i<tol_height) {
        currentH=imgR[0].ptr<uchar>(v_cur); //pointer to i-th row of H-channel
        currentS=imgR[1].ptr<uchar>(v_cur); //pointer to i-th row of S-channel
        currentV=imgR[2].ptr<uchar>(v_cur); //pointer to i-th row of V-channel
        //output2R=out2R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel
        output1R=out1R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel
        output0R=out0R.ptr<uchar>(v_cur);            //pointer to i-th row of third out-channel

        v_cur++;

        int beg_r=-3; // -3 means, that the current pixel could be a inner pixel of the cone
        int beg_l=-2;
        int end_r=-2;
        int end_l=-2;
        int j_r=j0;
        int j_l=j0;

        for (int j_rel=0;j_rel<tol;j_rel++) {
            j_r=j0+j_rel;

            *(output0R+j_r)+=100;  //PLOT

            //right
            if (isOrange(currentH[j_r],currentS[j_r],currentV[j_r])) {

                // either inner point ->  first search for left flank of cone, save as beg_r
                if (beg_r==-3) {
                    while (1) {
                        //*(output1R+j_l)+=100; //PLOT

                        //std::cout<<"begr "<<beg_r<<" j_l "<<j_l<<std::endl;

                        if (isOrange(currentH[j_l],currentS[j_l],currentV[j_l])) {
                            beg_r=j_l;
                        }
                        else if (beg_r-j_l > delta) {
                            break;
                        }

                        if (j0-j_l>tol || j_l==1) {
                            beg_r=-2; //
                            break;
                        }

                        j_l--;
                    }
                }

                //or  new cone
                if (beg_r==-1) {
                    beg_r=j_r;
                    end_r=j_r;
                }
                //or  pixel belongs to current cone
                else {
                    end_r=j_r;
                }
                //*(output1R+j_r)=100;                                                  //PLOT all orange Pixels in ROI in Channel 2R
            }

            //........or pixel is not orange...............................................................
            else {
                // std::cout<<"j0 "<<j0<<" jr "<<j_r<<" begr "<<beg_r<<" endr "<<end_r<<std::endl;
                // *(output1R+j_r)=100;
                //******* critical distance to last orange pixel  => cone finished **************************
                if (j_r-end_r==delta && end_r>0) {

                    //width>width_max => probably hidden
                    if (end_r-beg_r>width_max) {
                        done=1;
                        std::cout<<"R: width>width_max => probably hidden"<<std::endl;
                        break;
                    }
                    //::::::::::::: or suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
                    else if (end_r-beg_r>=width_min && end_r-beg_r<=width_max) {
                        //   for (int z=beg_r; z<=end_r; z++){*(output1R+z)+=150;}
                        //std::cout<<"RESULT:"<<beg_r<<", "<<end_r<<std::endl;
                        for (int z=beg_r; z<=end_r; z++){*(output0R+z)=255;*(output1R+z)=255;}//*(output2R+z)=255;}
                        done=1;
                        EDlR.start=beg_r;
                        EDrR.start=end_r;
                        valid_right_cone=1;
                        break;
                    }

                    beg_r=-1;
                    end_r=-1;
                }

                if (beg_r==-3 || beg_r==-2) {
                    beg_r=-1;
                    end_r=-1;
                }
            }

            //left
            if (j_l>j0-j_rel) {
                j_l=j0-j_rel;
                *(output1R+j_l)+=100;  //PLOT

                if (isOrange(currentH[j_l],currentS[j_l],currentV[j_l])) {
                    //either new cone
                    if (beg_l==-1) {
                        beg_l=j_l;
                        end_l=j_l;
                    }
                    //or  pixel belongs to current cone
                    else {
                        beg_l=j_l;
                    }
                    //PLOT all orange Pixels in ROI in Channel 2R
                    //*(output1R+j_l)=100;
                }

                //........or pixel is not orange...............................................................
                else {

                    //******* critical distance to last orange pixel  => cone finished **************************
                    if (beg_l-j_l==delta && beg_l>0) {

                        // std::cout<<" endl "<<end_l<<" begl "<<beg_l<<"breite "<<end_l-beg_l<<" min "<< width_min<<" max "<<width_max<<std::endl;
                        //width>width_max => probably hidden
                        if (end_l-beg_l>width_max) {
                            done=1;
                            std::cout<<"R: width>width_max => probably hidden"<<std::endl;
                            break;
                        }
                        //::::::::::::: or suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
                        else if (end_l-beg_l>=width_min && end_l-beg_l<=width_max) {
                            //   for (int z=beg_l; z<=end_l; z++){*(output1R+z)+=150;}
                            //std::cout<<"RESULT:"<<beg_l<<", "<<end_l<<std::endl;
                            for (int z=beg_l; z<=end_l; z++) {
                                *(output0R+z)=255;
                                *(output1R+z)=255;
                            }
                            done=1;
                            EDlR.start=beg_l;
                            EDrR.start=end_l;
                            valid_right_cone=1;
                            break;
                        }

                        beg_l=-1;
                        end_l=-1;
                    }

                    if ( beg_l==-2) {
                        beg_l=-1;
                        end_l=-1;
                    }
                }

            }

        }
    }
}


/////////////////////////////////////////////////////////////////////////

    void execute () {
        LOUT("Enter ConeDetection execute()\n");
        try{
            while (true) {
                BBOARD->waitForRectImages();
                rect_images = BBOARD->getRectImages();

                left = rect_images.images.image;
                right = rect_images.images.image_right;
                state = rect_images.state;

                //initialize output channels (needed to define size equal to img size)
                out0 =cv::Mat::zeros(img[0].size(),img[0].type());
                out1 =cv::Mat::zeros(img[0].size(),img[0].type());
                out2 =cv::Mat::zeros(img[0].size(),img[0].type());
                out0R =cv::Mat::zeros(img[0].size(),img[0].type());
                out1R =cv::Mat::zeros(img[0].size(),img[0].type());
                out2R =cv::Mat::zeros(img[0].size(),img[0].type());

                Mat img0 = left;
                Mat img0R = right;


                //convert to HSV and split channels => create global img and imgR Mat-arrays
                cv::Mat imgHSV;
                cv::cvtColor(img0,imgHSV,CV_BGR2HSV);
                cv::split(imgHSV,img);
                cv::cvtColor(img0R,imgHSV,CV_BGR2HSV);
                cv::split(imgHSV,imgR);

                cv::Mat imgDiff, imgDiffR, imgZero;
                test_diff_to_Orange(imgDiff,img);
                test_diff_to_Orange(imgDiffR,imgR);
                imgZero=cv::Mat::zeros(img[0].size(),img[0].type());
                //show_res(imgZero,imgDiff,imgDiffR,imgZero, "window_diff");


                // detect cones
                //detect('L');
                //show_res(imgDiff,out0,out1,out2,"window_result");
                //detect('R');
                //show_res(imgDiffR,out0R,out1R,out2R,"window_resultR");
                //show_res(imgDiff,imgDiffR,out2,out2R,"window_Stereo");

                out0 =cv::Mat::zeros(img[0].size(),img[0].type());
                out1 =cv::Mat::zeros(img[0].size(),img[0].type());
                out2 =cv::Mat::zeros(img[0].size(),img[0].type());
                out0R =cv::Mat::zeros(img[0].size(),img[0].type());
                out1R =cv::Mat::zeros(img[0].size(),img[0].type());
                out2R =cv::Mat::zeros(img[0].size(),img[0].type());

                detect('S');
                //show_res(imgIsOrangeR,out0R,out1R,out2R,"window_resultRS");
                //show_res(imgDiffR,out0R,out1R,out2R,"window_resultStereoR_diff");
                //show_res(imgDiff,out0,out1,out2,"window_resultStereoL_diff");
                show_res(imgDiff,imgDiffR,out2,out2R,"window_resultStereoLR_diff");

                //cv::imshow("MAP", outMAP);

                frame_counter++;

                boost::this_thread::sleep(boost::posix_time::milliseconds(20));
                boost::this_thread::interruption_point();
            }
        }catch(boost::thread_interrupted&){;}
    }
  };


} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ConeDetection> application_save_stereo_camera_image ("ConeDetection");

}
