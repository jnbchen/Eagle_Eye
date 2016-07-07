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
#include <string>


using namespace cv;
using namespace std;

namespace DerWeg {

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
    Mat img[3], imgR[3];

    //CoordinateTransform transformer;

    //double camera_height, v0, focus_length, base_length;


    cv::Mat out0, out1, out2, out0R,out1R,out2R;   //output channels

    //current values, overwritten each loop
    char current_camera;                            //left 'L' or right 'R' camera image
    double sum_vl, sum_ul, nl, sum_vr, sum_ur, nr;  // variables needed for apex estimation
    std::vector <double> apex_u, apex_v, apexR_u, apexR_v;            // vector with estimated apex coordinates
    int height;                                     // height of region of interest for edge search
    int width;                                      // cone top side width, depending on row number
    double u, v;                                    // coordinates of the last estimated cone apex

    // variables to get from config-file:
    int Hmax, Smin,Vmin;     // HSV values for thresholding in function isOrange()
    int Hdes, Sdes, Vdes;   // HSV values of perfect cone color, used to compute difference image

    int v0;          // image row of camera horizont (estimated from images)
    double f;   // focal length in pixel
    double b;     // distance between cameras
    double h;      // distance cone top side to camera horizont
    double w;      // width of cone top side
    double H;        // distance cone top side to ground

    double m_cone;   //slope of cone flanks

    int v_search_beg, v_search_end, u_search_beg;   //region of interest for search of cone top side



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

//        cfg.get("TrafficLightDetection::base_length", base_length);


        // variables to get from config-file:
        Hmax=20;
        Smin=25;
        Vmin=90;
        Hdes=7;
        Sdes=185;
        Vdes=240;

        v0=260;
        f=574.3259;
        b=0.1473;     // distance between cameras
        h=0.056;      // distance cone top side to camera horizont
        w=0.033;      // width of cone top side
        H=0.3;        // distance cone top side to ground

        m_cone = 5.35;   //slope of cone flanks

        v_search_beg = 190;
        v_search_end = 258;
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


    //======function to test wheather a possible cone top side in this place would be hidden by a previously detected cone
    bool isHidden(int u, int v, std::vector<int> beg,  std::vector<int> end,   std::vector<int> row){
        for(size_t i=0; i<beg.size();i++){
            if(u>= (beg[i]-1/m_cone*(v-row[i]) ) && u<= ( end[i]+1/m_cone*(v-row[i]))  ){
                return 1;
            }
        }
        return 0;
    }

    //======function to get a scalar value from H,S,V values, defined by the differences to the desired orange color=========
    int diff_to_Orange(int H, int S, int V){
        if (H<30 && V>20 && S>20){
            return (   255- std::min( 255 , abs(H-Hdes) +  abs( S-Sdes ) + abs(V-Vdes)   )       );
        }
        return 0;
    }

    //=====function to create image with diff_to_Orange function (only for visualization)========
    void test_diff_to_Orange (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
        img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
        for(int i=0; i<img_in[0].rows;i++){
            const uchar* H_in=img_in[0].ptr<uchar>(i);
            const uchar* S_in=img_in[1].ptr<uchar>(i);
            const uchar* V_in=img_in[2].ptr<uchar>(i);
            uchar* D_out=img_out.ptr<uchar>(i);
            for(int j=0; j<img_in[0].cols;j++){
                *D_out=diff_to_Orange(H_in[j],S_in[j],V_in[j]);
                D_out++; // move pointer forwards
            }
        }
    }

    //======function to compute a length in image pixels based on the length in meter and the distance of the object. This distance is defined by the quotient of a known length in world and the corresponding pixel number in the image  ====================
    int meter2pixel(double length, double ref_world, int ref_image){
        //e.g. ref_world = height over horizont, ref_image=v0-v with row number v, and image horizont v0
        return length *ref_image/ref_world;
    }

    //======== function to detect edge in region of interest defined by corner coordinates (row,col) of the cone =====================
    void detectEdge(int row, int col, int left_right, char cam){     //left edge (col=beg) => left_right=-1 <->  right edge (col=end) => left_right=1

        //initialize counters for apex estimation
        if(left_right==-1){
            sum_ul=0; //sum of column number / u-coordinates
            sum_vl=0; //sum of row numbers / v-coordinates
            nl=0; //number of detected edge pixels
        }
        else{
            sum_ur=0;
            sum_vr=0;
            nr=0;
        }


        // height of region of interest, dependent on row
        height=meter2pixel(H,h,v0-row); //global variable !

        const uchar* EdgeH; //pointer to i-th row
        const uchar* EdgeS;
        const uchar* EdgeV;
        uchar* EdgeOut;

        //----------iterate through rows------------------------------------------
        for(int iE=row; iE<std::min(img[0].rows, row+height);iE++){

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




            int jEdge=col+left_right*(iE-row)*1/m_cone;      //first edge estimation starting in cone top side vertex: column number dependent on row number (line equation)
            EdgeOut+=std::max(1,jEdge-(int)(5+0.5*width));                               //move pointer from first column in the current row to a few  points in front of the edge estimation
            //--------------iterate through columns------------------------------------------
            for(int jE=std::max(1,jEdge-(int)(5+0.5*width)); jE<std::min(jEdge+(int)(5+0.5*width), img[0].cols-20); jE++){ //consider filter mask width in what to choose as minimum column number

                *EdgeOut+=50;                                  //PLOT all considered pixels (=region of interest) in Edge Channel

                //*********left cone edge**********************************************
                if(left_right==-1 ){
                //::::::::::: either found edge point ::::::::::::::::::::::::::::::::::::::::::::::
                    if(std::max( 0,  diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE])- 2*diff_to_Orange(EdgeH[jE-1],EdgeS[jE-1],EdgeV[jE-1]) )     >40   ){   // filter mask = [-2,1]; threshold=40
                        *EdgeOut+=200;                             //PLOT detected edge pixel in  bright color
                        //add to counters for apex estimation
                        sum_ul+=jE;
                        sum_vl+=iE;
                        nl++;
                    }
                    // :::::::::::: or no edge point ::::::::::::::::::::::
                    // nothing to do
                }

                //**************right cone edge************************************************
                else {
                    //::::::::::: either found edge point ::::::::::::::::::::::::::::::::::::::::::::::
                    if(std::max( 0,  diff_to_Orange(EdgeH[jE],EdgeS[jE],EdgeV[jE])- 2*diff_to_Orange(EdgeH[jE+1],EdgeS[jE+1],EdgeV[jE+1]) )     >40   ){   // filter mask = [1,-2]; threshold=40
                        *EdgeOut+=200;             //PLOT pixel
                        //add to counters for apex estimation
                        sum_ur+=jE;
                        sum_vr+=iE;
                        nr++;
                    }
                    // :::::::::::: or no edge point ::::::::::::::::::::::
                    // nothing to do
                }

                // move pointer to next column
                EdgeOut++;

            }
        }
    }


    //=========function to  estimate cone apex ================================================

    void estimateApexMono(){
        //either enough edge pixel
        if (nl>0.2*height && nr>0.2*height){
            double cl=(sum_vl+m_cone*sum_ul)/nl;
            double cr=(sum_vr-m_cone*sum_ur)/nr;
            u=(cl-cr)/(2*m_cone);
            v=(cl+cr)/2;
            int uc= std::min(std::max(0,(int)u),img[0].cols); // pixel number
            int vc=std::min(std::max(0,(int)v),img[0].rows);
            if(current_camera=='L'){
                apex_u.push_back(u);
                apex_v.push_back(v);
                out2.at<uchar>(vc,uc)=255; out2.at<uchar>(vc-1,uc)=255;out2.at<uchar>(vc+1,uc)=255;out2.at<uchar>(vc,uc-1)=255;out2.at<uchar>(vc,uc+1)=255;                                                                                 //PLOT point in channel 2
                cv::line(out2, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
                cv::line(out2, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
            }
            else{
                apexR_u.push_back(u);
                apexR_v.push_back(v);
                out2R.at<uchar>(vc,uc)=255; out2R.at<uchar>(vc-1,uc)=255;out2R.at<uchar>(vc+1,uc)=255;out2R.at<uchar>(vc,uc-1)=255;out2R.at<uchar>(vc,uc+1)=255;
                cv::line(out2R, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
                cv::line(out2R, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
            }
            std::cout<<"estimated apex in u= "<<u<<"; v= "<<v << " based on " <<nl << " left and "<<nr<<" right pixels in a ROI with a height of "<<height<<" pixels" <<std::endl;
        }
        //or not enough
        else{
            std::cout<<"not enough edge pixel found"<<std::endl;
        }
    }

    //========== visualize where cone in the right camera image is expected to be
    void visualizeDispExpectation(int beg, int end, int i){
        if (current_camera=='L'){
            //const uchar* currentH=imgR[0].ptr<uchar>(i); //pointer to i-th row of H-channel
            //const uchar* currentS=imgR[1].ptr<uchar>(i); //pointer to i-th row of S-channel
            //const uchar* currentV=imgR[2].ptr<uchar>(i); //pointer to i-th row of V-channel
            uchar* output2R=out2R.ptr<uchar>(i);            //pointer to i-th row of third out-channel
            //int widthR=(int)(-2*(v-i)/m_cone);              //width of top side: delta_v=(-|+)m_cone*delta_u
            //int widthR=end-beg;
            int disp=meter2pixel(b,h,v0-i);//meter2pixel(b,w,widthR);           //expected disparity
            //int j0= (int)u-disp-widthR/2;
            int j0=beg-disp;
            if (j0>0){
                output2R+=j0;
                for (int j=j0; j<beg;j++){
                    *output2R=255;
                    output2R++;
                }
            }
        }
    }



    //========== search in right stereo image for corresponding cone
    void searchRightImage(int beg, int end, int i){
        //search only in current row; begin in beg-disp-tolerance (see visualizeDispExpectation() ), search for orange region with suitable with (see detect() )
        ;
    }

    //============= estimate apex from both stereo cameras with least squares: estimated parameters are uK,vK (column and row of apex in left camera image and disparity b_
    void estimateApexStereo(){
        //least squares result see Maple file
        ;
    }

    //calculate cone position in vehicle fixed coordinate system from estimated apex column and disparity
    void determinePosition(){
        ;
    }


    void detect (char camera){
        current_camera=camera; //set global variable to current camera left (L) or right(R) or Stereo(S)

        //------local variables---------------------------------------------------------------
        int beg, end;                                 // begin and end of current truncated cone top side
        int delta;                                    // critical pixel distance, when to assume that pixels ahead would no more belong to the current cone
        int width_min;                                // width minus tolerance
        int width_max;                                // width plus tolerance
        std::vector<int> hidden_beg, hidden_end, hidden_row;     //vectors with left point of cone top side, with corresponding right point and with the row number of the detected cone top side

        const uchar* currentH;
        const uchar* currentS;
        const uchar* currentV;  //pointer to i-th row of ...-channel
        uchar* output0;
        uchar* output1;
        uchar* output2;

        //------iterate through rows----------------------------------------------------------
        for(int i=v_search_beg; i<v_search_end;i++){

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
            beg=-1;                            // -1 means that no cone begin was detected
            end=-1;                            // no active cone

            //evaluate row-dependent parameters
            width=meter2pixel(w,h,v0-i);  // width of top side if truncated cone top side  would be in this row.
            width_min=0.7*width;
            width_max=1.3*width+2;
            delta=0.5*width;        //critical distance to last orange pixel


            //----iterate through columns---------------------------------------------------------------------
            for (int j=u_search_beg; j<img[0].cols-1; j++){

                if(isHidden(j,i,hidden_beg,hidden_end,hidden_row)){*output0+=100;} //PLOT hidden regions in Channel 0

                //......either pixel is orange...................................................................
                if( isOrange(currentH[j],currentS[j],currentV[j]) ){

                    //either  new cone
                    if(beg==-1){
                        beg=j;
                        end=j;
                    }
                    //or not relevant: cone with top side here would be hidden by already detected cone
                    else if(isHidden(j,i,hidden_beg,hidden_end,hidden_row) ){
                        beg=-1;
                        end=-1;
                    }
                    //or  pixel belongs to current cone
                    else{
                        end=j;
                    }
                    *output1=100;                                                  //PLOT all orange Pixels in ROI in Channel 1
                }

                //........or pixel is not orange...............................................................
                else {

                    //******* critical distance to last orange pixel  => cone finished **************************
                    if (j-end==delta){

                        //::::::::::::: either suitable width of detected cone => VALID CONE DETECTED:::::::::::::::
                        if(end-beg>=width_min && end-beg<=width_max && beg!=u_search_beg){

                            for (int z=beg; z<=end; z++){
                                *(output1-j+z)+=150; //plot line from cone begin to cone end
                            }

                            // define hidden region
                            hidden_beg.push_back(beg-10);
                            hidden_end.push_back(end+10);
                            hidden_row.push_back(i);

                            //detect edges
                            detectEdge(i,beg,-1,camera);      //detect left edge
                            detectEdge(i,end,+1,camera);      //detect right edge

                            //estimate cone apex
                            if(camera!='S'){
                                estimateApexMono();
                            }

                            //visualize expected disparity
                            visualizeDispExpectation(beg, end, i);

                            //camera mode 'S' (Stereo)
                            if(camera=='S'){
                                // search for cone in right stereo image based on the current result in left camera image
                                searchRightImage(beg, end, i);
                                detectEdge(i,beg,-1,'R'); //need to adapt detectEdge function to store separated variables for left and right camera image (instead of nl, nr, sum_ul, sum_ur, sum_vl, sum_vr -- create -->  nlL, nrL, sum_ulL etc. and nlR, nrR etc)
                                estimateApexStereo();
                                determinePosition();
                            }
                        }

                        //::::::::::: or no suitable width  ::::::::::::::::::::::::::::::::::::::::::::::::::::::::
                        //   => nothing to do

                        //cone finished => reset beg and end counter to not active (-1)
                        beg=-1;
                        end=-1;
                    }
                    //*************else:  critical distance to last orange pixel not reached *********************
                    //  => nothing to do
                }

                if(j==200){
                    //plot width
                    *output1=100;
                    *(output1-width)=100;
                }

                output0++; //move pointer to next column
                output1++;
                output2++;
            }
        }
    }



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
            detect('L');
            show_res(imgDiff,out0,out1,out2,"window_result");

            detect('R');
            show_res(imgDiffR,out0R,out1R,out2R,"window_resultR");

            show_res(imgDiff,imgDiffR,out2,out2R,"window_Stereo");


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
