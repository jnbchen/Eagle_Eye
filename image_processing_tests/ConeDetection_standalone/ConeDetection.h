#ifndef CONEDETECTION_H
#define CONEDETECTION_H

#include "opencv2/opencv.hpp"
#include <iostream>

class ConeDetection{

  
 private:


  cv::Mat img[3], imgR[3];                       //arrays with H,S,V(maybe normalized V) channels of left and right stereo image
  cv::Mat out0, out1, out2, out0R,out1R,out2R, outMAP;   //output channels
  //current values, overwritten each loop
  char current_camera;                            //left 'L' or right 'R' camera image
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


  //accepted difference between v-coordinate of apex estimation in left and right image:
  //int accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
  double ac_v_diff_prop_to_height;
  int ac_v_diff_const;

  //half width of region of interest for edge detection
  int ROI_half_width;

    

 
 public:
 
  void init();

  void show_res(const cv::Mat& ch012, const cv::Mat& ch0, const cv::Mat& ch1, const cv::Mat& ch2, const std::string windowname);

  bool isOrange(int H, int S, int V);

  void test_isOrange (cv::Mat& img_out, const cv::Mat (& img_in) [3]);

  bool isHidden(int u, int v, std::vector<int> beg,  std::vector<int> end,   std::vector<int> row);
  
  int diff_to_Orange(int H, int S, int V);

  void test_diff_to_Orange (cv::Mat& img_out, const cv::Mat (& img_in) [3]);

  void test_diff_to_Orange_threshold (cv::Mat& img_out, const cv::Mat (& img_in) [3]);

  int meter2pixel(double length, double ref_world, int ref_image);
  double pixel2meter(int pixel, double ref_world, int ref_image);

  void detectEdge(int row, int left_right, char cam, EdgeData& EdgeData_res);

  void estimateApexMono(const char camera, EdgeData& EDl,EdgeData& EDr);

  void visualizeDispExpectation(int beg, int end, int i);

  void searchRightImage(int beg, int end, int i, EdgeData &EDlR, EdgeData & EDrR, bool & valid_right_cone);

  void estimateApexStereo(EdgeData& EDl, EdgeData& EDr, EdgeData& EDlR, EdgeData& EDrR);

  void determinePosition(const EdgeData & EDl);

  void detect (char camera);

  void execute(int argc, char** argv);

  
};
  
#endif
