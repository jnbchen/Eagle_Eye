#include "ConeDetection.h"
#include <sstream>



void ConeDetection::init(){
  Hmax=20; Smin=25;Vmin=90;     // HSV values for thresholding in function isOrange()
  Hdes=7; Sdes=185; Vdes=240;   // HSV values of perfect cone color, used to compute difference image

  u0=332; //???
  v0=260;          // image row of camera horizont (estimated from images)
  f=574.3259;   // focal length in pixel
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
  n_min_prop_to_height=0.05;
  n_min_const=1;

  //accepted difference between v-coordinates of apex estimation in left and right image:
  //int accepted_v_diff = ac_v_diff_prop_to_height * height+ ac_v_diff_const;
  ac_v_diff_prop_to_height=0.05;
  ac_v_diff_const=1;

  //half width of region of interest for edge detection
  ROI_half_width=7;  //(int)(5+0.5*width);
  
}



//======function to show results=========================================================

void ConeDetection::show_res(const cv::Mat& ch012, const cv::Mat& ch0, const cv::Mat& ch1, const cv::Mat& ch2, const std::string windowname){
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

bool ConeDetection::isOrange(int H, int S, int V){
  return ( H<=Hmax && S>=Smin && V>=Vmin);
}



//=====function to create image with isOrange function (only for visualization)========
void ConeDetection::test_isOrange (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
  img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
  for(int i=0; i<img_in[0].rows;i++){
    const uchar* H_in=img_in[0].ptr<uchar>(i);
    const uchar* S_in=img_in[1].ptr<uchar>(i);
    const uchar* V_in=img_in[2].ptr<uchar>(i);
    uchar* D_out=img_out.ptr<uchar>(i);
    for(int j=0; j<img_in[0].cols;j++){
      *D_out=255*isOrange(H_in[j],S_in[j],V_in[j]);
      D_out++; // move pointer forwards
    }
  }
}

//======function to test wheather a possible cone top side in this place would be hidden by a previously detected cone

bool ConeDetection::isHidden(int u, int v, std::vector<int> beg,  std::vector<int> end,   std::vector<int> row){ 
  for(int i=0; i<beg.size();i++){
    if(u>= (beg[i]-1/m_cone*(v-row[i]) ) && u<= ( end[i]+1/m_cone*(v-row[i]))  ){
      return 1;
    }
  }
  return 0;
}


//======function to get a scalar value from H,S,V values, defined by the differences to the desired orange color=========

int ConeDetection::diff_to_Orange(int H, int S, int V){
  if (H<30 && V>20 && S>20){
    return (   255- std::min( 255 , abs(H-Hdes) +  abs( S-Sdes ) + abs(V-Vdes)   )       );
  }
  return 0;
}

//=====function to create image with diff_to_Orange function (only for visualization)========
void ConeDetection::test_diff_to_Orange (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
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

//=====function to create image with threshold o  diff_to_Orange function (only for visualization)========
void ConeDetection::test_diff_to_Orange_threshold (cv::Mat& img_out, const cv::Mat (& img_in) [3]){
  img_out=cv::Mat::zeros(img_in[0].size(),img_in[0].type());
  for(int i=0; i<img_in[0].rows;i++){
    const uchar* H_in=img_in[0].ptr<uchar>(i);
    const uchar* S_in=img_in[1].ptr<uchar>(i);
    const uchar* V_in=img_in[2].ptr<uchar>(i);
    uchar* D_out=img_out.ptr<uchar>(i);
    for(int j=0; j<img_in[0].cols;j++){
      if(diff_to_Orange(H_in[j],S_in[j],V_in[j])>50){
	*D_out=255;
      }
      D_out++; // move pointer forwards
    }
  }
}


//======function to compute a length in image pixels based on the length in meter and the distance of the object. This distance is defined by the quotient of a known length in world and the corresponding pixel number in the image  ==================== 

int ConeDetection::meter2pixel(double length, double ref_world, int ref_image){
  //e.g. ref_world = height over horizont, ref_image=v0-v with row number v, and image horizont v0
  return length *ref_image/ref_world; 
}

double ConeDetection::pixel2meter(int pixel, double ref_world, int ref_image){
  return pixel * ref_world/ref_image;
}





//========== visualize where cone in the right camera image is expected to be
void ConeDetection::visualizeDispExpectation(int beg, int end, int i){
  if (current_camera=='L'){
    const uchar* currentH=imgR[0].ptr<uchar>(i); //pointer to i-th row of H-channel
    const uchar* currentS=imgR[1].ptr<uchar>(i); //pointer to i-th row of S-channel
    const uchar* currentV=imgR[2].ptr<uchar>(i); //pointer to i-th row of V-channel
    uchar* output2R=out2R.ptr<uchar>(i);            //pointer to i-th row of third out-channel
    //int widthR=(int)(-2*(v-i)/m_cone);              //width of top side: delta_v=(-|+)m_cone*delta_u
    int widthR=end-beg;
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





//calculate cone position in vehicle fixed coordinate system from estimated apex column and disparity
void ConeDetection::determinePosition(const EdgeData & EDl){
  // f/x=disp/b => x=f*b/disp
  double x=pixel2meter(f,b,EDl.apex_disp);
  // u/y=disp/b
  double y=pixel2meter(-(EDl.apex_u-u0),b,EDl.apex_disp);
  //std::cout<<"found cone at x="<<x<<" y="<<y<<std::endl;

  //visualize:
  cv::circle(outMAP,cv::Point(250-100*y,500-100*x),15,255,1,8); //img, center, radius, color, thickness
  std::stringstream coordinate_sstream;
  coordinate_sstream<<x<<"|"<<y;
  std::string coordinate_string;
  coordinate_sstream>>coordinate_string;
  cv::putText(outMAP,coordinate_string, cv::Point(250-100*y,500-100*x),cv::FONT_HERSHEY_SIMPLEX,0.5,255,1,8);
}
  








