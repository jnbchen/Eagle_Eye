#include "ConeDetection.h"


//=========function to  estimate cone apex ================================================

void ConeDetection::estimateApexMono(const char camera, EdgeData& EDl,EdgeData& EDr){

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
    
  if(camera=='L'){
    out2.at<uchar>(vc,uc)=255; out2.at<uchar>(vc-1,uc)=255;out2.at<uchar>(vc+1,uc)=255;out2.at<uchar>(vc,uc-1)=255;out2.at<uchar>(vc,uc+1)=255;                                                                                 //PLOT point in channel 2
    cv::line(out2, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
    cv::line(out2, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100); //PLOT line in channel 2
  }
  else{
    out2R.at<uchar>(vc,uc)=255; out2R.at<uchar>(vc-1,uc)=255;out2R.at<uchar>(vc+1,uc)=255;out2R.at<uchar>(vc,uc-1)=255;out2R.at<uchar>(vc,uc+1)=255;
    cv::line(out2R, cv::Point(-(v+1.3*height-cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
    cv::line(out2R, cv::Point((-v+1.3*height+cl)/m_cone,v+1.3*height),cv::Point(u,v),100);
  }
  // std::cout<<"estimated apex in u= "<<u<<"; v= "<<v << " based on " <<EDl.n << " left and "<<EDr.n<<" right pixels in a ROI with a height of "<<height<<" pixels" <<std::endl;
  
}



//============= estimate apex from both stereo cameras with least squares: estimated parameters are uK,vK (column and row of apex in left camera image and disparity b_ 
void ConeDetection::estimateApexStereo(EdgeData& EDl, EdgeData& EDr, EdgeData& EDlR, EdgeData& EDrR){

  estimateApexMono('L', EDl,EDr) ;
  estimateApexMono('R', EDlR,EDrR);
  EDl.apex_disp=EDl.apex_u-EDlR.apex_u;
}
