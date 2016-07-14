//erfordert:    g++ `pkg-config --cflags opencv` thresholding_values.cpp -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d

//Aufruf z.B. mit ./a.out PATHNAME/*rect.png
//=> Code wird fuer alle Bilder ausgefuehrt, die auf rect.png enden
//Bildwechsel bei Tastendruck

#include<iostream>
#include<sstream>
#include"opencv2/opencv.hpp"


int H_min(255), S_min(255), V_min(255),  H_max(255), S_max(255), V_max(255);
cv::Mat img1;
cv::Mat img0;
cv::Mat imgH, imgS,imgV;
cv::Mat img1_ch[3];

//Notiz: H<18, S>33, V>85
//H<24, S>23, V>91

void HSV_channels0(){
    cv::Mat imgHSV;
    cv::cvtColor(img0,imgHSV,CV_BGR2HSV);
    cv::Mat imgHSV_ch[3];
    cv::split(imgHSV,imgHSV_ch);
    imgH= imgHSV_ch[0];
    imgS= imgHSV_ch[1];
    imgV= imgHSV_ch[2];
}


void treshold (const cv::Mat& img_in, cv::Mat& img_out,int min, int max){
  img_out.create(img_in.size(),CV_8UC1);
  for(int i=0; i<img_in.rows; i++){
      const uchar* current = img_in.ptr<uchar>(i);
      uchar* output=img_out.ptr<uchar>(i);
      for(int j=0;j<img_in.cols;j++){
	if(current[j]<max && current[j]>=min){
	  *output=255;
	}
	else{*output=0;}
	*output++;
      }
    }

}

void show_merged_image(){
    treshold(imgH,img1_ch[0],H_min,H_max);
  treshold(imgS,img1_ch[1],S_min,S_max);
  treshold(imgV,img1_ch[2],V_min,V_max);
  cv::merge(img1_ch,3,img1);
  cv::imshow("window1",img1);
}

void trackbar_callback (int, void*){ //what to do when trackbar is moved
  show_merged_image();
}


///////////////////////////////////////

int main (int argc, char** argv){

  for (int i=1; i<argc; i++){

    std::cout<<"nr."<<i<<" name "<<argv[i]<<std::endl;

    //read left image
    img0 = cv::imread(argv[i],1); //read image
    cv::imshow("window0",img0);

    //create corresponding right image filename with suffix _right
    std::string filename = argv[i];
    std::string filenameR = filename.substr(0, filename.length()-4);
    std::stringstream filenameR_stream;
    filenameR_stream<<filenameR<<"_right.png"; //add suffix to filename

    //read right image
    cv::Mat img0R=cv::imread(filenameR_stream.str(),1);
    cv::imshow("window0R",img0R);

    //convert left image to HSV
    cv::Mat imgHSV;
    cv::cvtColor(img0,imgHSV,CV_BGR2HSV);
    cv::Mat imgHSV_ch[3];
    cv::split(imgHSV,imgHSV_ch);
    cv::imshow("windowH",imgHSV_ch[0]);
    cv::imshow("windowS",imgHSV_ch[1]);
    cv::imshow("windowV",imgHSV_ch[2]);

    //convert right image to HSV
    cv::Mat imgHSVR;
    cv::cvtColor(img0R,imgHSVR,CV_BGR2HSV); //convert to HSV
    cv::Mat imgHSV_chR[3];
    cv::split(imgHSVR,imgHSV_chR);


    //search for relevant points
    cv::namedWindow("window1",1);
    HSV_channels0();
    show_merged_image();  
    cv::createTrackbar("H min","window1",&H_min, 255, trackbar_callback);
    cv::createTrackbar("H max","window1",&H_max, 256, trackbar_callback); 
    cv::createTrackbar("S min","window1",&S_min, 255, trackbar_callback);
    cv::createTrackbar("S max","window1",&S_max, 256, trackbar_callback);
    cv::createTrackbar("V min","window1",&V_min, 255, trackbar_callback);
    cv::createTrackbar("V max","window1",&V_max, 256, trackbar_callback);

    std::cout<<"Hmax"<<H_max<<std::endl;

    cv::Mat img_res;
    img_res.create(img0.size(),CV_8UC1);
    for(int i=0; i<img0.rows; i++){
      const uchar* currentH = imgHSV_ch[0].ptr<uchar>(i);
      const uchar* currentS = imgHSV_ch[1].ptr<uchar>(i);
      const uchar* currentV = imgHSV_ch[2].ptr<uchar>(i);
      uchar* output=img_res.ptr<uchar>(i);
      for(int j=0;j<img0.cols;j++){
	if(currentH[j]<H_max && currentH[j]>=H_min && currentS[j]<S_max && currentS[j]>=S_min  && currentV[j]<V_max && currentV[j]>=V_min){
	  *output=255;
	}
	else{*output=0;}
	*output++;
      }
    }
    cv::imshow("window_res",img_res);

    cv::waitKey(0);
  }
  return 0;
}

   
