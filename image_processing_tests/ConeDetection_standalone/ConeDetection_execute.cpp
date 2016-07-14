#include "ConeDetection.h"
#include <sstream>


void ConeDetection::execute (int argc, char** argv){

  //iterate through all files given as arguments

  for (int i=1; i<argc; i++){

    std::cout<<"nr. "<<i<<" name " <<argv[i]<<std::endl;

    //read left image
    cv::Mat img0=cv::imread(argv[i],1);
    cv::imshow("window0",img0);

    //create corresponding right image filename with suffix _right
	    std::string filename = argv[i];
	    std::string filenameR = filename.substr(0, filename.length()-4);
	    std::stringstream filenameR_stream;
	    filenameR_stream<<filenameR<<"_right.png"; //add suffix to filename

	    //read right image
	    cv::Mat img0R=cv::imread(filenameR_stream.str(),1);
    cv::imshow("window0R",img0R);

    //convert to HSV and split channels => create global img and imgR Mat-arrays
    cv::Mat imgHSV;
    cv::cvtColor(img0,imgHSV,CV_BGR2HSV);
    cv::split(imgHSV,img);
    //   cv::imshow("imgH",img[0]);
    //   cv::imshow("imgS",img[1]);
    //   cv::imshow("imgV",img[2]);
    cv::cvtColor(img0R,imgHSV,CV_BGR2HSV);
    cv::split(imgHSV,imgR);

    //initialize output channels (needed to define size equal to img size)
    out0 =cv::Mat::zeros(img[0].size(),img[0].type());
    out1 =cv::Mat::zeros(img[0].size(),img[0].type());
    out2 =cv::Mat::zeros(img[0].size(),img[0].type());
    out0R =cv::Mat::zeros(img[0].size(),img[0].type());
    out1R =cv::Mat::zeros(img[0].size(),img[0].type());
    out2R =cv::Mat::zeros(img[0].size(),img[0].type());
    outMAP=cv::Mat::zeros(500,500,CV_8UC1);


    cv::Mat imgDiff, imgDiffR, imgZero;
    test_diff_to_Orange(imgDiff,img);
    test_diff_to_Orange(imgDiffR,imgR);
    imgZero=cv::Mat::zeros(img[0].size(),img[0].type());
    //show_res(imgZero,imgDiff,imgDiffR,imgZero, "window_diff");

    cv::Mat imgIsOrange, imgIsOrangeR;
    test_isOrange (imgIsOrange,img);
    test_isOrange (imgIsOrangeR,imgR);
    show_res(imgZero,imgIsOrange,imgIsOrangeR,imgZero,"window_isOrange");

    // cv::Mat imgDiffThreshold, imgDiffThresholdR;
    // test_diff_to_Orange_threshold(imgDiffThreshold,img);
    // test_diff_to_Orange_threshold(imgDiffThresholdR,imgR);
    // show_res(imgZero,imgDiffThreshold,imgDiffThresholdR,imgZero, "window_diff_threshold");

    // detect cones
    detect('L');
    show_res(imgDiff,out0,out1,out2,"window_resultL");
	
    detect('R');
    show_res(imgDiffR,out0R,out1R,out2R,"window_resultR");

    show_res(imgDiff,imgDiffR,out2,out2R,"window_resultLR");

    out0 =cv::Mat::zeros(img[0].size(),img[0].type());
    out1 =cv::Mat::zeros(img[0].size(),img[0].type());
    out2 =cv::Mat::zeros(img[0].size(),img[0].type());
    out0R =cv::Mat::zeros(img[0].size(),img[0].type());
    out1R =cv::Mat::zeros(img[0].size(),img[0].type());
    out2R =cv::Mat::zeros(img[0].size(),img[0].type());
    detect('S');
    //show_res(imgIsOrangeR,out0R,out1R,out2R,"window_resultRS");
    show_res(imgDiffR,out0R,out1R,out2R,"window_resultStereoR_diff");
    show_res(imgDiff,out0,out1,out2,"window_resultStereoL_diff");
    show_res(imgDiff,imgDiffR,out2,out2R,"window_resultStereoLR_diff");

//cv::imshow("diff_Test_left_img", imgDiff);
//cv::imshow("diff_Test_right_img", imgDiffR);

    cv::imshow("MAP", outMAP);

    //wait until key press
    cv::waitKey(0);
  }
}
