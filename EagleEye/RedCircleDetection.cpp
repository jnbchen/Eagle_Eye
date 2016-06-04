#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <string>
#include <vector>

int main(int argc, char *argv[]){
    cv::Mat image;
    cv::Mat im_in;
    cv::Mat hsv_in;
    cv::Mat bgr_in;
    
    if (argc<2){
        image = cv::imread("/Users/chenjinbo/Downloads/BildR000074.png");
    }
    if (argc==2){
        image=cv::imread((argv[1]));
    }
    if (argc>2){
        std::cout<<"Error! Too many arguments!"<<std::endl;
    }
    if (image.empty()){
        std::cout << "error detected. something went wrong with opening the image. is it empty? exiting"<<std::endl;
        return -1;
    }
    cv::Mat orig_image = image.clone();
    
    // Image cutting orig: 659x493;
    CvRect rect;
    rect.x=0,rect.y=140,rect.width=659,rect.height=333;
    im_in = image(rect);
    
    // blur+cvt
    cv::medianBlur(im_in, im_in, 3);
    cv::cvtColor(im_in, hsv_in, cv::COLOR_BGR2HSV);
    cv::namedWindow("Original Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Orginal Image", im_in);
    cv::namedWindow("Orginal Image converted to HSV", cv::WINDOW_AUTOSIZE);
    cv::imshow("Original Image converted to HSV", hsv_in);
    
    // threshold
    cv::Mat lower_red_hue_range;
    cv::Mat upper_red_hue_range;
    cv::Mat green_hue_image;
    cv::Mat yellow_hue_image;
    
    //cv::Mat lower_green_hue_range;
    //cv::Mat upper_green_hue_range;
    //cv::Mat lower_yellow_hue_range;
    //cv::Mat upper_yellow_hue_range;
    
    cv::inRange(hsv_in, cv::Scalar(0,100,200), cv::Scalar(3,255,255), lower_red_hue_range);
    cv::inRange(hsv_in, cv::Scalar(160,100,200),cv::Scalar(179, 255, 255), upper_red_hue_range);
    cv::inRange(hsv_in, cv::Scalar(35,100,200), cv::Scalar(77,255,255), green_hue_image);
    cv::inRange(hsv_in, cv::Scalar(26,100,200), cv::Scalar(34,255,255), yellow_hue_image);
    //cv::inRange(hsv_in, cv::Scalar(25,100,100), cv::Scalar(35,255,255), lower_green_hue_range);
    //cv::inRange(hsv_in, cv::Scalar(70,100,100),cv::Scalar(80,255,255), upper_green_hue_range);
    //cv::inRange(hsv_in, cv::Scalar(15,100,100),cv::Scalar(26,255,255),lower_yellow_hue_range);
    //cv::inRange(hsv_in,cv::Scalar(30,100,100),cv::Scalar(35,255,255),upper_yellow_hue_range);
    
    //combining the above
    cv::Mat red_hue_image=im_in.clone();
    cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);
    cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9,9), 2, 2);
    cv::GaussianBlur(green_hue_image, green_hue_image, cv::Size(9,9),2,2);
    cv::GaussianBlur(yellow_hue_image, yellow_hue_image, cv::Size(9,9),2,2);
    
    // Use the Hough transform to detect circles in the combined threshold image
    std::vector<cv::Vec3f> circles_red; //red
    cv::HoughCircles(red_hue_image, circles_red, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);
    
    std::vector<cv::Vec3f> circles_green;//green
    cv::HoughCircles(green_hue_image, circles_green, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);
    
    std::vector<cv::Vec3f> circles_yellow;//yellow
    cv::HoughCircles(yellow_hue_image, circles_yellow, CV_HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

    
        // Loop over all detected circles and outline them on the image
        //red circle
        if(circles_red.size() == 0) std::exit(-1);
        for(size_t current_circle = 0; current_circle < circles_red.size(); ++current_circle) {
            cv::Point center(std::round(circles_red[current_circle][0]), std::round(circles_red[current_circle][1]));
            int radius = std::round(circles_red[current_circle][2]);
            
            cv::circle(im_in, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
    
            cv::circle(im_in, center, radius, cv::Scalar(0, 255, 0), 5);
        }
    
        //green cirle
        for(size_t i = 0; i < circles_green.size(); i++) {
        cv::Point center(std::round(circles_green[i][0]), std::round(circles_green[i][1]));
        int radius = std::round(circles_green[i][2]);
        
        cv::circle(im_in, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        
        cv::circle(im_in, center, radius, cv::Scalar(0, 255, 0), 5);
    }
    
        //yellow cirle
        for(size_t j = 0; j < circles_yellow.size(); j++) {
        cv::Point center(std::round(circles_yellow[j][0]), std::round(circles_yellow[j][1]));
        int radius = std::round(circles_yellow[j][2]);
        
        cv::circle(im_in, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        
        cv::circle(im_in, center, radius, cv::Scalar(0, 255, 0), 5);
    }
    
        //imshow
        cv::namedWindow("Threshold lower red image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold lower red image", lower_red_hue_range);
        cv::namedWindow("Threshold upper red image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold upper red image", upper_red_hue_range);
        cv::namedWindow("Combined Threshold red Images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined Threshold red Images", red_hue_image);
        cv::namedWindow("Threshold green image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold green image", green_hue_image);
        cv::namedWindow("Threshold yellow image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold yellow image", yellow_hue_image);cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected red circles on the input image", image);
    
        cv::waitKey(0);
    return 0;
}