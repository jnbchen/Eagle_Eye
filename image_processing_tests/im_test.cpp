#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

    int use_color;
    string file_path;

    // -------------------------------------------
    if (argc >=2) {
        file_path = argv[1];
    } else {
        file_path = "left.png";
    }
    if (argc >=3) {
        use_color = atoi(argv[2]);
    } else {
        use_color = 0;
    }
    //--------------------------------------------

    // First argument:
    // path to picture
    // Second argument:
    // CV_LOAD_IMAGE_GRAYSCALE = 0
    // CV_LOAD_IMAGE_COLOR = 1
    Mat image = imread(file_path, use_color);

    // Check for invalid input
    if(! image.data ) {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

/*
    cout << "Type CV_8UC1: "<<bool(image.type() == CV_8UC1) <<endl;
    cout << "Type CV_8UC3: "<<bool(image.type() == CV_8UC3) <<endl;
    cout << "Type "<<image.type()<<endl;
    cout << "Channels "<<image.channels()<<endl;

    cout << "r,c " << image.rows << ", "<<image.cols <<endl;
*/

// Colored picture operations -----------------------------------------------------------------------
if (use_color==1) {
Mat im_hsv, extract_v;
cvtColor(image,im_hsv,CV_RGB2HSV);
extractChannel(image, extract_v,0);

//imshow( "w1", im_hsv );
//imshow( "w2", extract_v );
imshow( "Original", image );

Mat high_value;
inRange(im_hsv,Scalar(100,150,200),Scalar(120,255,255),high_value);
imshow( "High Intensity", high_value );

Mat median;
medianBlur(high_value,median,31);
imshow( "Median blur", median );


/*
    for(int i = 0; i < image.rows; i++) {
        for(int j = 0; j < image.cols; j++) {
            Vec3b values = image.at<Vec3b>(i, j);
            int b = values[0];
            int g = values[1];
            int r = values[2];

            if ((i+j)%100 == 0) {
                //cout << "i,j = "<< i << ", " <<j<<endl;
                //cout << "Blue: " << b << endl;
                //cout << "Green: " << g << endl;
                //cout << "Red: " << r << endl;

                image.at<Vec3b>(i, j)[0]=0;
                image.at<Vec3b>(i, j)[1]=255;
                image.at<Vec3b>(i, j)[2]=255;
            }
        }
    }
*/


}

// Greyscale picture operations -----------------------------------------------------------------------
if (use_color == 0) {
    for(int i = 0; i < image.rows; i++) {
        for(int j = 0; j < image.cols; j++) {
            int intensity = image.at<uchar>(i,j);

            if ((i+j)%100 == 0) {
                //cout << "i,j = "<< i << ", " <<j<<endl;
                //cout << intensity << "   ";

                image.at<uchar>(i, j)=255;
            }
        }
    }
}

/*  //For faster access use pointers:

    int *myData = image.data;
    int width = image.cols;
    int height = image.rows;
    int _stride = image.step;//in case cols != strides
    for(int i = 0; i < height; i++)
    {
        for(int j = 0; j < width; j++)
        {
            int val = myData[ i * _stride + j];
            if (i+j%10==0) {
                cout << val << endl;
            }
        }
    }
*/

//    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
//    imshow( "Display window", image );                   // Show our image inside it.

    waitKey(0);                                          // Wait for a keystroke in the window
    return 0;
}
