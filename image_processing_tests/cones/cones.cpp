#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <list>

using namespace std;
using namespace cv;

int main( int argc, char** argv ) {

string file_path;

// -------------------------------------------
if (argc >=2) {
    file_path = argv[1];
} else {
    file_path = "cone.png";
}
//--------------------------------------------

// First argument:
// path to picture
// Second argument:
// CV_LOAD_IMAGE_GRAYSCALE = 0
// CV_LOAD_IMAGE_COLOR = 1
Mat image = imread(file_path, 1);

// Check for invalid input
if(! image.data ) {
    cout <<  "Could not open or find the image" << std::endl ;
    return -1;
}


// PREPROCESSING ===============================================================================================


//imshow( "Original", image );

Mat im_hsv;
cvtColor(image,im_hsv,CV_RGB2HSV);


Mat high_value;
// Detect red with high intensity and good saturation:
inRange(im_hsv,Scalar(100,80,50),Scalar(135,255,255),high_value);
imshow( "Pylons", high_value );

Mat median;
medianBlur(high_value,median,3);
imshow( "Median blur", median );

Mat morph = median.clone();

int erode_size = 3;
int dilate_size = 3;
Mat element_erode = getStructuringElement( MORPH_ELLIPSE, Size( 2*erode_size + 1, 2*erode_size+1 ),Point( -1, -1 ) );
Mat element_dilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilate_size + 1, 2*dilate_size+1 ),Point( -1, -1 ) );

dilate(morph,morph, element_dilate);
erode(morph,morph, element_erode);

erode(morph,morph, element_erode);
dilate(morph,morph, element_dilate);

//dilate(morph,morph, element_dilate);
//erode(morph,morph, element_erode);

imshow( "Morph", morph );



// EDGE DETECTION =================================================================================================



// filtering with [-1 0 1] (just for comparison)
Mat boundary;
Mat kernel(1,3,CV_64FC1);
kernel.at<double>(0,0) = -1;
kernel.at<double>(0,1) = 0;
kernel.at<double>(0,2) = 1;
filter2D(morph, boundary, CV_64F, kernel);

Mat vis_boundary = boundary + 1;
vis_boundary = vis_boundary * 0.50;
imshow( "Horizontal Edges", vis_boundary );



//Sobel filtering (just for comparison)
Mat sobel;
Sobel(morph, sobel, CV_64F, 1, 0, 3);

Mat vis_sobel= sobel +1;
vis_sobel*=.50;
imshow( "Sobel Edges", vis_sobel );



// MY METHOD FOR EDGE PIXELS:

Mat extract_edge_pix = morph;
Mat edge_vis = morph.clone();
edge_vis = Scalar(0);

// get edge pixels
list< pair< Point2f, int > > edges;

int last_val = 0;
for(int i = 1; i < extract_edge_pix.rows - 1 ; i++) {
    last_val = extract_edge_pix.at<uchar>(i, 0);

    for(int j = 1; j < extract_edge_pix.cols - 1; j++) {
        int value = extract_edge_pix.at<uchar>(i, j);

        int vertical_difference = extract_edge_pix.at<uchar>(i + 1, j) - extract_edge_pix.at<uchar>(i - 1, j);

        if (value != last_val && vertical_difference >= 0) {
            int sign = (value - last_val > 0)? 1 : -1;
            edges.push_back(pair<Point2f, int>(Point2f(j,i), sign) );
            edges.push_back(pair<Point2f, int>(Point2f(j + sign, i), sign) );

            last_val = value;
            edge_vis.at<uchar>(i,j) = 255;
            edge_vis.at<uchar>(i,j+sign) = 255;
        }
    }
}

imshow( "My Edges", edge_vis);





// RANSAC ===============================================================================================


Mat line_image = image.clone();
Mat support_image_vis = image.clone();



double slope_deg = 80; //degree
double slope_rad = slope_deg * M_PI/180; // radians
double slope = tan(slope_rad); // delta_y / delta_x

Point2f normal_pos(slope,1);
normal_pos *=  1/norm(normal_pos);
Point2f normal_neg(-slope,1);
normal_neg *= 1/norm(normal_neg);


// PARAMETERS ==========================
int min_supporting_size = 25;
double max_distance_to_line = 7;

// counter
int c=0;
int c_max = 200;

while (edges.size() > 0 && c < c_max) {
    c++;

    list< pair< Point2f, int > >::iterator b_iter = edges.begin();
    int index  = (rand() % edges.size());
    for (int k=0; k < index; k++) {
        b_iter++;
    }

    Point2f p = (*b_iter).first;
    int sign = (*b_iter).second;
    //Point2f p = edges.front().first; //edges[index].first;
    //int sign = edges.front().second; //edges[index].second;


    Point2f normal = (sign > 0)? normal_pos : normal_neg;

    //get supporting set:
    vector< Point2f > supporting_points;
    int b = rand()%256;
    int g = rand()%256;
    int r = rand()%256;
    Scalar color(b,g,r);

    for (list< pair< Point2f, int > >::iterator it = edges.begin(); it != edges.end(); ++it) {
        Point2f test_pt = (*it).first;
        Point2f diff = test_pt - p;
        // distance of test point to line
        double distance = abs(diff.dot(normal));

        int test_sign = (*it).second;

        if (distance < max_distance_to_line && sign == test_sign) {

            //cout << distance << endl;

            supporting_points.push_back(test_pt);
            it = edges.erase(it);
            if (it != edges.begin()) {
                it--;
            }
            circle(support_image_vis, test_pt, 1,color,1);
        }

    }

    //imshow( "Support", support_image_vis );
    //waitKey(0);


    if (supporting_points.size() > min_supporting_size) {
        Vec4f line_fit;
        fitLine(supporting_points, line_fit, CV_DIST_L2, 0, 0.01, 0.01);
        Point2f direction(line_fit[0],line_fit[1]);
        Point2f start(line_fit[2],line_fit[3]);

        //cout << abs(abs(direction.y/direction.x) - slope) << endl;

        //Filtering lines which have a slope too much differing from 80 degree
        if ( abs(abs(direction.y/direction.x) - slope) > 3.5) continue;

        line(line_image, start, start + 2000 * direction, Scalar(255,255,0),2);

        line(line_image, start, start - 2000 * direction, Scalar(255,255,0),2);
    }


}

//imshow( "Original", image );

imshow( "Support", support_image_vis );


imshow( "Lines", line_image );






/*
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
*/

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
