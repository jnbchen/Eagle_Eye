#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#include <cmath>
#include <list>
#include <vector>

using namespace std;
using namespace cv;

typedef std::vector<std::vector<Point2i> > SegList ;


Mat sobel_rot(double angle, int size, int padding=1) {    
    // Get sobel operator for x direction
    Mat kx, ky;
    getDerivKernels(kx, ky, 1, 0, size);
    Mat sobel_x = ky * kx.t();
    Mat sobel_x_pad;
    
    // Add zeros arround
    int p = padding;
    copyMakeBorder(sobel_x, sobel_x_pad,p,p,p,p,BORDER_CONSTANT,Scalar(0));
    
    // Rotate
    Point2i rot_center(size/2+p, size/2+p);
    Mat rot = getRotationMatrix2D(rot_center, angle, 1.0);
    Mat sobel_rot;
    warpAffine(sobel_x_pad, sobel_rot, rot, sobel_x_pad.size(), INTER_LINEAR);
    
    return sobel_rot;
}



SegList angled_ransac(std::vector<Point2i>& edge_points, 
                      double angle_deg, 
                      double max_dist, 
                      size_t min_size) {
    
    SegList all_segments;
    double angle_rad = angle_deg * M_PI/180;
    Point2f normal(cos(angle_rad), sin(angle_rad));
    
    while (edge_points.size() > 0) {
        
        Point2i p = edge_points.back();
        edge_points.pop_back();
        
        std::vector<Point2i> segment;
        segment.push_back(p);
        
        for (int i = edge_points.size() - 1; i >= 0; --i) {
            Point2i tp = edge_points[i];
            Point2f dist = tp - p;
            if (abs(dist.dot(normal)) < max_dist) {
                segment.push_back(tp);
                edge_points.erase(edge_points.begin() + i);
            }
            else {
                continue;
            }
        }
        all_segments.push_back(segment);
    }
    
    for (int i = all_segments.size() -1; i >= 0; --i) {
        if (all_segments[i].size() < min_size) {
            all_segments.erase(all_segments.begin() + i);
        }
        else {
            continue;
        }
    }
    
    return all_segments;
}



double fit_angled_line(std::vector<Point2i>& p, double angle_deg) {
    double m = tan(angle_deg * M_PI/180);
    double sum = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        sum += p[i].y - m * p[i].x;
    }
    return sum / p.size();
}



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


//======================================================================
// PREPROCESSING

//imshow( "Original", image );

Mat median;
//medianBlur(image, median, 3);
GaussianBlur(image, median, Size(5,5), 0);
//imshow( "Median blur", median );

Mat im_hsv;
cvtColor(median, im_hsv, CV_RGB2HSV);

Mat high_value;
// Detect red with high intensity and good saturation:
inRange(im_hsv,Scalar(100,80,50),Scalar(135,255,255),high_value);
//imshow( "Pylons", high_value );

Mat morph = high_value.clone();

int erode_size = 3;
int dilate_size = 3;
Mat element_erode = getStructuringElement( MORPH_ELLIPSE, Size( 2*erode_size + 1, 2*erode_size+1 ),Point( -1, -1 ) );
Mat element_dilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilate_size + 1, 2*dilate_size+1 ),Point( -1, -1 ) );

erode(morph,morph, element_erode);
dilate(morph,morph, element_dilate);
imshow( "Morph", morph );


//======================================================================
// EDGE DETECTION

// TODO: optimal slope ?
double slope_deg = 80;


int filter_size = 3;
double angle = 90 - slope_deg;
Mat sobel_left = sobel_rot(-angle, filter_size);
Mat sobel_right = sobel_rot(angle, filter_size);
cout << sobel_left << endl;

// TODO: how is the correct rotated sobel filter calculated?
/*
std::cout << sobel_left << std::endl;
std::cout << sum(sobel_left) << std::endl;

Mat sobel_left2 = sobel_rot(-angle, siz, 2);
std::cout << sobel_left2 << std::endl;
std::cout << sum(sobel_left2) << std::endl;

Mat sobel_left3 = sobel_rot(-angle, siz, 3);
std::cout << sobel_left3 << std::endl;
std::cout << sum(sobel_left3) << std::endl;


std::cout << sobel_right << std::endl;
std::cout << sum(sobel_right)[0] << std::endl;

Mat sobel_right2 = sobel_rot(angle, siz, 2);
std::cout << sobel_right2 << std::endl;
std::cout << sum(sobel_right2) << std::endl;

Mat sobel_right3 = sobel_rot(angle, siz, 3);
std::cout << sobel_right3 << std::endl;
std::cout << sum(sobel_right3) << std::endl;
*/


Mat grad_left, grad_right;
filter2D(morph, grad_left, CV_32FC1, sobel_left);
filter2D(morph, grad_right, CV_32FC1, sobel_right);

/*
double min, max;
minMaxLoc(grad_left, &min, &max);
minMaxLoc(grad_right, &min, &max);
std::cout << "Min/Max left: " << min << " " << max << std::endl;
std::cout << "Min/Max right: " << min << " " << max << std::endl;
*/

// TODO: lower thresholds?
Mat edges_left, edges_right;
threshold(grad_left, edges_left, 1000, 255, THRESH_BINARY);
threshold(grad_right, edges_right, -1000, 255, THRESH_BINARY_INV);

edges_left.convertTo(edges_left, CV_8UC1);
edges_right.convertTo(edges_right, CV_8UC1);

imshow( "Left Edges", edges_left);
imshow( "Right Edges", edges_right);

// TODO: check if there are non-zero elements at all
std::vector<Point2i> edge_points_left, edge_points_right;
findNonZero(edges_left, edge_points_left);
findNonZero(edges_right, edge_points_right);


//======================================================================
// SEGMENTATION
// by Ransac for given angle

int min_supporting_size = 5;
double max_distance_to_line = 15;

SegList left_segments = angled_ransac(edge_points_left, 
                                      angle, 
                                      max_distance_to_line, 
                                      min_supporting_size);
SegList right_segments = angled_ransac(edge_points_right, 
                                       -angle, 
                                       max_distance_to_line, 
                                       min_supporting_size);

std::cout << "Found " << left_segments.size() << " left segments" << std::endl;
std::cout << "Found " << right_segments.size() << " right segments" << std::endl;

Mat support = image.clone();

for (size_t i = 0; i < left_segments.size(); ++i) {
    Scalar color(rand()%256,rand()%256,rand()%256);
    for (size_t j = 0; j < left_segments[i].size(); ++j) {
        circle(support, left_segments[i][j], 1, color, 1);
    }
}

for (size_t i = 0; i < right_segments.size(); ++i) {
    Scalar color(rand()%256,rand()%256,rand()%256);
    for (size_t j = 0; j < right_segments[i].size(); ++j) {
        circle(support, right_segments[i][j], 1, color, 1);
    }
}

imshow("Support", support);



//======================================================================
// FIT LINES
// by LeastSquares with given angle

// TODO: check if there are segments at all

Mat lines = image.clone();
double slope_rad = slope_deg * M_PI/180;

std::vector<double> left_lines;
for (size_t i = 0; i < left_segments.size(); ++i) {
    double y_offset = fit_angled_line(left_segments[i], -slope_deg);
    left_lines.push_back(y_offset);
    
    Scalar color(255,0,0);
    Point2f start(0, y_offset);
    Point2f direction(cos(-slope_rad), sin(-slope_rad));
    line(lines, start, start + 10000 * direction, color, 1);
}

std::vector<double> right_lines;
for (size_t i = 0; i < right_segments.size(); ++i) {
    double y_offset = fit_angled_line(right_segments[i], slope_deg);
    right_lines.push_back(y_offset);
    
    Scalar color(0,0,255);
    Point2f start(0, y_offset);
    Point2f direction(cos(slope_rad), sin(slope_rad));
    line(lines, start, start + 10000 * direction, color, 1);
}



//======================================================================
// FIND PEAKS

// TODO: add max height threshold

// minimum height of intersection (max for image coords pointing down)
double max_v = 230;

std::sort(left_lines.begin(), left_lines.end(), std::greater<double>());
for (size_t i = 0; i < left_lines.size(); ++i) {
    //cout << left_lines[i] << endl;
}
std::sort(right_lines.begin(), right_lines.end(), std::less<double>());
for (size_t i = 0; i < right_lines.size(); ++i) {
    //cout << right_lines[i] << endl;
}


std::vector<Point2i> peaks;
double m = tan(slope_rad);

while (left_lines.size() > 0) {
    double c_l = left_lines.back();
    double c_r = right_lines.back();
    right_lines.pop_back();
    
    double v = (c_l - c_r) / 2.0 + c_r;
    
    if (v <= max_v) {
        double u = (v - c_r) / m;
        peaks.push_back(Point2f(u, v));
        left_lines.pop_back();
    }
    else {
        continue;
    }
}

std::cout << "Peak coordinates:" << std::endl;
for (size_t i = 0; i < peaks.size(); ++i) {
    cout << peaks[i] << endl;
    circle(lines, peaks[i], 1, Scalar(0,255,0), 3);
}

imshow("Lines", lines);

/*
std::stringstream ss;
ss << "test/" << file_path;
imwrite(ss.str(), lines);
*/

std::cout << "Done" << std::endl;

waitKey(0);
}
