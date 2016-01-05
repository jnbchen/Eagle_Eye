#include "TrafficSignDetector.h"

#include <ctype.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include <opencv2/legacy/compat.hpp>

using namespace std;
using namespace DerWeg;

TrafficSign::TrafficSign (SignType t, int x, int y, int h) {
  type = t;
  center_x = x;
  center_y = y;
  height = h;
}

TrafficSign::TrafficSign (const TrafficSign& s) { operator= (s); }

const TrafficSign& TrafficSign::operator= (const TrafficSign& s) {
  type=s.type;
  center_x = s.center_x;
  center_y = s.center_y;
  height = s.height;
  return *this;
}


std::vector<TrafficSign> DerWeg::TrafficSignDetector::process(const cv::Mat& m){
  IplImage i1 = m;

  std::vector<TrafficSign> found_signs;

  // calculate ROI: use the 80% in the center, leave the rest out
  CvRect roi = cvRect(0, i1.height * 0, i1.width, i1.height * 1);
  IplImage *input_image_bgr = cvCreateImage(cvSize(roi.width, roi.height), i1.depth, i1.nChannels);
  cvSetImageROI(&i1, roi);
  cvCopy(&i1, input_image_bgr);

  std::vector<CvSeq*> possible_contours;
  std::vector<CvRect> possible_roi;
  std::vector<IplImage*> images_to_examine;

  // first method: apply Canny filter to the grayscale copy of the input image
  IplImage* inputimage_gray = cvCreateImage(cvSize(input_image_bgr->width, input_image_bgr->height), input_image_bgr->depth, 1);
  cvCvtColor(input_image_bgr, inputimage_gray, CV_BGR2GRAY);

  IplImage* canny = cvCreateImage(cvSize(input_image_bgr->width, input_image_bgr->height), input_image_bgr->depth, 1);
  IplImage* canny_copy = cvCreateImage(cvSize(input_image_bgr->width, input_image_bgr->height), input_image_bgr->depth, 1);
  cvCanny(inputimage_gray, canny, 127, 255);
  cvCopy(canny, canny_copy);

  images_to_examine.push_back(canny);

  // second method: apply color filter (hue component in HSV) to a HSV copy of the input image
  IplImage* color_filtered = cvCreateImage(cvSize(input_image_bgr->width, input_image_bgr->height), input_image_bgr->depth, 1);
  IplImage* input_image_hsv = cvCreateImage(cvSize(input_image_bgr->width, input_image_bgr->height), input_image_bgr->depth, input_image_bgr->nChannels);
  cvCvtColor(input_image_bgr, input_image_hsv, CV_BGR2HSV);
  redBlueColorFilter(input_image_hsv, color_filtered, false, false, 0);


  // open the image to get rid of some noise
  IplImage *img_opened = cvCreateImage(cvGetSize(color_filtered), 8, 1); // image holding the opened structure
  IplImage *img_opened_copy = cvCreateImage(cvGetSize(input_image_hsv), 8, 1);       // copy because the contour finding will use the opened image

  IplConvKernel *kernel = cvCreateStructuringElementEx(2, 2, 0 ,0, CV_SHAPE_RECT);
  cvMorphologyEx(color_filtered, img_opened, NULL, kernel, CV_MOP_OPEN, 1);
  cvReleaseStructuringElement(&kernel);

  cvCopy(img_opened, img_opened_copy);
  images_to_examine.push_back(img_opened);

  // find contours in both images
  CvMemStorage *storage = cvCreateMemStorage();
  CvSeq *first_contour = NULL;

  for (vector<IplImage*>::iterator it = images_to_examine.begin(); it!=images_to_examine.end(); ++it) {
    IplImage *examined = *it;
    cvFindContours(examined, storage, &first_contour, sizeof(CvContour), CV_RETR_LIST);
    for (CvSeq *c=first_contour; c != NULL; c = c->h_next){
      possible_contours.push_back(c);
    }
  }

  // tidy up
  cvReleaseImage(&canny);
  cvReleaseImage(&canny_copy);
  cvReleaseImage(&color_filtered);
  cvReleaseImage(&img_opened);
  cvReleaseImage(&img_opened_copy);

  cvReleaseImage(&input_image_bgr);

  int n = 0;
  for (vector<CvSeq*>::iterator it = possible_contours.begin (); it != possible_contours.end (); ++it){
    CvSeq* c = *it;
    n++;
    // calculate the Hu moments for the shape.
    // First, get the moments
    CvMoments m;
    cvMoments(c, &m);

    // with those, calculate the Hu moments
    CvHuMoments huM;
    cvGetHuMoments(&m, &huM);
    CvRect roi_bb = cvBoundingRect(c);

    int a = roi_bb.height;
    int b = roi_bb.width;

    double lambda = 2; // aspect ratio of the found rect

    if (MIN(a, b) != 0 ) lambda = MAX(a, b) / MIN(a, b);

    if (lambda < 1.3 && MAX(a,b) < 120 && MIN(a,b) > 20){
      //#warning "logarithmic operations here good for debugging, but should maybe be removed"
      if (std::abs(huM.hu5) < 1e-11 && std::abs(huM.hu7) < 1e-11){
        possible_roi.push_back(cvContourBoundingRect(c));
      }
    }
  }
  // now, examine every interesting rect. Some regions may be found with both methods (and just slightly different
  // bounding boxes), so if a sign is approved, the region has to be marked so that it is only processed once.

  // to mark the region, create an empty, black image
  IplImage *region_markers = cvCreateImage(cvGetSize(inputimage_gray), 8, 1);
  cvZero(region_markers);
  for (std::vector<CvRect>::iterator it = possible_roi.begin (); it != possible_roi.end (); ++it){
    CvRect interest_rect = *it;
    // first, test if the region has already been marked. Therefore test if the
    // center of the rect is marked in a different color than black.
    CvPoint cntr = cvPoint(interest_rect.x + interest_rect.width * 0.5, interest_rect.y + interest_rect.height * 0.5);
    if (region_markers->imageData[cntr.x + cntr.y * region_markers->widthStep] != 0){
      continue;
    }
    // first, create a subimage which has the size of the ROI
    IplImage *sub_img_hsv = cvCreateImageHeader(
      cvSize(
        interest_rect.width,
        interest_rect.height
      ),
      input_image_hsv->depth,
      input_image_hsv->nChannels
    );

    sub_img_hsv->origin = input_image_hsv->origin;
    sub_img_hsv->widthStep = input_image_hsv->widthStep;
    sub_img_hsv->imageData = input_image_hsv->imageData +
    interest_rect.y * input_image_hsv->widthStep +
    interest_rect.x * input_image_hsv->nChannels;

    // handling the ROI is much easier now, basically like handling a usual image.

    // create a "histogram" of the interest ellipse defined by the interest rect to find out
    // if there is a significant amount of red or blue pixels compared to other ones
    int nBlue = 0; // number of blue pixels
    int nRed = 0;  // number of red pixels
    int n = 0; // number of counted pixels
    IplImage* filtered_result = cvCreateImage (cvSize(sub_img_hsv->width, sub_img_hsv->height), 8, 1);
    cvZero(filtered_result);
    CvScalar shares = redBlueColorFilter(sub_img_hsv, filtered_result, true, true);
    nBlue = shares.val[0];
    nRed = shares.val[1];
    n = shares.val[2];

    // now do something with the found contents of red / blue.
    bool IS_RED = nRed > 0.6 * n && nRed < 0.9 * n;
    bool IS_BLUE = nBlue > 0.6 * n && nBlue < 0.9 * n;
    if (IS_RED){
      fillRect(interest_rect, region_markers, 255);
      TrafficSign f(NO_ENTRY, interest_rect.x + interest_rect.width / 2,  interest_rect.y + interest_rect.height / 2, interest_rect.height );
      found_signs.push_back(f);
    }
    if (IS_BLUE){
      // flood fill the filtered_result --> finding a good seed point may be
      // a problem, an approach was maybe to find a zero pixel in the center
      int offset_x = filtered_result->width / 2; /// center coordinate
      int offset_y = filtered_result->height / 2; /// center coordinate
      bool do_floodfill = false; /// indicates if there is something to fill, false by default
      for (int i = 0; i < 10 ; i++ ){
        if (filtered_result->imageData[(offset_y + i) * filtered_result->widthStep + offset_x] == 0){
          do_floodfill = true;
          offset_y += i;
          break;
        }
        if (filtered_result->imageData[(offset_y - i) * filtered_result->widthStep + offset_x] == 0){
          do_floodfill = true;
          offset_y -= i;
          break;
        }
      }
      if (do_floodfill){
        cvFloodFill (filtered_result, cvPoint(offset_x, offset_y), cvScalar(127));
        double x_s = 0;
        double y_s = 0;
        int count = 0;
        double x_0 = filtered_result->width / 2.0;
        double y_0 = filtered_result->height / 2.0;
        for (int y = 0; y < filtered_result->height; y++){
          for (int x = 0; x < filtered_result->width; x++){
            uchar val = filtered_result->imageData[y * filtered_result->widthStep + x];
            if (val == 127) {
              x_s += x - x_0;
              y_s += y - y_0;
              count += 1;
            }
          }
        }
        x_s = x_s / (double) count;
        y_s = y_s / (double) count;
        if (std::abs(y_s) > std::abs(x_s) && y_s < -1.5) {
          fillRect(interest_rect, region_markers, 127);
          TrafficSign f(GO_AHEAD, interest_rect.x + interest_rect.width / 2,  interest_rect.y + interest_rect.height / 2, interest_rect.height );
          found_signs.push_back(f);
        }
        else if (std::abs(x_s) > std::abs(y_s) && x_s > -1.5) {
          //  found blue sign pointing right
          fillRect(interest_rect, region_markers, 127);
          TrafficSign f(GO_RIGHT, interest_rect.x + interest_rect.width / 2,  interest_rect.y + interest_rect.height / 2, interest_rect.height );
          found_signs.push_back(f);
        }
        else if (std::abs(x_s) > std::abs(y_s) && x_s < -1.5) {
          //  found blue sign pointing left
          fillRect(interest_rect, region_markers, 127);
          TrafficSign f(GO_LEFT, interest_rect.x + interest_rect.width / 2,  interest_rect.y + interest_rect.height / 2, interest_rect.height );
          found_signs.push_back(f);
        }
      }
    }

    // release the subimages
    cvReleaseImageHeader(&sub_img_hsv);
    cvReleaseImage(&filtered_result);
    cvReleaseImage(&input_image_bgr);
  }
  cvReleaseMemStorage(&storage);
  cvReleaseImage(&input_image_hsv);
  cvReleaseImage(&inputimage_gray);
  cvReleaseImage(&region_markers);

  return found_signs;
}

void DerWeg::TrafficSignDetector::fillRect(CvRect r, IplImage *dest, int color){
  for (int i = r.y; i <= r.y + r.height; i++){
    for (int v = r.x; v <= r.x + r.width; v++){
      dest->imageData[i * dest->widthStep + v] = color;
    }
  }
}

CvScalar DerWeg::TrafficSignDetector::redBlueColorFilter(IplImage *src, IplImage *dest, bool use_ellipse, bool use_histogram, IplImage *){
  int nBlue = 0;
  int nRed = 0;
  int n = 0;
  /* calculate the ellipse parameters, see
   *    http://de.wikipedia.org/w/index.php?title=Datei:Notes_on_deduction_of_formula_of_an_ellipse.svg&filetimestamp=20100413174536
   *    for the meaning of a, b, e.
   */
  double a=1, b=1, e=0, x0=0, y0=0;
  if (use_ellipse){
    b = src->height / 2.0;
    e = sqrt(4 * (src->width / 2.0) * (src->width / 2.0) - b * b);
    a = src->width / 2.0;
    x0 = src->width / 2.0;
    y0 = src->height / 2.0;
  }
  int hist[64];
  int threshold = 0;
  if (use_histogram){
    // first pass: create histogram
    memset(hist, 0, sizeof(int) * 64);
    for(int y  = 0; y < src->height; y++){
      uchar* ptr = (uchar*) (src->imageData + y * src->widthStep);
      for(int x = 0; x < src->width; x++){
        if  ( (use_ellipse && (x - x0) * (x - x0) / (a * a) + (y - y0) * (y - y0) / (b * b) <= 1) || !use_ellipse) {
          int index = ptr[ 3 * x + 1 ] / 4.0;
          hist[ index ] ++;
        }
      }
    }

    // analysis of the histogram
    int max1 = 0;
    int max2 = 0;
    int pos1 = 0;
    int pos2 = 0;

    // flood the histogram
    int flood_level = (src->width * src->height) / 100;
    int highest_value  = 0;
    int highest_pos = 0;

    for (int i = 0; i < 64; i++){
      hist[i] = MAX(hist[i], flood_level);
      int curr_val = hist[i];
      if (curr_val <= flood_level){
        if (highest_value != 0) {
          if (highest_value > max1){
            pos2 = pos1;
            max2 = max1;
            max1 = highest_value;
            pos1 = highest_pos;
          }
          else if (highest_value >= max2 && highest_value < max1){
            max2 = highest_value;
            pos2 = highest_pos;
          }
        }
      }
      else {
        if (curr_val > highest_value){
          highest_value = curr_val;
          highest_pos = i;
        }
      }
    }

    threshold = 4 * ( MAX(pos1, pos2) - (MAX(pos1, pos2) - MIN(pos1, pos2)) / 2 );
  }


  // second pass: filter image, use value found with analysis of the histogram
  for(int y  = 0; y < src->height; y++){
    uchar* ptr = (uchar*) (src->imageData + y * src->widthStep);
    uchar* ptr2 = NULL;
    if (dest != 0) {
      ptr2 = (uchar*) (dest->imageData + y * dest->widthStep);
    }
    for(int x = 0; x < src->width; x++){
      if  ( (use_ellipse && (x - x0) * (x - x0) / (a * a) + (y - y0) * (y - y0) / (b * b) <= 1) || !use_ellipse) {
        bool condition_blue, condition_red;
        if (use_histogram){
          // be a little more liberate with the color values, but also look at the saturation
          bool condition_saturation = (int) ptr[3*x + 1] > threshold;
          condition_blue = ((int) ptr[3*x] > 90 && ptr[3*x] < 130) && condition_saturation;
          condition_red = ((int) ptr[3*x] > 170 || (int) ptr[3*x] < 20) && condition_saturation;
        }
        else{
          // because those rules are applied to the whole image, they must be as strict as possible
          // to get good regions.
          condition_blue = (int) ptr[3*x] > 100 && ptr[3*x] < 115;
          condition_red = (int) ptr[3*x] > 178 || (int) ptr[3*x] < 6;
        }
        bool condition_overexposed = false && (int) ptr[3*x+2] > 250;
        n++;
        if (!condition_overexposed && condition_blue){
          nBlue++;
          if (dest != 0) ptr2[x] = 255;
        }
        else if (!condition_overexposed && condition_red){
          nRed++;
          if (dest != 0) ptr2[x] = 255;
        }
        else {
          if (dest != 0) ptr2[x] = 0;
        }
      }
    }
  }

  return cvScalar(nBlue, nRed, n);
}

