#ifndef _DerWeg_IMAGEPROCESSINGFUNCTIONS_H__
#define _DerWeg_IMAGEPROCESSINGFUNCTIONS_H__

#include <opencv/cv.h>
#include <vector>

float median(const cv::Mat& mat){
    std::vector<float> array;
    if (mat.isContinuous()) {
      array.assign((float*)mat.datastart, (float*)mat.dataend);
    } else {
      for (int i = 0; i < mat.rows; ++i) {
        array.insert(array.end(), (float*)mat.ptr<uchar>(i), (float*)mat.ptr<uchar>(i)+mat.cols);
      }
    }

    std::vector<float>::iterator first = array.begin();
    std::vector<float>::iterator last = array.end();
    std::vector<float>::iterator middle = first + (last - first) / 2;
    std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
    float median = *middle;
    return median;
}

#endif

