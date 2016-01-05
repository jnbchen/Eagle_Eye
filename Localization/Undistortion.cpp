
#include "Undistortion.h"
#include <iostream>

using namespace std;
using namespace Entzerrung;

void Undistortion::undistort (IplImage* dest, IplImage* src, const PLModel& model, double scaling) {
  int offsetx=static_cast<int>(0.5*dest->width-0.5*scaling*src->width);
  int offsety=static_cast<int>(0.5*dest->height-0.5*scaling*src->height);
  for (int y=0; y<dest->height; ++y) {
    for (int x=0; x<dest->width; ++x) {
      Pos2 p = model.mapInt (Pos2((x-offsetx)/scaling,(y-offsety)/scaling));
      if (p.x>=0 && p.y>=0 && p.x<src->width && p.y<src->height)
        dest->imageData[x+y*dest->widthStep] = src->imageData[p.x+p.y*src->widthStep];
      else
        dest->imageData[x+y*dest->widthStep] = 0;
    }
  }
}

Undistortion::Undistortion (const PLModel& model, unsigned int imagewidth, unsigned int imageheight, double scaling) {
  width=imagewidth;
  height=imageheight;
  lut = new unsigned int [imagewidth*imageheight];
  int offsetx=static_cast<int>(0.5*(1-scaling)*imagewidth);
  int offsety=static_cast<int>(0.5*(1-scaling)*imageheight);
  for (int y=0; y<static_cast<int>(imageheight); ++y) {
    for (int x=0; x<static_cast<int>(imagewidth); ++x) {
      Pos2 p = model.mapInt (Pos2((x-offsetx)/scaling,(y-offsety)/scaling));
      if (p.x>=0 && p.y>=0 && p.x<width && p.y<height) {
        lut[x+y*width]=p.x+width*p.y;
      } else {
        lut[x+y*width]=0;
      }
    }
  }
}


Undistortion::~Undistortion () { delete [] lut; }

void Undistortion::undistort (IplImage* dest, IplImage* src) const {
  char* ptrdest = dest->imageData;
  char* ptrsrcbegin = src->imageData;
  const unsigned int* ptrlut = lut;
  const unsigned int* ptrlutend = lut+width*height;
  for (;ptrlut!=ptrlutend;++ptrlut) {
    *ptrdest++ = *(ptrsrcbegin+(*ptrlut));
  }
}


void Undistortion::undistort(cv::Mat_<cv::Vec3b> &dest, const cv::Mat_<cv::Vec3b> &src) const
{
	for(unsigned int i = 0; i < this->height; i++)
	{
		for(unsigned int j = 0; j < this->width; j++)
		{
			unsigned int shift = this->lut[i * this->width + j];
			unsigned int colIdx = shift % this->width;
			unsigned int rowIdx = shift / this->width;

			dest(i, j) = src(rowIdx, colIdx);
		}
	}
}