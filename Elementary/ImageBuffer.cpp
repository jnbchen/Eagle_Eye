
#include "ImageBuffer.h"
#include <cstring>

using namespace DerWeg;

ImageBuffer::ImageBuffer () {;}

ImageBuffer::ImageBuffer (const ImageBuffer& ib) { 
  operator= (ib);
}

ImageBuffer::ImageBuffer (unsigned int width, unsigned int height, int channels, bool is_stereo) : image (height, width, channels==1 ? CV_8UC1 : CV_8UC3) {
  if (is_stereo) {
    image_right = image.clone();
  }
}

const ImageBuffer& ImageBuffer::operator= (const ImageBuffer& ib) {
  image=ib.image;
  image_right=ib.image_right;
  capturetime=ib.capturetime;
  return *this;
}

ImageBuffer ImageBuffer::clone() const {
  ImageBuffer res;
  res.capturetime = capturetime;
  res.image = image.clone();
  res.image_right = image_right.clone();
  return res;
}

bool ImageBuffer::is_empty () const throw () {
  return image.empty();
}

bool ImageBuffer::is_stereo () const throw () {
  return !image_right.empty();
}
