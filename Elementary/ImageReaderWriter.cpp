
#include "ImageReaderWriter.h"
#include <sstream>
#include <iomanip>
#include <opencv/highgui.h>


namespace DerWeg {

  void ImageReader::setPrefix(const std::string& pref) {
    prefix = pref;
  }

  void ImageReader::setSuffix(const std::string& suf) {
    ext = suf;
  }

  std::string ImageReader::getFilename(bool left) {
    std::ostringstream str;
    str << prefix << (left ? "L" : "R") << std::setfill('0') << std::setw(6) << num << ext;
    return str.str();
  }

  ImageBuffer ImageReader::getImage() {
    num++;
    ImageBuffer ib;
    ib.image = cv::imread (getFilename(true).c_str(), -1);
    ib.image_right = cv::imread (getFilename(false).c_str(), -1);
    return ib;
  }




  void ImageWriter::setPrefix(const std::string& pref) {
    prefix = pref;
  }

  void ImageWriter::setSuffix(const std::string& suf) {
    ext = suf;
  }

std::string ImageWriter::getFilename(bool left) {
    std::ostringstream str;
    str << prefix << (left ? "L" : "R") << std::setfill('0') << std::setw(6) << num << ext;
    return str.str();
  }

  void ImageWriter::setImage(const ImageBuffer& img) {
    num++;
    if (!img.image.empty()) {
      cv::imwrite (getFilename(true).c_str(), img.image);
    }
    if (!img.image_right.empty()) {
      cv::imwrite (getFilename(false).c_str(), img.image_right);
    }
  }

} // namespace DerWeg
