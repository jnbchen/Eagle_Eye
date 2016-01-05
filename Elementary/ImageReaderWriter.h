
#ifndef IMAGEREADERWRITER_H__
#define	IMAGEREADERWRITER_H__

#include "ImageBuffer.h"

namespace DerWeg {

  /** class that reads a stream of images from disk.
   * Image names are created as prefixYXXXXXXsuffix where Y is either
   * L(left) or R(right) and XXXXXX is the running number of the image */
  class ImageReader {
    std::string prefix;
    int num;
    std::string ext;
  public:
    ImageReader() : prefix("bild"), num(0), ext(".png") { };
    ~ImageReader() {};

    void setPrefix(const std::string&); ///< change image prefix
    void setSuffix(const std::string&); ///< change image suffix
    std::string getFilename(bool left);  ///< file name of latest (left/right) image
    ImageBuffer getImage(); ///< get next image, if possible. Return an empty image if no image is available
  };

  /** class that writes images to disk.
   * Image names are created as prefixYXXXXXXsuffix where Y is either
   * L(left) or R(right) and XXXXXX is the running number of the image */
  class ImageWriter {
    std::string prefix;
    int num;
    std::string ext;
  public:
    ImageWriter() : prefix("bild"), num(0), ext(".png") { };
    ~ImageWriter() {};

    void setPrefix(const std::string&); ///< change image prefix
    void setSuffix(const std::string&); ///< change image suffix
    std::string getFilename(bool left);  ///< file name of latest (left/right) image
    void setImage(const ImageBuffer&);  ///< write this file
  };

} // namespace DerWeg

#endif // IMAGEREADERWRITER_H__
