#ifndef _DerWeg_TRAFFICSIGNDETECTOR_H_
#define _DerWeg_TRAFFICSIGNDETECTOR_H_

#include <vector>
#include <opencv/cv.h>

namespace DerWeg {

  /** enum to classify traffic signs */
  enum SignType {
    NO_ENTRY,
    GO_LEFT,
    GO_RIGHT,
    GO_AHEAD
  };

  /** This structure represents a sign found in an image. The type is
   * one from the enumeration "Sign", information about the location and
   * the diameter is added. */
  struct TrafficSign {
    int center_x;  ///< center of bounding box in image coordinates
    int center_y;  ///< center of bounding box in image coordinates
    int height;    ///< height of bounding box

    SignType type;  ///< traffic sign type

    TrafficSign (SignType t, int x, int y, int h);
    TrafficSign (const TrafficSign& s);
    const TrafficSign& operator= (const TrafficSign& s);
  };

  /** the traffic sign detector itself. Call process (image) to detect
   * traffic signs in an image */
  class TrafficSignDetector {
  private:
    void fillRect(CvRect, IplImage*, int);
    CvScalar redBlueColorFilter(IplImage *src, IplImage *dest, bool use_ellipse = false, bool use_histogram = false, IplImage *viz = 0);
  public:
    std::vector<TrafficSign> process (const cv::Mat&);
  };

}

#endif // SIGNDETECTOR_H
