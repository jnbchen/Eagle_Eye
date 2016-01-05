
#ifndef _DerWeg_ImageBuffer_h_
#define _DerWeg_ImageBuffer_h_

#include "Timestamp.h"
#include <opencv/cv.h>


namespace DerWeg {

  /** struct to represent a raw image buffer plus information about the
    capture time */
  struct ImageBuffer {
    cv::Mat image;          ///< the image (mono) or the left image (stereo)
    cv::Mat image_right;    ///< the right image (only stereo)
    Timestamp capturetime;  ///< the timestamp

    ImageBuffer ();
    /** copy constructor creates flat (!) copy, i.e. image buffer is not cloned */
    ImageBuffer (const ImageBuffer&);
    /** create new image buffer 
     * \param[in] width      desired image width in pixels
     * \param[in] height     desired image height in pixels
     * \param[in] channels   1 for a monochromatic picture, 3 for a color picture
     * \param[in] is_stereo  true if a stereo image pair should be created */
    ImageBuffer (unsigned int width, unsigned int height, int channels, bool is_stereo =false);
    /** creates a flat (!) copy, i.e. image buffers are not cloned */
    const ImageBuffer& operator= (const ImageBuffer&);
    /** creates a deep (!) copy of itself, i.e. image buffers are copied */
    ImageBuffer clone() const;
    /** return true if image buffer is empty */
    bool is_empty () const throw ();
    /** return true if image buffer contains a stereo image pair */
    bool is_stereo () const throw ();
  };

}

#endif
