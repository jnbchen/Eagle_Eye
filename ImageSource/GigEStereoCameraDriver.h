#ifndef _DerWeg_GigEStereoCameraDriver_h_
#define _DerWeg_GigEStereoCameraDriver_h_

#include "GigE.h"
#include <vector>

#include "../Elementary/ImageBuffer.h"


namespace DerWeg {

  void stereoFrameDoneCallback (tPvFrame* frame);  ///< Callback function, necessary to declare it here although it should not be used explicitely

  /** A camera driver for Prosilica GigE cameras. */
  class GigEStereoCameraDriver {
  private:
    /** specialized buffer structure with access attributes */
    class StereoExtendedImageBuffer : public ImageBuffer {
      tPvFrame frameLeft;   ///< Pv-driver specific representation
      tPvFrame frameRight;
      int locks;    ///< number of reading (>0) or writing (<0) accesses on buffer
      bool bufferRead;   ///< attribute is true if buffer was already read
      bool leftComplete, rightComplete;
      Timestamp leftCapture, rightCapture;
    public:
      StereoExtendedImageBuffer ();
      StereoExtendedImageBuffer (unsigned int width, unsigned int height, int channels);
      StereoExtendedImageBuffer (const StereoExtendedImageBuffer&);
      /** resize buffer with given width and height. channels is either 1 (Mono) or 3 (BGR) */
      void resize (unsigned int width, unsigned int height, int channels);

      friend class DerWeg::GigEStereoCameraDriver;
      friend void DerWeg::stereoFrameDoneCallback (tPvFrame* frame);
    };

    static std::vector<GigEStereoCameraDriver*> driversList;  ///< a list of all driver objects, necessary to implement the callback function frameDoneCallback
    static boost::mutex mutexDriversList;   ///< mutex to control access on driversList
    static std::vector<StereoExtendedImageBuffer*> lostBuffers;   ///< a list of image buffers which exist, however, the instance of GigECameraDriver which owned them does not exist any more
    static boost::mutex mutexLostBuffers;   ///< mutex to control access on lostBuffers
    friend void ::DerWeg::stereoFrameDoneCallback (tPvFrame*);

    tPvHandle leftHandle, rightHandle;   ///< handle on camera

    bool didStart;   ///< is transmission running?
    bool bufferOverflow;   ///< flag is set when all buffers are locked
    boost::mutex mutexBuffers;   ///< mutex to control access on buffers, didStart, and bufferOverflow
    boost::condition_variable conditionalBufferReleased;   ///< conditional to signal when a buffer has been released

    std::vector<StereoExtendedImageBuffer*> buffers;  ///< the image buffers owned by the driver

    void putFrameOnBus (unsigned int k =1);   ///< search k empty image buffer and put them on the bus for being filled. mutexBuffers must be locked outside of putFrameOnBus!

    Timestamp captureStart;   ///< time when startTransmission was called
    unsigned long int numImages;   ///< number of images captured

    void init(unsigned int leftIp, unsigned int rightIp) throw (std::invalid_argument);
  public:
    /** color modes to encode color mode of the camera */
    enum ColorMode { Mono8, BGR24, Bayer8 };  
    /** start camera driver,
      but do not start transmission. Throw exception
      if no camera has been found or initialisation failed.
      \arg n: number of images that can be provided by the camera driver in parallel
      \arg leftIp, rigtIp: IP adresses of desired cameras */
    GigEStereoCameraDriver (unsigned int n=1) throw (std::invalid_argument);
    GigEStereoCameraDriver (unsigned int n, unsigned int leftIp, unsigned int rightIp) throw (std::invalid_argument);
    ~GigEStereoCameraDriver () throw ();

    /** set color mode */
    bool setImageFormat(ColorMode mode);
    /** set image size to with w, height h, horizontal offset u0,
     v ertical offset v0, and color mode */
    bool setImageFormat(unsigned int w, unsigned int h, unsigned int u0, unsigned int v0, ColorMode mode);

    /** start the transmission of images from the camera to the computer. return true on success */
    bool startTransmission () throw();
    /** stop the transmission of images from the camera to the computer. return true on success */
    bool stopTransmission () throw();

    /** get the latest image captured by the camera. If blocking=true and the
      latest image has already been read and transmission is running the
      method will wait until a new image is captured. If no image is available
      or the transmission is not running a NULL-pointer is returned. The
      image buffer obtained must be released explicitely calling releaseImage().
      The image buffer is valid until releaseImage() is called. */
    const ImageBuffer* getImage (bool blocking);
    /** release an image buffer. Afterwards, the buffer is not valid any more
      and accessing it might cause a segmentation fault or unexpected behavior */
    static void releaseImage (const ImageBuffer* buf);

    /** return maximal image width */
    unsigned int maxImageWidth() throw (std::invalid_argument);
    /** return maximal image height */
    unsigned int maxImageHeight() throw (std::invalid_argument);

    /** set white balance to values (u,v). Return true on success */
    bool setWhiteBalance (unsigned int u, unsigned int v) throw();
    /** set auto white balance and return true on success */
    bool setAutoWhiteBalance () throw();
    /** set gain to value g and return true on success */
    bool setGain (unsigned int g) throw();
    /** set exposure to value e and return true on success */
    bool setExposure (unsigned int e) throw();
    /** set auto exposure with target value e and return true on success */
    bool setAutoExposure (unsigned int e) throw();

    /** return frame rate in frames per second */
    double frameRate () throw();

  private:
    GigEStereoCameraDriver (const GigEStereoCameraDriver&);  ///< not implemented
    const GigEStereoCameraDriver& operator= (const GigEStereoCameraDriver&);  ///< not implemented
  };

}


#endif // _DerWeg_GigEStereoCameraDriver_h_
