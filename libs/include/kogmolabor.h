// perform dense stereo vision asynchronously on the GPU

#pragma once

#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 3
	#include <opencv2/core.hpp>
	#include <opencv2/cuda.hpp>
#elif CV_MAJOR_VERSION == 2
	#include <opencv2/core/core.hpp>
	#include <opencv2/gpu/gpu.hpp>
#else
	#error "unsupported version of OpenCV"
#endif
#include <boost/smart_ptr/scoped_ptr.hpp>
#include <boost/noncopyable.hpp>

namespace DerWeg {
  
  /** forward declaration for pimpl pattern, not to be used anywhere else */
  struct StereoGPUImpl;
  
  /** class to perform stereo vision asynchronously on the GPU */
  class StereoGPU : boost::noncopyable {
  
  // user interface
  public:
    
    /** initialize all parameters
     * @param calibration_filename
     *   path of the file containing the stereo camera calibration */
    StereoGPU(const std::string calibration_filename);
    
    /** explicit destructor required for pimpl pattern with "boost::scoped_ptr" */
    ~StereoGPU();
    
    /** enqueue stereo image pair to be processed on the GPU, returns almost 
     *  immediately
     * @pre
     *   no previous image pair being processed, no results waiting for retrieval
     * @post
     *   input copied to internal buffers, computations enqueued
     * @param left_image
     *   8-bit image from left camera
     * @param right_image
     *   8-bit image from right camera */
    void runStereoMatching(const cv::Mat left_image,
                           const cv::Mat right_image);
    
    /** retrieve left rectified image already before stereo matching has 
     *  completed, still blocks if rectification not yet finished
     * @pre
     *   a stereo image pair has been enqueued
     * @post
     *   rectification of input images has completed
     * @param left_rectified_image
     *   rectified left image with same color mode as input
     */
    void getRectifiedLeftImage(cv::Mat& left_rectified_image);
    
    /** retrieve right rectified image already before stereo matching has 
     *  completed, still blocks if rectification not yet finished
     * @pre
     *   a stereo image pair has been enqueued
     * @post
     *   rectification of input images has completed
     * @param right_rectified_image
     *   rectified right image with same color mode as input
     */
    void getRectifiedRightImage(cv::Mat& right_rectified_image);
    
    /** retrieve stereo matching results, blocks if not yet finished
     * @pre
     *   a stereo image pair has been enqueued
     * @post
     *   no operations enqueued, results copied to output images
     * @param depth
     *   distances of the rectified left image's pixels in meters and in 
     *   floating-point format
     * @param confidence
     *   confidences of distances in unsigned char format (255 best, 0 worst) */
    void getStereoResults(cv::Mat& distance_image,
                          cv::Mat& confidence_image);
    
    /** get rectified projection matrix e.g. for reconstructing 3D points
     * @param projection_matrix
     *   3x4 projection matrix of the (virtual) rectified camera */
    void getProjectionMatrix(cv::Mat& projection_matrix);
    
    /** check if computations still running or already finished */
    bool isRunning();
    
  // internal implementation using the pimpl pattern
  private:
    boost::scoped_ptr< StereoGPUImpl > pimpl ;
    
  }; // class "DerWeg::StereoGPU"
  
} // namespace "DerWeg"
