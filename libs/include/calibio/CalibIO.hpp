// utility function for reading a calibration file
// Author: Andreas Geiger, 2011

#ifndef __CALIB_IO_H__
#define __CALIB_IO_H__

#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdint.h>
#include <vector>
#include <string>

class CvMat; // forward declaration to avoid OpenCV compile time dependency (jz)

class CalibIO {

public:

  // calibration parameters of a single camera
  struct Camera {
    CvMat *S;      // image size before rectification
    CvMat *K;      // calibration matrix before rectification
    CvMat *D;      // distortion parameters before rectification
    CvMat *R;      // extrinsic rotation before rectification
    CvMat *T;      // extrinsic translation before rectification
    CvMat *S_rect; // image size after rectification
    CvMat *R_rect; // rectifying rotation
    CvMat *P_rect; // projection matrix after rectification
    Camera();
    Camera( const Camera& that );
    Camera& operator=( const Camera& that );
    ~Camera();
  };

  CalibIO();
  ~CalibIO();
  bool readCalibFromFile(std::string calib_file_name);
  void showCalibrationParameters(bool compact=true);
  // Warning, returns pointers to internal data. If you want the data to
  // life longer than this object, you should copy it.
  void computeLUT(int camera_index, bool unwarp_only, CvMat*& cu, CvMat*& cv );

  std::string         calib_time;  // calibration time
  float               corner_dist; // corner dist
  std::vector<Camera> cameras;     // camera calibration

private:

  void                     showCvMat  (CvMat *m,std::string matrix_name,uint32_t cam);
  std::vector<std::string> splitLine  (std::string line);
  std::string              readString (FILE *calib_file,const char *string_name,bool &success);
  float                    readValue  (FILE *calib_file,const char *value_name,bool &success);
  CvMat*                   readMatrix (FILE *calib_file,const char *matrix_name,uint32_t cam,uint32_t m,uint32_t n,bool &success);

  // LUTs read directly from a TS style file
  std::vector<CvMat*> cus_TS;
  std::vector<CvMat*> cvs_TS;
};

#endif
