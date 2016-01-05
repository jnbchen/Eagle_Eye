
#ifndef _Entzerrung_undistortion_h_
#define _Entzerrung_undistortion_h_

#include "PLModel.h"
//#include <cv.h>
#include <opencv2/opencv.hpp>

namespace Entzerrung {

  /** Klasse, um das Entzerren von Bildern zu kapseln. Interpoliert Grauwerte 
      nicht sondern setzt den Grauwert des naechsten Nachbarn */
  class Undistortion {
    unsigned int* lut;
    int width;
    int height;
  public:
    /** ein einzelnes Bild entzerren, ohne LUT; Zielbild (dest) und Quellbild 
        (src) koennen unterschiedlich gross sein */
    static void undistort (IplImage* dest, IplImage* src, const PLModel& model, double scaling =1.0);
    
    /** Konstruktor, erzeugt eine LUT */
    Undistortion (const PLModel& model, unsigned int imagewidth, unsigned int imageheight, double scaling =1.0);
    ~Undistortion ();
    /** entzerren unter Ausnutzung der LUT. src und dest muessen die im #
        Konstruktor uebergebene Groesse besitzen */
    void undistort (IplImage* dest, IplImage* src) const;

	void undistort(cv::Mat_<cv::Vec3b> &dest, const cv::Mat_<cv::Vec3b> &src) const;
  };

}

#endif

