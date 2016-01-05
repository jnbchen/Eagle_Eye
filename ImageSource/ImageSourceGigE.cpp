
#include "GigECameraDriver.h"
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace {
  unsigned int ipString2uint (const std::string& s) {
    std::stringstream inout;
    for (unsigned int i=0; i<s.length(); ++i) {
      if (s[i]=='.') {
        inout.put(' ');
      } else {
        inout.put(s[i]);
      }
    }
    unsigned int r = 0;
    unsigned int shift = 0;
    while(!inout.eof()) {
      unsigned int n=0;
      inout >> n;
      r = r|(n<<shift);
      shift+=8;
    }
    return r;
  }
}

namespace DerWeg {

  /** An implementation of ImageSource that reads images from
    a Prosilica GigEVision camera */
  class ImageSourceGigE : public KogmoThread {
    GigECameraDriver* camera;
    GigECameraDriver::ColorMode mode_cam;
    GigECameraDriver::ColorMode mode_app;
    void execute ();
  public:
    ImageSourceGigE() : camera(NULL) {};
    ~ImageSourceGigE() {
      if (!camera) return;
      camera->stopTransmission();
      delete camera;
    }
    void init (const ConfigReader&);
  };


  void ImageSourceGigE::init (const ConfigReader& cr) {
    std::string sval;
    unsigned int uval;
    std::vector<unsigned int> uuval;
    std::string ipString;
    cr.get("GigETop::address", ipString);
    unsigned int ip = ipString2uint (ipString);

    camera = new GigECameraDriver (4, ip);
    sval="BGR24";
    cr.get("GigETop::color_space", sval);
    mode_app = (sval=="Mono8" ? GigECameraDriver::Mono8 : GigECameraDriver::BGR24);
    sval="Bayer8";
    cr.get("GigETop::camera_color_space", sval);
    mode_cam = (sval=="Mono8" ? GigECameraDriver::Mono8 : GigECameraDriver::Bayer8);
    cr.get("GigETop::image_size", uuval);
    if (uuval.size()>=4)
      camera->setImageFormat (uuval[0], uuval[1], uuval[2], uuval[3], mode_cam);
    else
      camera->setImageFormat (mode_cam);
    if (cr.get("GigETop::white_balance", uuval))
      camera->setWhiteBalance (uuval[0], uuval[1]);
    else
      camera->setAutoWhiteBalance ();
    if (cr.get("GigETop::gain", uval))
      camera->setGain (uval);
    if (cr.get("GigETop::exposure", uval))
      camera->setExposure (uval);
    if (cr.get("GigETop::auto_exposure", uval))
      camera->setAutoExposure (uval);
    camera->startTransmission();
  }

  void ImageSourceGigE::execute() {
    try{
      const ImageBuffer* ib1 = NULL;
      while (true) {
        ib1=camera->getImage(true);
        if (ib1==NULL)
          break;
        ImageBuffer bbbuf;
        switch (mode_cam) {
          case GigECameraDriver::Mono8:
            switch (mode_app) {
              case GigECameraDriver::Mono8: bbbuf.image = ib1->image.clone(); break;
              case GigECameraDriver::BGR24:
              default: cv::cvtColor(ib1->image, bbbuf.image, CV_GRAY2BGR); break;
            }
            break;
          case GigECameraDriver::BGR24:
            switch (mode_app) {
              case GigECameraDriver::Mono8: cv::cvtColor(ib1->image, bbbuf.image, CV_BGR2GRAY); break;
              case GigECameraDriver::BGR24:
              default: bbbuf.image = ib1->image.clone(); break;
            }
            break;
          case GigECameraDriver::Bayer8:
            switch (mode_app) {
              case GigECameraDriver::Mono8:
                {
                  cv::Mat imp;
                  cv::cvtColor(ib1->image, imp, CV_BayerBG2BGR);
                  cv::cvtColor(imp, bbbuf.image, CV_BGR2GRAY);
                }
                break;
              case GigECameraDriver::BGR24:
              default: cv::cvtColor(ib1->image, bbbuf.image, CV_BayerBG2BGR); break;
            }
            break;
        }

//        Timestamp now;
//        EOUT("M " << now.get_msec() << std::endl);
        //cv::imwrite("./Hallo.png", bbbuf.image);

        BBOARD->setTopImage(bbbuf);
        camera->releaseImage (ib1);
        boost::this_thread::interruption_point();
      }
    }catch(boost::thread_interrupted&){;}
  }

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ImageSourceGigE> imagesource_gige ("TopGigE");

} // namespace
