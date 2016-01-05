
#include "GigEStereoCameraDriver.h"
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>

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

  /** An implementation of StereoImageSource that reads stereo images from
    a Prosilica GigEVision camera */
  class StereoImageSourceGigE : public KogmoThread {
    GigEStereoCameraDriver * camera;
    GigEStereoCameraDriver::ColorMode mode_cam;
    GigEStereoCameraDriver::ColorMode mode_app;
    void execute ();
  public:
    StereoImageSourceGigE() {};
    ~StereoImageSourceGigE() { camera->stopTransmission(); delete camera;}
    void init (const ConfigReader&);
  };


  void StereoImageSourceGigE::init (const ConfigReader& cr) {
    std::string sval;
    unsigned int uval;
    std::vector<unsigned int> uuval;
    std::string leftIpString, rightIpString;
    cr.get("GigE::left_address", leftIpString);
    cr.get("GigE::right_address", rightIpString);
    unsigned int leftIp = ipString2uint (leftIpString);
    unsigned int rightIp = ipString2uint (rightIpString);

    camera = new GigEStereoCameraDriver(4, leftIp, rightIp);
    sval="BGR24";
    cr.get("GigE::color_space", sval);
    mode_app = (sval=="Mono8" ? GigEStereoCameraDriver::Mono8 : GigEStereoCameraDriver::BGR24);
    sval="Bayer8";
    cr.get("GigE::camera_color_space", sval);
    mode_cam = (sval=="Mono8" ? GigEStereoCameraDriver::Mono8 : GigEStereoCameraDriver::Bayer8);
    cr.get("GigE::image_size", uuval);
    if (uuval.size()>=4)
      camera->setImageFormat (uuval[0], uuval[1], uuval[2], uuval[3], mode_cam);
    else
      camera->setImageFormat (mode_cam);
    if (cr.get("GigE::white_balance", uuval))
      camera->setWhiteBalance (uuval[0], uuval[1]);
    else
      camera->setAutoWhiteBalance ();
    if (cr.get("GigE::gain", uval))
      camera->setGain (uval);
    if (cr.get("GigE::exposure", uval))
      camera->setExposure (uval);
    if (cr.get("GigE::auto_exposure", uval))
      camera->setAutoExposure (uval);
    camera->startTransmission();
  }

  void StereoImageSourceGigE::execute() {
    try{
      const ImageBuffer* ib1 = NULL;
      while (true) {
        ib1=camera->getImage(true);
        if (ib1==NULL)
          break;
        ImageBuffer bbbuf;
        switch (mode_cam) {
          case GigEStereoCameraDriver::Mono8:
            switch (mode_app) {
              case GigEStereoCameraDriver::Mono8: 
                bbbuf.image = ib1->image.clone();
                bbbuf.image_right = ib1->image_right.clone();
                break;
              case GigEStereoCameraDriver::BGR24: 
              default: 
                cv::cvtColor(ib1->image, bbbuf.image, CV_GRAY2BGR);
                cv::cvtColor(ib1->image_right, bbbuf.image_right, CV_GRAY2BGR);
                break;
            }
            break;
          case GigEStereoCameraDriver::BGR24:
            switch (mode_app) {
              case GigEStereoCameraDriver::Mono8: 
                cv::cvtColor(ib1->image, bbbuf.image, CV_BGR2GRAY);
                cv::cvtColor(ib1->image_right, bbbuf.image_right, CV_BGR2GRAY);
                break;
              case GigEStereoCameraDriver::BGR24: 
              default: 
                bbbuf.image = ib1->image.clone();
                bbbuf.image_right = ib1->image_right.clone();
                break;
            }
            break;
          case GigEStereoCameraDriver::Bayer8:
            switch (mode_app) {
              case GigEStereoCameraDriver::Mono8: 
                {
                  cv::Mat imp;
                  cv::cvtColor(ib1->image, imp, CV_BayerBG2BGR);
                  cv::cvtColor(imp, bbbuf.image, CV_BGR2GRAY);
                  cv::cvtColor(ib1->image_right, imp, CV_BayerBG2BGR);
                  cv::cvtColor(imp, bbbuf.image_right, CV_BGR2GRAY);
                }
                break;
              case GigEStereoCameraDriver::BGR24: 
              default: 
                cv::cvtColor(ib1->image, bbbuf.image, CV_BayerBG2BGR);
                cv::cvtColor(ib1->image_right, bbbuf.image_right, CV_BayerBG2BGR);
                break;
            }
            break;
        }        

        BBOARD->setImage(bbbuf);
//        Timestamp now;
//        EOUT ("S " << now.get_msec() << std::endl);

        camera->releaseImage (ib1);
        boost::this_thread::interruption_point();
      }
    }catch(boost::thread_interrupted&){;}
  }

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StereoImageSourceGigE> stereoimagesource_gige ("StereoGigE");

} // namespace

