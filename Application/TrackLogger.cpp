
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/ImageReaderWriter.h"

#include "opencv/highgui.h"

namespace DerWeg {

  /** an application that writes camera images to file */
  class TrackLogger : public KogmoThread {
    ImageWriter writer;
    ImageBuffer ib;
    std::ostream *outputStream;
  public:
    TrackLogger () {
      this->outputStream = new std::ofstream("tracklog.txt");
    }
    ~TrackLogger () {
      this->outputStream->flush();
      delete this->outputStream;
    }
    void init (const ConfigReader& cfg) {
      //std::string pf="image";
      //cfg.get ("SaveCameraImage::image_prefix", pf);
      //writer.setPrefix(pf);
    }
    void execute () {
      try{
        unsigned int imCounter = 0;
        while (true) {
          BBOARD->waitForTopImage();
          if(!(BBOARD->getTopImage().is_empty()))
          {
              cv::Mat Image = BBOARD->getTopImage().image;
              char FileName[20];
              sprintf(FileName, "./im/%06d.png", imCounter++);
              cv::imwrite(FileName, Image);
              // get current pose from blackboard
              Pose vehiclePose;
              vehiclePose = BBOARD->getVehiclePose();

              Vec currentPosition = vehiclePose.position;
              Angle currentOrientation = vehiclePose.orientation;
              double currentVelocity = vehiclePose.velocity;
              double currentYawRate = vehiclePose.yawrate;
              Timestamp currentTimestamp = vehiclePose.timestamp;
              long int currentTime = currentTimestamp.get_msec();

              // store current pose (IRGENDIE STEHT DA NIX DRIN!!!)
              *outputStream << currentTime << " " << currentPosition.x << " " << currentPosition.y << " " << currentOrientation.get_deg() << " " << currentVelocity << " " << currentYawRate << " " << FileName << std::endl;
          }
          boost::this_thread::interruption_point();
          //usleep (100000);
        }
//        fclose(test);
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TrackLogger> application_log_track ("TrackLogger");
}
