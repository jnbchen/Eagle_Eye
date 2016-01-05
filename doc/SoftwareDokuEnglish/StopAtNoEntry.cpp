#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../ImageProcessing/TrafficSignDetector.h"

namespace DerWeg {
  /** A very simple example and testing application that
      stops the car before a No-Entry sign */
  class StopAtNoEntry : public KogmoThread {
    TrafficSignDetector detector;
  public:
    StopAtNoEntry () {;}
    ~StopAtNoEntry () {;}
    void execute () {
      try{
        Velocity dv;
        dv.steer = Angle::deg_angle(0);
        while (true) { // Infinite Loop
          ImageBuffer ib = BBOARD->getImage(); // gets Image
          if (ib.image.empty()) { // does not retrieve an image, then stops.
            dv.velocity=0;
          } else {
            dv.velocity=0.5;
            std::vector<DerWeg::TrafficSign> signs_detected = 
                detector.process (ib.image); // Looks for street signs on the image.
            for (unsigned int i=0; i<signs_detected.size(); ++i) {
              // Browses through all recognized street signs.
              if (signs_detected[i].type==DerWeg::NO_ENTRY) {
                // Recognizes a No-Entry sign.
                dv.velocity=0;
              }
            }
          }
          BBOARD->setDesiredVelocity(dv); // sets desired velocity

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };
} // namespace DerWeg

namespace {
  // Registers plugin in the factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, 
     DerWeg::StopAtNoEntry> application ("StopAtNoEntry");
}
