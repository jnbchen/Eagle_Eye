#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../ImageProcessing/TrafficSignDetector.h"

namespace DerWeg {
  /** eine sehr einfache Beispiel- und Testanwendung, die bei einem 
   * Einfahrt-Verboten-Schild anhält */
  class StopAtNoEntry : public KogmoThread {
    TrafficSignDetector detector;
  public:
    StopAtNoEntry () {;}
    ~StopAtNoEntry () {;}
    void execute () {
      try{
        Velocity dv;
        dv.steer = Angle::deg_angle(0);
        while (true) { // Endlosschleife
          ImageBuffer ib = BBOARD->getImage(); // Bild holen
          if (ib.image.empty()) { // kein Bild bekommen, daher anhalten
            dv.velocity=0;
          } else {
            dv.velocity=0.5;
            std::vector<DerWeg::TrafficSign> signs_detected = 
                detector.process (ib.image); // Schilder im Bild suchen
            for (unsigned int i=0; i<signs_detected.size(); ++i) {
              // alle erkannten Schilder durchgehen
              if (signs_detected[i].type==DerWeg::NO_ENTRY) {
                // Einfahrt verboten-Schild erkannt
                dv.velocity=0;
              }
            }
          }
          BBOARD->setDesiredVelocity(dv); // setze Sollgeschwindigkeit

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };
} // namespace DerWeg

namespace {
  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, 
     DerWeg::StopAtNoEntry> application ("StopAtNoEntry");
}
