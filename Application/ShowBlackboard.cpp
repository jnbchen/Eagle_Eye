
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <iostream>

using namespace std;

namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die lediglich den Weg des Fahrzeugs zeigt */
  class ShowBlackboard : public KogmoThread {
  public:
    void execute () {
      try{
        unsigned int c=0;
        while (true) {
          if (c%10==0) {
            LOUT(BBOARD->getActive() << '\t' << BBOARD->getDesiredVelocity().velocity << '\t' << BBOARD->getDesiredVelocity().steer.get_deg_180() << endl);
          }
          c++;

          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ShowBlackboard> application ("ShowBlackboard");

}
