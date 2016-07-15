#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

#include "ConvexPolygon.h"

using namespace std;

namespace DerWeg {

  /** IndicatorControl */
  class IndicatorControl : public KogmoThread {

    private:
        ConvexPolygon intersection_region;
        ConvexPolygon turnout_region;

    public:
        IndicatorControl () {;}
        ~IndicatorControl () {;}


	void init(const ConfigReader& cfg) {
        vector<double> tmp1;
	}


    void execute () {
      try{
        LOUT("Modul Indicator\n");
        while (true) {

            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::IndicatorControl> application ("IndicatorControl");

}
