#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

#include "ConvexPolygon.h"
#include "Lights.h"

using namespace std;

namespace DerWeg {

  /** IndicatorControl */
  class IndicatorControl : public KogmoThread {

    private:
        ConvexPolygon intersection_region;
        ConvexPolygon turnout_region;
        Lights lights;

    public:
        IndicatorControl () {;}
        ~IndicatorControl () {;}


	void init(const ConfigReader& cfg) {
        vector<double> tmp1;
        cfg.get("IndicatorControl::intersection_region", tmp1);
        intersection_region = ConvexPolygon(tmp1);

        vector<double> tmp2;
        cfg.get("IndicatorControl::turnout_region", tmp2);
        turnout_region = ConvexPolygon(tmp2);
	}


    void execute () {
      try{
        LOUT("Modul Indicator\n");
        lights.indicator_off();
        boost::this_thread::sleep(boost::posix_time::milliseconds(300));
        while (true) {
            bool indicator_on = false;

            Vec pos = BBOARD->getState().sg_position;

            if (intersection_region.isInside(pos)) {
                switch (BBOARD->getReferenceTrajectory().segment_id) {
                    case 11:
                    case 23:
                    case 32:
                    case 44:
                        lights.left_indicator_on();
                        indicator_on = true;
                        break;
                    case 14:
                    case 22:
                    case 33:
                    case 41:
                        lights.right_indicator_on();
                        indicator_on = true;
                        break;
                }
            }
            if (turnout_region.isInside(pos)) {
                switch (BBOARD->getReferenceTrajectory().segment_id) {
                    case 15:
                    case 25:
                    case 35:
                    case 45:
                        lights.right_indicator_on();
                        indicator_on = true;
                        break;
                }
            }
            if (!BBOARD->getOnTrack()) {
                lights.hazard_lights_on();
                indicator_on = true;
            }
            if (!indicator_on) {
                lights.indicator_off();
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(200));
            boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){lights.indicator_off();}
    }

    void deinit () {
        lights.indicator_off();
    }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::IndicatorControl> application ("IndicatorControl");

}
