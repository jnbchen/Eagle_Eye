
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Gnuplot.h"
#include <fstream>

namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die lediglich den Weg des Fahrzeugs zeigt */
  class ShowLocalization : public KogmoThread {
    Gnuplot gp;
    std::ofstream dlog;
    Pose p0;
    bool started;
  public:
    ShowLocalization () : dlog ("/tmp/kogmolaborpos.log") {;}
    ~ShowLocalization () {;}
    void execute () {
      try{
        started=false;
        while (true) {
          p0=BBOARD->getVehiclePose();
          for (unsigned int i=0; i<10; ++i) {
            Pose p = BBOARD->getVehiclePose ();
            if (p.position!=p0.position) started=true;
            dlog << p.timestamp.get_msec() << '\t' << p.position.x << '\t' << p.position.y << '\t';
            dlog << p.orientation.get_deg_180() << '\t' << p.velocity << '\t' << p.yawrate << std::endl;

            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
            boost::this_thread::interruption_point();
          }
          if (started)
            gp ("plot \"/tmp/kogmolaborpos.log\" u 2:3 notitle w l");
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::ShowLocalization> application_show_localization ("ShowLocalization");

}
