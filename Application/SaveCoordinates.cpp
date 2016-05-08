
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <fstream>
#include <time.h>

using namespace std;

namespace DerWeg {

  /** Speichert Position und Orientierung des Fahrzeugs in eine Datei */
  class SaveCoordinates : public KogmoThread {
      ofstream logfile;
  public:
    SaveCoordinates () {;}

    void init (const ConfigReader& cfg) {

        time_t systemzeit;
        systemzeit = time(0);
        //char *asctime(const struct tm *t);
        string timeString;
        timeString = ctime(&systemzeit);

        logfile.open ("./CoordinateLogs/coord_log_" + timeString);
        //LOUT("Logfile opened");

        logfile << "x" << '\t'
                << "y" << '\t'
                << "phi" << '\t'
                << endl;
    }

    void deinit () {
        logfile.close();
        //LOUT("Logfile closed");
    }

    void execute () {
      try{
        //unsigned int c=0;
        while (true) {
          if (true) {
            //LOUT(BBOARD->getActive() << '\t' << BBOARD->getDesiredVelocity().velocity << '\t' << BBOARD->getDesiredVelocity().steer.get_deg_180() << endl);
            logfile << BBOARD->getVehiclePose().position.x << '\t'
                    << BBOARD->getVehiclePose().position.y << '\t'
                    << BBOARD->getVehiclePose().orientation.get_deg() << '\t'
                    << endl;
          }
          //c++;

          boost::this_thread::sleep(boost::posix_time::milliseconds(100));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::SaveCoordinates> application ("SaveCoordinates");

}


