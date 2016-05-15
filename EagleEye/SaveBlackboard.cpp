
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <fstream>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <string.h>

using namespace std;

namespace DerWeg {

  /** Speichert Position, Orientierung, Geschwindigkeit und Lenkwinkel des Fahrzeugs in eine Datei */
  class SaveBlackboard : public KogmoThread {
      ofstream logfile;
  public:
    SaveBlackboard () {;}

    void init (const ConfigReader& cfg) {

        time_t systemzeit;
        systemzeit = time(0);
        //char *asctime(const struct tm *t);
        string timeString;
        //timeString = ctime(&systemzeit);
        tm * localt = localtime(&systemzeit);

        ostringstream oss;
        oss << setfill('0') << setw(4) << localt->tm_year + 1900 << '-' << setw(2) <<  localt->tm_mon +1 << '-' << setw(2) << localt->tm_mday << '_'
            << setw(2) <<  localt->tm_hour << '-' << setw(2) << localt->tm_min << '-' << setw(2) << localt->tm_sec;
        timeString = oss.str();

        char buf[100];
        strcpy(buf, "../data/CoordinateLogs/coord_log_");
        strcat(buf, timeString.c_str());
        logfile.open (buf);

        //logfile.open ("../data/CoordinateLogs/coord_log_" + timeString);
        //LOUT("Logfile opened");

        logfile << "active" << '\t'
                << "x" << '\t'
                << "y" << '\t'
                << "phi" << '\t'
                << "v_soll" << '\t'
                << "v_ist" << '\t'
                << "delta_soll" << '\t'
                << "delta_ist" << '\t'
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
            logfile << BBOARD->getActive() << '\t'
                    << BBOARD->getVehiclePose().position.x << '\t'
                    << BBOARD->getVehiclePose().position.y << '\t'
                    << BBOARD->getVehiclePose().orientation.get_deg() << '\t'
                    << BBOARD->getDesiredVelocity().velocity << '\t'
                    << BBOARD->getOdometry().velocity << '\t'
                    << BBOARD->getDesiredVelocity().steer.get_deg_180()
                    << BBOARD->getOdometry().steer.get_deg_180() << '\t'
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
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::SaveBlackboard> application ("SaveBlackboard");

}

