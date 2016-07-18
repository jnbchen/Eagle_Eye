#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

#include "../Elementary/Vec.h"
#include <vector>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

using namespace std;

namespace DerWeg {

    /** PylonLogging */
    class PylonLogging : public KogmoThread {
        private:
            std::ofstream logfile;
            std::string path;

        public:
            PylonLogging () : path("../data/PylonLogs/") {}
            ~PylonLogging () {}

            void init (const ConfigReader& cfg) {
                time_t systemzeit;
                systemzeit = time(0);
                std::string timeString;
                tm* localt = localtime(&systemzeit);

                std::ostringstream oss;
                oss << setfill('0') << setw(4) << localt->tm_year + 1900 << setw(2) <<  localt->tm_mon +1 << setw(2) << localt->tm_mday << '_'
                    << setw(2) <<  localt->tm_hour << setw(2) << localt->tm_min << setw(2) << localt->tm_sec;
                timeString = oss.str();

                std::ostringstream file_name;
                file_name << path << timeString << ".txt";
                logfile.open(file_name.str());

                logfile << "# x y d a f" << std::endl;
            }

            void deinit () {
                logfile.close();
            }

            void execute() {
                try {
                    while (true) {
                        PylonMeasurements pm = BBOARD->getPylonMeasurements();

                        for (size_t i = 0; i < pm.measurements.size(); ++i) {
                            logfile << pm.measurements[i].position.x << " "
                                    << pm.measurements[i].position.y << " "
                                    << pm.measurements[i].distance << " "
                                    << pm.measurements[i].view_angle << " "
                                    << pm.measurements[i].frame_number << std::endl;

                            std::stringstream anicar_viewer;
                            anicar_viewer << "darkRed plus "
                                          << pm.measurements[i].position.x << " "
                                          << pm.measurements[i].position.y << "\n";
                            BBOARD->addPlotCommand(anicar_viewer.str());
                        }

                        boost::this_thread::sleep(boost::posix_time::milliseconds(50));
                        boost::this_thread::interruption_point();
                    }
                }
                catch (boost::thread_interrupted&) {}
            }
  };


} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::PylonLogging> application_save_stereo_camera_image ("PylonLogging");

}
