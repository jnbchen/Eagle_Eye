
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "StarGazer.hpp"
#include <fstream>
#include <sys/stat.h>

using namespace std;
using namespace DerWeg;

namespace DerWeg {

  class StarGazerOffsetCalibration : public KogmoThread {
  private:
    StarGazer* sg;
    std::ofstream* logfile;
    int wait_cycles; // 0, wenn bereits aktiv war, ansonsten zum Ignorieren der ersten 10 Beobachtungen
  public:
    StarGazerOffsetCalibration ();
    ~StarGazerOffsetCalibration ();
    void execute ();
    void init (const ConfigReader&);
  };

}



StarGazerOffsetCalibration::StarGazerOffsetCalibration () :
  sg(NULL), logfile (NULL), wait_cycles (10)
{;}

StarGazerOffsetCalibration::~StarGazerOffsetCalibration () {
  if (sg) {
    delete sg;
  }
  if (logfile) {
    (*logfile) << std::flush;
    delete logfile;
  }
}

void StarGazerOffsetCalibration::init (const ConfigReader& cfg) {
  std::string device;
  cfg.get("StarGazer::device", device);
  try{
    struct stat file_info;
    int r = stat(device.c_str(),&file_info);
    if (r!=0)
      throw runtime_error (string("openStarGazer: device ")+device+string(" does not exist."));
    if (!S_ISCHR(file_info.st_mode))
      throw runtime_error (string("openStarGazer: device ")+device+string(" is not a character device."));
    if ((file_info.st_mode & 6) != 6)
      throw runtime_error (string("openStarGazer: device ")+device+string(" does not have read/write access."));
    sg = new StarGazer (device);
  }catch(std::runtime_error& e){
    EOUT(e.what() << endl);
    sg=NULL;
  }
  logfile = new std::ofstream ("StarGazerOffsetCalibration.dat");
}

void StarGazerOffsetCalibration::execute () {
  if (!sg || !logfile) return;
  try{
    while(true) {
      StarGazer::PositionData pd = sg->get_position();
      bool is_active = BBOARD->getActive ();
      if (!is_active) {
        if (wait_cycles==0) {
          (*logfile) << '\n';
          LOUT ('\n');
        }
        wait_cycles = 10;
      } else if (wait_cycles>0) {
        --wait_cycles;
      }
      if (wait_cycles==0 && !pd.dead) {
        (*logfile) << pd.id << '\t' << pd.x << '\t' << pd.y << '\t' << pd.z << '\t' << pd.theta << '\n';
        LOUT (pd.id << '\t' << pd.x << '\t' << pd.y << '\t' << pd.z << '\t' << pd.theta << '\n');
      } else if (pd.dead){
        LOUT ("dead");
      } else {
        LOUT (pd.id);
      }

      boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      boost::this_thread::interruption_point();
    }
  }catch(boost::thread_interrupted&){;}
}



namespace {
  // Anmelden bei der Plugin-Factory
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StarGazerOffsetCalibration> app ("StarGazerOffsetCalibration");
}
