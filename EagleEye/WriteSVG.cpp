
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include <iostream>
#include <sstream>
#include <fstream>

namespace DerWeg {

  /** */
  class WriteSVG : public KogmoThread {
    private:
        std::string line_end;
        std::fstream outfile1;
        std::stringstream sstream1;
        long pos;

  public:

    void init(const ConfigReader& cfg){
        std::ifstream infile1;
        infile1.open("../data/PositionSVG/svg_beg_prototype");
        std::string line_beg;
        std::getline(infile1,line_beg);
        std::getline(infile1,line_end);
        infile1.close();

        outfile1.open("../data/PositionSVG/Position.svg", std::ios::in | std::ios::out | std::ios::trunc);
        outfile1<<line_beg<<"\n";
        pos = outfile1.tellp();
        outfile1<<line_end;
        outfile1.close();
    }


    void execute () {
      try{
        unsigned int c=0;
        while (true) {
            sstream1<<BBOARD->getVehiclePose().position.x << ",\t"<< BBOARD->getVehiclePose().position.y << "\n";
          if (c%5==0) {
            outfile1.open("../data/PositionSVG/Position.svg",std::ios::in | std::ios::out);
            outfile1.seekp(pos,std::ios::beg);
            outfile1<<sstream1.rdbuf();
            pos=outfile1.tellp();
            outfile1<<line_end;
            outfile1.close();
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
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::WriteSVG> application ("WriteSVG");

}
