#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  /** StateMachine */
  class StateMachine : public KogmoThread {

      private:
        int segment_id;

  public:
    StateMachine () {;}
    ~StateMachine () {;}


    void init(const ConfigReader& cfg) {
        LOUT("Exec Statemachine init()" << "\n");

        cfg.get("StateMachine::segment_id", segment_id);

        /** Write segment ID to BLackBoard */
        DrivingMode dm;
        dm.current_mode = segment_id;
        dm.next_mode = 0;
        BBOARD->setDrivingMode(dm);
    }


    void execute() {
      try{
        while (true) {

            //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
            boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StateMachine> application ("StateMachine");

}
