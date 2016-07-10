#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Vec.h"

namespace DerWeg {

  /** StateEstimator */
  class StateEstimator : public KogmoThread {
  private:
    //difference between front and rear axis
    double rear_offset;
  public:
    StateEstimator () {;}
    ~StateEstimator () {;}

    void init(const ConfigReader& cfg) {
      cfg.get ("StateEstimator::rear_offset", rear_offset);
    }

    void execute () {
      BBOARD->waitForOdometry();
      BBOARD->waitForVehiclePose();
      try{
        while (true) {

            State state;
            Pose pose = BBOARD->getVehiclePose();
            Odometry odometry = BBOARD->getOdometry();

            state.sg_position = pose.position;
            //Calculate position of rear axis center
            state.rear_position = pose.position - rear_offset * Vec(1,0).rotate(pose.orientation);
            state.control_position = state.rear_position;
            state.stddev = pose.stddev;
            state.orientation = pose.orientation;
            state.velocity = pose.velocity;
            state.yawrate = pose.yawrate;
            state.timestamp = pose.timestamp;

            state.velocity_tire = odometry.velocity;
            state.steer = odometry.steer;

            BBOARD->setState(state);

            //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
            boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::StateEstimator> application ("StateEstimator");

}
