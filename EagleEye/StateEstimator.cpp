#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Vehicle/VehicleKinematics.h"
#include "../Elementary/Vec.h"
#include "../Elementary/Eigen/Dense"
#include "UKF.h"
#include <cmath>

namespace DerWeg {

  /** StateEstimator */
  class StateEstimator : public KogmoThread {
  private:
    //difference between front and rear axis
    double rear_offset;

    State state;

    VehicleKinematics kin;
    Vec position;
    Angle heading;
    double velocity;
    Angle steering_angle;
    Timestamp timestamp;
    ukf UKF;
    VectorXd raw_measurements_；


  public:
    StateEstimator () : kin(0, 527) {;}
    ~StateEstimator () {;}

    void init(const ConfigReader& cfg) {
      cfg.get ("StateEstimator::rear_offset", rear_offset);
    }

    void execute () {
      BBOARD->waitForOdometry();
      BBOARD->waitForVehiclePose();
      try{
        while (true) {
            Pose pose = BBOARD->getVehiclePose();
            Odometry odo = BBOARD->getOdometry();

            // use Unscented Kalman Filter
            raw_measurements_ = VectorXd(3)；
            raw_measurements_(0) = pose.position.x;
            raw_measurements_(1) = pose.position.y;
            raw_measurements_(2) = pose.orientation;

            UKF.ProcessMeasurement(raw_measurements_);

            double p_x = UKF.x_(0);
            double p_y = UKF.x_(1);
            double v = UKF.x_(2);
            double yaw = UKF.x_(3);
            double yawrate = UKF.x_(4);

            Timestamp now;

            pose.position = Vec(p_x, p_y);
            pose.orientation = yaw;
            pose.velocity = v;
            pose.yawrate=yawrate;
            pose.timestamp=now;
            pose.stddev = 0;

            //Calculate position of rear axis center
            state.sg_position = pose.position;
            state.orientation = pose.orientation;
            state.rear_position = state.sg_position - rear_offset * Vec(1,0).rotate(state.orientation);
            state.control_position = state.rear_position;
            state.stddev = pose.stddev;

            state.velocity = pose.velocity;
            state.yawrate = pose.yawrate;
            state.timestamp = pose.timestamp;

            state.velocity_tire = odo.velocity;
            state.steer = odo.steer;

            if (!(0 <= state.velocity && state.velocity <= 2)) {
                //EOUT("Error in Velocity Estimation: state.velocity = " << state.velocity << " \n");
            }
            if (!(0 <= state.velocity_tire && state.velocity_tire <= 2)) {
                //EOUT("Error in Velocity Estimation: state.velocity_tire = " << state.velocity_tire << " \n");
            }

            BBOARD->setState(state);

            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
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
