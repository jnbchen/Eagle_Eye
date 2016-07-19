#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Vehicle/VehicleKinematics.h"
#include "../Elementary/Vec.h"

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

            state.orientation = pose.orientation;
            Timestamp now;

            //if (BBOARD->getOnTrack()) {
            if (true) {

                state.sg_position = pose.position;

            } else {

                Angle old_heading = heading;
                double difftime = 1e-3*now.diff_usec(timestamp);

                kin.heun_step(state.sg_position, state.orientation, velocity, odo.velocity, steering_angle, odo.steer, difftime);
                velocity=odo.velocity;
                double diffangle = heading.get_rad()-old_heading.get_rad();
                if (diffangle>=2*M_PI)
                diffangle-=2*M_PI;
                else if (diffangle<=-2*M_PI)
                diffangle+=2*M_PI;
                steering_angle=odo.steer;

                //pose.position=position;
                //pose.orientation=heading;
                pose.velocity=velocity;
                pose.yawrate=diffangle/difftime*1e3;
                pose.timestamp=now;
                pose.stddev = 0;

                // Plot in AnicarViewer
                std::stringstream plt;
                plt << "thick green dot "
                << state.sg_position.x << " " << state.sg_position.y << "\n";
                BBOARD->addPlotCommand(plt.str());
            }

            timestamp=now;

            //Calculate position of rear axis center
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
