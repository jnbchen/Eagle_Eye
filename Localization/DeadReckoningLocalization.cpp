
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Vehicle/VehicleKinematics.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  class DeadReckoningLocalization : public KogmoThread {
    VehicleKinematics kin;
    Vec position;
    Angle heading;
    double velocity;
    Angle steering_angle;
    Timestamp timestamp;
  public:
    DeadReckoningLocalization () : kin (0, 527), velocity(0) {;}
    ~DeadReckoningLocalization () {;}
    void init (const ConfigReader& cfg) {
      std::vector<double> x;
      cfg.get ("DeadReckoning::init", x);
      if (x.size()>=2) {
        position = Vec (x[0], x[1]);
        if (x.size()>=3) {
          heading = Angle::deg_angle (x[2]);
        }
      }
    }
    void execute () {
      try{
        while (true) {
          Odometry odo = BBOARD->getOdometry ();
          Timestamp now;
          Angle old_heading = heading;
          double difftime = 1e-3*now.diff_usec(timestamp);
          kin.heun_step (position, heading, velocity, odo.velocity, steering_angle, odo.steer, difftime);
          velocity=odo.velocity;
          double diffangle = heading.get_rad()-old_heading.get_rad();
          if (diffangle>=2*M_PI)
            diffangle-=2*M_PI;
          else if (diffangle<=-2*M_PI)
            diffangle+=2*M_PI;
          steering_angle=odo.steer;
          timestamp=now;
          Pose vp;
          vp.position=position;
          vp.orientation=heading;
          vp.velocity=velocity;
          vp.yawrate=diffangle/difftime*1e3;
          vp.timestamp=now;
          BBOARD->setVehiclePose (vp);
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::DeadReckoningLocalization> globalposition_dr ("DeadReckoning");

} // namespace
