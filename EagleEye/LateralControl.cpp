#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"

namespace DerWeg {

  /** LateralControl */
  class LateralControl : public KogmoThread {
  public:
    LateralControl () {;}
    ~LateralControl () {;}
    void execute () {
      try{
        while (true) {

          // to be implemented ...

          /*

          // Struct to contain the input parameter for lateral control
            struct ControlInput {
                double distance;        ///< Distance to reference curve
                Angle diff_angle;        ///< Angle difference between ego and curve
                double curvature;       ///< Curvature of reference curve
            }


          ControlInput control_input(const BezierCurve& bc, const State& state) const throw() {
            // what eevaaaaaa ...
            }

          if (bc == BBOARD->getReferenceCurve().path) {
            new_start = t;
          }
          else {
            bc = BBOARD->getReferenceCurve().path;
            new_start = 0;
          }
          */

          //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::LateralControl> application ("LateralControl");

}
