#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Angle.h"
#include "BezierCurve.h"
#include "math.h"

namespace DerWeg {

    struct ControllerInput {
        double distance;        ///< Distance to reference curve
        Angle diff_angle;        ///< Angle difference between ego and curve
        double curvature;       ///< Curvature of reference curve

        ControllerInput(double dist, Angle angle, double curv) : distance(dist), diff_angle(angle), curvature(curv) {;} /// Constructor
    };



  /** LateralControl */
  class LateralControl : public KogmoThread {

    private:
        double lastProjectionParameter = 0;
        BezierCurve bc;

    public:
        LateralControl () {;}
        ~LateralControl () {;}

        //ControllerInput calculate_curve_data(const BezierCurve& bc, const State& state);

        void execute () {
          try{
            while (true) {

              if (bc != BBOARD->getReferenceTrajectory().path) {
                bc = BBOARD->getReferenceTrajectory().path;
                lastProjectionParameter = 0;
              }

              ControllerInput input = calculate_curve_data(BBOARD->getState());

              /*
              Stanley-Controller here
              */

              //boost::this_thread::sleep(boost::posix_time::milliseconds(20));
              boost::this_thread::interruption_point();
            }
          }catch(boost::thread_interrupted&){;}
        }

        /* Calculates the distance from the current position to the bezier curve, the angle of the vehicle with respect to the curve,
        and the curvature of the curve. Those values are needed for the controller. */
        ControllerInput calculate_curve_data(const State& state)  {
            lastProjectionParameter = bc.project(state.position, lastProjectionParameter);

            //evaluate bezier curve and derivatives at the projection parameter
            Vec f = bc(lastProjectionParameter);
            Vec df = bc.prime(lastProjectionParameter);
            Vec ddf = bc.double_prime(lastProjectionParameter);

            //Calculate distance form current position to curve
            Vec diff = f - state.position;
            double distance = diff.length();

            //Calculate difference angle between vehicle and curve
            double phi;
            if (df.x != 0 && abs(df.y/df.x) < 1) {
                //atan2 takes the direction into account
                phi = atan2(df.y, df.x);
            } else {
                //avoid singularities by rotating coordinate system for 2nd and 4th quadrant
                phi = M_PI/2 + atan2(-df.x, df.y);
            }
            Angle diff_angle = Angle::rad_angle(phi) - state.orientation;

            //Calculate the curvature of the bezier curve
            double curvature_numerator = ddf.y * df.x - ddf.x * df.y;
            double curvature_denominator = pow(df.squared_length(), 3.0/2);
            double curvature = curvature_numerator / curvature_denominator;

            return ControllerInput(distance, diff_angle, curvature);
        }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::LateralControl> application ("LateralControl");

}
