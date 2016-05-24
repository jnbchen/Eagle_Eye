#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Angle.h"
#include "BezierCurve.h"
#include "math.h"
#include "iostream"

using namespace std;

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
        double lastProjectionParameter;
        BezierCurve bc;

        double newton_tolerance;
        int newton_max_iter;

        double stanley_k0, stanley_k1;
        double axis_distance;

    public:
        LateralControl () : lastProjectionParameter(0) {;}
        ~LateralControl () {;}


	void init(const ConfigReader& cfg) {
        cfg.get("LateralControl::newton_tolerance", newton_tolerance);
        cfg.get("LateralControl::newton_max_iter", newton_max_iter);
        cfg.get("LateralControl::stanley_k0", stanley_k0);
        cfg.get("LateralControl::stanley_k1", stanley_k1);
        cfg.get("LateralControl::axis_distance", axis_distance);
	}

        //ControllerInput calculate_curve_data(const BezierCurve& bc, const State& state);

        void execute () {
          try{
            while (true) {

              //If path changed set estimate for newton algo to zero, else use the previous result
              if (bc != BBOARD->getReferenceTrajectory().path) {
                  //LOUT("New curve detected, by LateralControl" << endl);
                bc = BBOARD->getReferenceTrajectory().path;
                lastProjectionParameter = 0;
              }

              ControllerInput input = calculate_curve_data(BBOARD->getState());


              //Stanley-Controller here
              double u = input.curvature - stanley_k0 * input.distance - stanley_k1 * input.diff_angle.get_rad_pi();
              double delta = atan(axis_distance * u);

              //Set steering angle, keep velocity
              Velocity dv = BBOARD->getDesiredVelocity();
              dv.steer = Angle::rad_angle(delta);
              BBOARD->setDesiredVelocity(dv);

              boost::this_thread::sleep(boost::posix_time::milliseconds(20));
              boost::this_thread::interruption_point();
            }
          }catch(boost::thread_interrupted&){;}
        }

        /* Calculates the distance from the current position to the bezier curve, the angle of the vehicle with respect to the curve,
        and the curvature of the curve. Those values are needed for the controller. */
        ControllerInput calculate_curve_data(const State& state)  {
            lastProjectionParameter = bc.project(state.position, lastProjectionParameter,
                                                 newton_tolerance, newton_max_iter);
            //LOUT("Projected Parameter: " << lastProjectionParameter << endl);

            //evaluate bezier curve and derivatives at the projection parameter
            Vec f = bc(lastProjectionParameter);
            Vec df = bc.prime(lastProjectionParameter);
            Vec ddf = bc.double_prime(lastProjectionParameter);

            //Calculate distance form current position to curve
            //The difference vector is normal to the tangent in the projection point
            //We use this to assign a sign to the distance:
            //The distance is positive, if the actual position is left of the curve
            // and negative if it's right of the curve (regarding moving direction)
            Vec diff = state.position - f;
            double distance = diff.length();
            //If the point is right of df, let the distance have a negative sign
            if (diff * df.rotate_quarter() < 0) {
                distance *= -1;
            }

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

            /*
            LOUT("Distance: " << distance << endl);
            LOUT("Diff_Angle: " << diff_angle.get_deg_180() << endl);
            LOUT("Curvature: " << curvature << endl);
            LOUT("Position: " << state.position.x << ", " << state.position.y << endl);
            */

            return ControllerInput(distance, diff_angle, curvature);
        }

  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::LateralControl> application ("LateralControl");

}
