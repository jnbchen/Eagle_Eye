#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Angle.h"

#include "BezierCurve.h"
#include "PathPlanning.h"
#include "Lights.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

using namespace std;

namespace DerWeg {

    struct ControllerInput {
        double distance;        ///< Distance to reference curve
        Angle diff_angle;        ///< Angle difference between ego and curve
        double curvature;       ///< Curvature of reference curve

        ControllerInput(double dist, Angle angle, double curv) : distance(dist), diff_angle(angle), curvature(curv) {;} /// Constructor
        std::ostream *outputStream;
    };



  /** LateralControl */
  class LateralControl : public KogmoThread {


    private:
        double lastProjectionParameter;
        BezierCurve bc;

        double newton_tolerance;
        int newton_max_iter;

        double precontrol_k;
        double stanley_k0, stanley_k1;
        double axis_distance;
        // Determines whether Stanley controller is used, or matlab QP
        //bool use_stanley;

        double v_max;
        double v_min;
        double a_lateral_max;
        // curvature, we always assume to have -> prevents the car to drive move than a certain velocity and
        // and also prevents dividing by zero
        double virtual_min_kappa;

        //if set to one, set velocity manually, if set to zero, car automatically accelerates
        bool manual_velocity;

        double VEL_THRESH;
        double v_diff_max;
        int min_percentage;

        Lights brake_lights;

        PathPlanning pylon_planner;

        //logging file
        std::ostream *outputStream;

        /*//MONITORING/////////////////////////////////////////////////////
        std::string svg_end="</g> </svg>";
        std::fstream outfile1;
        std::stringstream sstream1;
        long pos;
        //END/////////////////////////////////////////////////*/

    public:
        LateralControl () : lastProjectionParameter(0) {
            this->outputStream = new std::ofstream("lateral_control_log.txt");
      }
        ~LateralControl () {
            this->outputStream->flush();
            delete this->outputStream;
      }


	void init(const ConfigReader& cfg) {
        cfg.get("LateralControl::newton_tolerance", newton_tolerance);
        cfg.get("LateralControl::newton_max_iter", newton_max_iter);
        cfg.get("LateralControl::precontrol_k", precontrol_k);
        cfg.get("LateralControl::stanley_k0", stanley_k0);
        cfg.get("LateralControl::stanley_k1", stanley_k1);
        cfg.get("LateralControl::axis_distance", axis_distance);
        //cfg.get("LateralControl::use_stanley", use_stanley);

        cfg.get("LongitudinalControl::v_max", v_max);
        cfg.get("LongitudinalControl::v_min", v_min);
        cfg.get("LongitudinalControl::a_lateral_max", a_lateral_max);
        cfg.get("LongitudinalControl::manual_velocity", manual_velocity);

        virtual_min_kappa = a_lateral_max / pow(v_max, 2);

        cfg.get("BrakeLights::VEL_THRESH", VEL_THRESH);
        cfg.get("BrakeLights::v_diff_max", v_diff_max);
        cfg.get("BrakeLights::min_percentage", min_percentage);

        pylon_planner = PathPlanning(cfg);


        /*//MONITORING//////////////////////////////////////////////////////
        std::ifstream infile1;
        infile1.open("../preprocessing/monitoring_prototype.svg");
        std::string svg_beg;
        while (std::getline(infile1,svg_beg,)){;}
        infile1.close();

        outfile1.open("../preprocessing/monitoring.svg", std::ios::in | std::ios::out | std::ios::trunc);
        outfile1<<svg_beg<<"\n";
        pos = outfile1.tellp();
        outfile1<<svg_end;
        outfile1.close();
        //END/////////////////////////////////////////////////////////////////*/
	}


    void execute () {
      try{
          Timestamp tinit;

          /*//MONITORING///////////////////////////////////////////////////////
          unsigned int c=0;
          //END/////////////////////////////////////////////////////////////////*/

        while (true) {
          if (BBOARD->getOnTrack()){

            //If path changed set estimate for newton algo to zero, else use the previous result
            if (bc != BBOARD->getReferenceTrajectory().path) {
                //LOUT("New curve detected, by LateralControl" << endl);
                bc = BBOARD->getReferenceTrajectory().path;
                lastProjectionParameter = 0;
            }

            int seg_id = BBOARD->getReferenceTrajectory().segment_id;

            //get current steering angle and velocity
            Velocity dv = BBOARD->getDesiredVelocity();

            // Lateral control ======================================================
            State s = BBOARD->getState();
            ControllerInput input = calculate_curve_data(s);
            //Stanley-Controller here
            double u = precontrol_k * input.curvature - stanley_k0 * input.distance - stanley_k1 * input.diff_angle.get_rad_pi();
            double delta = atan(axis_distance * u);

            // set steering angle
            dv.steer = Angle::rad_angle(delta);

            if (abs(dv.steer.get_deg_180()) > 28) {
                //LOUT("Steering angle = " << dv.steer.get_deg_180() << "\n");
            }

            //Velocity control
            double max_velocity;
            if (!manual_velocity) {
              // calculate maximal velocity from curvature
              double kappa = max(virtual_min_kappa, abs(BBOARD->getReferenceTrajectory().curvature_lookahead * 1000));
              // multiply with 1000, because the curvature has units 1/mm

              // get maximal velocity for the current curvature to not exceed given lateral acceleration
              max_velocity = max(v_min, pow(a_lateral_max / kappa, 0.5));
            } else {
              // if velocity is set manually, use last velocity from blackboard
              max_velocity = dv.velocity;
              //LOUT("max_v = "<<max_velocity<<std::endl);
            }

            double v_max_tl =  BBOARD->getReferenceTrajectory().v_max_tl;
            // Get v_max from TrajectoryGenerator (could be reduced because of a traffic light)
            double set_velocity = min(max(0.0, min(max_velocity, v_max_tl)), s.velocity_tire + 0.3);
            //LOUT("set_velocity" << set_velocity<<"\n");

            //Brake lights:
            double diff_velocity = set_velocity - dv.velocity;
            if (abs(diff_velocity) > VEL_THRESH) {
                // Only these are relevant velocity changes
                if (diff_velocity > 0) {
                    // Case set_velocity > old_velocity
                    brake_lights.brake_light_off();
                } else {
                    // Case set_velocity < old_velocity
                    const double percentage = - diff_velocity / v_diff_max;
                    const double scaled_percentage = min_percentage + percentage * (100 - min_percentage);
                    const int perc = min(floor(scaled_percentage + 0.5), 100.0);
                    brake_lights.brake_light_on(perc);
                    //LOUT("Percentage brake lights = " << perc << endl);
                }
            }

            double x = s.sg_position.x;
            if (seg_id == 41 && x >6000 && x<7000 ||
                seg_id == 13 && x >5000 && x<6000 ||
                seg_id == 32 && x >8000 && x<9000 ||
                seg_id == 25 && x >3000 && x<4000) {
                    set_velocity *= 0.7;
            }

            dv.velocity = set_velocity;

            //LOUT("max_velocity = " << max_velocity << endl);
            //LOUT("Ref-curve: v_max = " << BBOARD->getReferenceTrajectory().v_max << endl);
            //LOUT("dv.velocity = " << dv.velocity<<endl);
            if (v_max_tl < 0) {
                LOUT("v_max_tl = " << v_max_tl << endl);
            }

            // set steering angle and velocity
            BBOARD->setDesiredVelocity(dv);





            //State s = BBOARD->getState();
            Vec currentPosition = s.control_position;
            Angle currentOrientation = s.orientation;
            double currentVelocity = s.velocity_tire;

            Timestamp currentTimestamp = s.timestamp;
            long int currentTime = currentTimestamp.get_msec();

            *outputStream << currentTime <<" "
                        << static_cast<double>(tinit.elapsed_msec()) << " "
                        << currentPosition.x <<" "
                        << currentPosition.y <<" "
                        << currentOrientation.get_deg()<<" "
                        << currentVelocity << " "
                        << s.steer.get_deg_180() << " "
                        << input.distance <<  " "
                        << input.diff_angle.get_deg_180()<<" "
                        << input.curvature*1000 << " "
                        << - stanley_k0 * input.distance*1000 <<  " "
                        << - stanley_k1 * input.diff_angle.get_rad_pi()*1000 <<  " "
                        <<  u*1000  << " "
                        << delta << " "
                        << std::endl;


          } else {
            // OBSTACLE PATH PLANNING HERE

            // Stop vehicle
            Velocity dv;
            //dv.velocity = 0;
            //dv.steer = Angle::rad_angle(0);

            dv = pylon_planner.findPath(BBOARD->getPylonMap().circles);

            BBOARD->setDesiredVelocity(dv);
          }


          /*//MONITORING//////////////////////////////////////////////////////////////
          sstream1<<"use x=\""<<getVehiclePose().position.x<<"\" y=\" "<<getVehiclePose().position.y<<" \" xlink:href=\"#car\" transform=\"rotate("<< BBOARD->getVehiclePose().orientation.get_deg()  <<","<< BBOARD->getVehiclePose().position.x <<"," << BBOARD->getVehiclePose().position.y "\")\"/> \n";
          sstream1<<"<polyline points=\""<<getVehiclePose().position.x<<","<<getVehiclePose().position.y<<" "<<getProjPos().x<<","<<getProjPos().y<<" \" style=\"fill:none;stroke:black;stroke-opacity:0.1;stroke-width:10\" marker-start=\"url(#M_dot)\" marker-end=\"url(#M_dot)\"/> \n";

          if (c%1==0) {
            outfile1.open("../preprocessing/monitoring.svg",std::ios::in | std::ios::out);
            outfile1.seekp(pos,std::ios::beg);
            outfile1<<sstream1.rdbuf();
            pos=outfile1.tellp();
            outfile1<<"<use x=\"0\" y=\"0\" xlink:href=\"seg.svg#segment"<<ID<<"\"/> \n";
            outfile1<<svg_end;
            outfile1.close();
          }
          c++;
          //END///////////////////////////////////////////////////////////////////////*/



          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }

    /* Calculates the distance from the current position to the bezier curve, the angle of the vehicle with respect to the curve,
    and the curvature of the curve. Those values are needed for the controller. */
    ControllerInput calculate_curve_data(const State& state)  {
        Vec pos = state.control_position;

        stringstream pos_point;
        pos_point << "thick black dot " << pos.x << " " << pos.y;
        BBOARD->addPlotCommand(pos_point.str());

        lastProjectionParameter = bc.project(pos, lastProjectionParameter,
                                             newton_tolerance, newton_max_iter);
        //LOUT("Projected Parameter: " << lastProjectionParameter << endl);

        //evaluate bezier curve and derivatives at the projection parameter
        Vec f = bc(lastProjectionParameter);

        stringstream project_point;
        project_point << "thick green dot " << f.x << " " << f.y;
        BBOARD->addPlotCommand(project_point.str());

        Vec df = bc.prime(lastProjectionParameter);
        //Vec ddf = bc.double_prime(lastProjectionParameter);

        //Calculate distance form current position to curve
        //The difference vector is normal to the tangent in the projection point
        //We use this to assign a sign to the distance:
        //The distance is positive, if the actual position is left of the curve
        // and negative if it's right of the curve (regarding moving direction)
        Vec diff = pos - f;
        double distance = diff.length();
        //If the point is right of df, let the distance have a negative sign
        if (diff * df.rotate_quarter() < 0) {
            distance *= -1;
        }

        //Calculate difference angle between vehicle and curve
//            double phi;
//            if (df.x != 0 && abs(df.y/df.x) < 1) {
//                //atan2 takes the direction into account
//                phi = atan2(df.y, df.x);
//            } else {
//                //avoid singularities by rotating coordinate system for 2nd and 4th quadrant
//                phi = M_PI/2 + atan2(-df.x, df.y);
//            }
        Angle diff_angle = state.orientation - bc.orientation(df);

        //Calculate the curvature of the bezier curve
//            double curvature_numerator = ddf.y * df.x - ddf.x * df.y;
//            double curvature_denominator = pow(df.squared_length(), 3.0/2);
//            double curvature = curvature_numerator / curvature_denominator;
        double curvature = bc.curvature(lastProjectionParameter, df);

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
