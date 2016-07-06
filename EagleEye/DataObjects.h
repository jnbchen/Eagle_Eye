
#ifndef _DerWeg_DATAOBJECTS_H__
#define _DerWeg_DATAOBJECTS_H__

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include "../Elementary/Timestamp.h"
#include "../Elementary/ImageBuffer.h"

#include "BezierCurve.h"


namespace DerWeg {

    // none, if no traffic light was detected
    // "red" represents the red signal, yellow signal and red-yellow signal
    enum TrafficLightState {none, red, green};


  /** struct to describe present dynamic state of Anicar from localization with StateEstimator*/
  struct State {
    Vec sg_position;       ///< the position (x,y) in mm of the stargazer
    Vec control_position; ///< the position (x,y) in mm at which the car is controlled (-> which has to stay on the curve)
    double stddev;      ///< an approximation to the standard deviation of position in mm
    Angle orientation;  ///< the orientation
    double velocity;    ///< the longitudinal velocity (in m/s)
    double yawrate;     ///< the yaw rate (in rad/s)
    Timestamp timestamp; ///< point in time to which information refers

    float velocity_tire;  ///< the velocity in m/s at the tires
    Angle steer;     ///< the steering angle in degree

    bool operator== (const State&) const throw ();
    bool operator!= (const State&) const throw ();
  };


  /** Struct to describe the reference path possibly with velocity information */
  struct ReferenceTrajectory {
    BezierCurve path;  ///< reference path, a bezier curve
    int segment_id; ///< ID of the segment the current BezierCurve belongs to
    double v_max; ///< maximal velocity the car shall drive (e.g. calculated by the traffic light behaviour)

    bool operator== (const ReferenceTrajectory&) const throw ();
    bool operator!= (const ReferenceTrajectory&) const throw ();
  };


  /** Struct to describe the position and state of traffic lights */
  struct TrafficLightData {
    TrafficLightState state;
    Vec position;
  };


  /** Struct to bundle relevant data for input to image processing modules */
  struct RectImages {
      ImageBuffer images;
      State state;
      ReferenceTrajectory reference_trajectory;
  };




} // namespace DerWeg

#endif // DATAOBJECTS_H__

