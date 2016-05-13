
#ifndef _DerWeg_DATAOBJECTS_H__
#define _DerWeg_DATAOBJECTS_H__

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include "../Elementary/Timestamp.h"


namespace DerWeg {

  /** struct to describe present dynamic state of Anicar from localization with StateEstimator*/
  struct State {
    Vec position;       ///< the position (x,y) in mm
    double stddev;      ///< an approximation to the standard deviation of position in mm
    Angle orientation;  ///< the orientation
    double velocity;    ///< the longitudinal velocity (in m/s)
    double yawrate;     ///< the yaw rate (in rad/s)
    Timestamp timestamp; ///< point in time to which information refers

    float velocity_tire;  ///< the velocity in m/s at the tires
    Angle steer;     ///< the steering angle in degree

    /*
    State () throw ();
    State (const State&) throw ();
    ~State () throw ();
    const State& operator= (const State&) throw ();
    */
    bool operator== (const State&) const throw ();
    bool operator!= (const State&) const throw ();
  };


  /** struct to describe the present bezier curve of the segment */
  struct ReferenceCurve {
    double x1, x2, x3, x4;  ///< x position of start(1), end(4) and control points (2, 3)
    double y1, y2, y3, y4;  ///< y position of start(1), end(4) and control points (2, 3)

    /*
    ReferenceCurve () throw ();
    ReferenceCurve (const ReferenceCurve&) throw ();
    ~ReferenceCurve () throw ();
    const ReferenceCurve& operator= (const ReferenceCurve&) throw ();
    */
    bool operator== (const ReferenceCurve&) const throw ();
    bool operator!= (const ReferenceCurve&) const throw ();
  };


  /** struct to describe the driving mode */
  struct DrivingMode {
    char current_mode;  ///< current dirivng mode (strait, right, left, out)
    char next_mode;  ///< next dirivng mode in time (strait, right, left, out)

    /*
    DrivingMode () throw ();
    DrivingMode (const DrivingMode&) throw ();
    ~DrivingMode () throw ();
    const DrivingMode& operator= (const DrivingMode&) throw ();
    */
    bool operator== (const DrivingMode&) const throw ();
    bool operator!= (const DrivingMode&) const throw ();
  };


} // namespace DerWeg

#endif // DATAOBJECTS_H__

