
#ifndef _DerWeg_VEHICLE_H__
#define _DerWeg_VEHICLE_H__

#include "../Elementary/Angle.h"
#include "../Elementary/Vec.h"
#include "../Elementary/Timestamp.h"

namespace DerWeg {

  /** struct MotorCommand represents all information that is send to the
      motor controller, i.e. motor revolutions per second and steering angle */
  struct MotorCommand {
    float rpm;     ///< motor revolutions per minute
    unsigned char steer;   ///< steering angle in ticks

    MotorCommand () throw ();  ///< default constructor initializes zero rpm and zero steering angle
    MotorCommand (const MotorCommand&) throw ();
    ~MotorCommand () throw ();
    const MotorCommand& operator= (const MotorCommand&) throw ();
  };

  /** For old car */
  struct MotorCommand_oldcar {
    unsigned char pwmspeed;     ///< motor revolutions per minute
    unsigned char steer;   ///< steering angle in ticks

    MotorCommand_oldcar () throw ();  ///< default constructor initializes zero rpm and zero steering angle
    MotorCommand_oldcar (const MotorCommand_oldcar&) throw ();
    ~MotorCommand_oldcar () throw ();
    const MotorCommand_oldcar& operator= (const MotorCommand_oldcar&) throw ();
  };

  /** struct MotorFeedback represents all information that is send from the
      motor controller to the main program, i.e. motor revolutions per second and steering angle */
  struct MotorFeedback {
    float rpm;     ///< motor revolutions per minute
    unsigned char steer;   ///< steering angle in ticks

    MotorFeedback () throw ();  ///< default constructor initializes zero rpm and zero steering angle
    MotorFeedback (const MotorFeedback&) throw ();
    ~MotorFeedback () throw ();
    const MotorFeedback& operator= (const MotorFeedback&) throw ();
  };

  /** For old car */
  struct MotorFeedback_oldcar {
    unsigned char pwmspeed;      ///< motor revolutions per minute
    unsigned char steer;   	///< steering angle in ticks

    MotorFeedback_oldcar () throw ();  ///< default constructor initializes zero rpm and zero steering angle
    MotorFeedback_oldcar (const MotorFeedback_oldcar&) throw ();
    ~MotorFeedback_oldcar () throw ();
    const MotorFeedback_oldcar& operator= (const MotorFeedback_oldcar&) throw ();
  };

  /** struct Velocity represents all information that describes the
      velocity of a car, i.e. longitudinal velocity and steering angle */
  struct Velocity {
    float velocity;  ///< the velocity in m/s
    Angle steer;     ///< the steering angle in degree

    Velocity () throw ();  ///< default constructor initializes zero rpm and zero steering angle
    Velocity (const Velocity&) throw ();
    ~Velocity () throw ();
    const Velocity& operator= (const Velocity&) throw ();
    bool operator== (const Velocity&) const throw ();
    bool operator!= (const Velocity&) const throw ();
  };

  /** struct odometry describes the odometry of the car */
  struct Odometry {
    float velocity;  ///< the velocity in m/s
    Angle steer;     ///< the steering angle in degree

    Odometry() throw ();
    Odometry(const Odometry&) throw ();
    ~Odometry() throw ();
    const Odometry& operator= (const Odometry&) throw ();
    bool operator== (const Odometry&) const throw ();
    bool operator!= (const Odometry&) const throw ();
  };

  /** struct to describe present pose and velocity of the car from self localization */
  struct Pose {
    Vec position;       ///< the position (x,y) in mm
    double stddev;      ///< an approximation to the standard deviation of position in mm
    Angle orientation;  ///< the orientation
    double velocity;    ///< the longitudinal velocity (in m/s)
    double yawrate;     ///< the yaw rate (in rad/s)
    Timestamp timestamp; ///< point in time to which information refers

    Pose () throw ();
    Pose (const Pose&) throw ();
    ~Pose () throw ();
    const Pose& operator= (const Pose&) throw ();
    bool operator== (const Pose&) const throw ();
    bool operator!= (const Pose&) const throw ();
  };

} // namespace DerWeg

#endif // VEHICLE_H__
