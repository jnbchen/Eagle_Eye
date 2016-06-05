
#ifndef _DerWeg_BLACKBOARD_H__
#define _DerWeg_BLACKBOARD_H__

#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include "../Vehicle/vehicle.h"
#include "../Elementary/ImageBuffer.h"
#include "../Elementary/Configuration.h"
#include "../Elementary/ThreadSafeLogging.h"
#include "../EagleEye/DataObjects.h"    // here all eagle eye specific data structures


namespace DerWeg {

/**
* Blackboardklasse
*
* Singleton zum Bereitstellen verschiedener Attribute
* \author Birgitt Sauter
*/

class Blackboard {
  private:
    static Blackboard* instance;               ///< pointer onto the only instance of class Blackboard (singleton)
    Blackboard();                              ///< private constructor (since singleton)
    ~Blackboard();                          ///< private destructor to avoid someone deleting Blackboard

    boost::mutex bbmutex;                          ///< mutex to lock when accessing the blackboard
    boost::condition_variable condActive;          ///< condition variable to signal change of active
    boost::condition_variable condImage;           ///< condition variable to signal change of image
    boost::condition_variable condTopImage;        ///< condition variable to signal change of top image
    boost::condition_variable condDesiredVelocity; ///< condition variable to signal change of desired velocity
    boost::condition_variable condPose;            ///< condition variable to signal change of pose
    boost::condition_variable condOdometry;        ///< condition variable to signal change of odometry
    boost::condition_variable condMessage;         ///< condition variable to signal change of message
    boost::condition_variable condPlotcmd;         ///< condition variable to signal change of plot commands

    boost::condition_variable condState;           ///< condition variable to signal change of state
    boost::condition_variable condReferenceTrajectory;  ///< condition variable to signal change of reference trajectory
    boost::condition_variable condDrivingMode;     ///< condition variable to signal change of driving mode

    bool exitProgram;                          ///< attribute indicating when the program should be stopped (stopProgram=true)

    bool active;                               ///< true, if vehicle is allowed to move
    ImageBuffer img;                           ///< present camera image, owned by the ImageSource, not by Blackboard
    ImageBuffer top_img;                       ///< present camera image of topview camera (i.e. for localization), owned by the ImageSource, not by Blackboard
    Velocity desiredVelocity;                  ///< the desired velocity of the car
    Pose vehiclePose;                          ///< the present pose and velocity of the car (from self localization)
    Odometry odometry;                         ///< the present odometry (velocity and steering angle)

    State state;                               ///< the present estimated state from StateEstimator
    ReferenceTrajectory reference_trajectory;  ///< the present bezier curve for Control
    //DrivingMode driving_mode;                  ///< the driving mode from the StateMachine for TrajectoryGeneration

    std::stringstream message;                 ///< messages from the applications, to be sent to GUI
    std::stringstream plotcmd;                 ///< plot commands from the applications, to be sent to GUI

  public:
    static Blackboard* getInstance();          ///< method to get the only onstance of Blackboard (singleton)

    // Methods to set/get status information:
    void setExitProgram();                     ///< set attribute exitProgram, i.e. indicates that the program should be exited
    bool getExitProgram();                     ///< read exit attribute

    void setActive (const bool);               ///< activate (true) or deactivate (false) vehicle
    bool getActive ();                         ///< check whether vehicle is activated
    bool waitForActive (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                     ///< waits until activity has been changed and return true (or until timeout and return false)

    ImageBuffer getImage ();                   ///< return present camera image. Return empty image if no image is available
    void setImage (const ImageBuffer& image);  ///< provide the present camera image. The image remains in the ownership of the provider
    bool waitForImage (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                      ///< waits until a new image has been read from image source and return true (or until timeout and return false)

    ImageBuffer getTopImage ();                   ///< return present camera image of topview camera. Return empty image if no image is available
    void setTopImage (const ImageBuffer& image);  ///< provide the present camera image of topview camera. The image remains in the ownership of the provider
    bool waitForTopImage (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                   ///< waits until a new topview image has been read from image source and return true (or until timeout and return false)

    Velocity getDesiredVelocity ();            ///< get the desired velocity
    void setDesiredVelocity (const Velocity&); ///< set the desired velocity
    bool waitForDesiredVelocity (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));            ///< waits until a new desired velocity has been set and return true (or until timeout and return false)

    Pose getVehiclePose ();                    ///< get the vehicle pose from self localization
    void setVehiclePose (const Pose&);         ///< set the vehicle pose from self localization
    bool waitForVehiclePose (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                ///< waits until a new vehicle pose has been read from localization and return true (or until timeout and return false)

    Odometry getOdometry ();                   ///< get odometry of the vehicle
    void setOdometry (const Odometry&);        ///< set odometry of the vehicle
    bool waitForOdometry (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                   ///< waits until a new odometry measurement has been read and return true (or until timeout and return false)

    std::string getMessage ();                 ///< get messages (and remove message buffer)
    void addMessage (const std::string&);      ///< add message to message buffer
    bool waitForMessage (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                    ///< waits until a new message has been read and return true (or until timeout and return false)

    std::string getPlotCommand ();             ///< get plot command (and remove plot command buffer)
    void addPlotCommand (const std::string&);  ///< add plot command to plot command buffer
    bool waitForPlotCommand (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                ///< waits until a new plot command has been read and return true (or until timeout and return false)

    ///////////////////////////////////////
    // Eagle Eye Data

    // State
    State getState ();                    ///< get the state from StateEstimator
    void setState (const State&);         ///< set the state from StateEstimator
    bool waitForState (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                ///< waits until a new state has been read from StateEstimator and return true (or until timeout and return false)

    // ReferenceTrajectory
    ReferenceTrajectory getReferenceTrajectory ();                    ///< get the reference_trajectory from TrajectoryGenerator
    void setReferenceTrajectory (const ReferenceTrajectory&);         ///< set the reference_trajectory from TrajectoryGenerator
    bool waitForReferenceTrajectory (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                ///< waits until a new reference_curve has been read from TrajectoryGenerator and return true (or until timeout and return false)
/*
    // DrivingMode
    DrivingMode getDrivingMode ();                    ///< get the driving_mode from StateMachine
    void setDrivingMode (const DrivingMode&);         ///< set the driving_mode from StateMachine
    bool waitForDrivingMode (boost::posix_time::time_duration timeout = boost::posix_time::microseconds(1000000));                ///< waits until a new driving_mode has been read from StateMachine and return true (or until timeout and return false)
*/
  };

#define BBOARD DerWeg::Blackboard::getInstance()    // macro to simplify accessing the blackboard

}

#endif // BLACKBOARD_H__
