namespace DerWeg {

  class Blackboard {
  public:
    ...

    /** get, set, and wait for a change of the 'activated' attribute */
    void setActive (bool);
    bool getActive ();
    bool waitForActive (boost::posix_time::time_duration timeout);
    /** get, set, and wait for the next camera image. It returns an empty
     *  image if currently no image is available */
    ImageBuffer getImage ();
    void setImage (const ImageBuffer& image);
    bool waitForImage (boost::posix_time::time_duration timeout);
    /** get, set, and wait for a change of the desired velocity */
    Velocity getDesiredVelocity ();
    void setDesiredVelocity (const Velocity&);
    bool waitForDesiredVelocity (boost::posix_time::time_duration timeout);
    /** get, set, and wait for a change of the vehicle pose */
    Pose getVehiclePose ();
    void setVehiclePose (const Pose&);
    bool waitForVehiclePose (boost::posix_time::time_duration timeout);
    /** get, set, and wait for a change of the odometry */
    Odometry getOdometry ();
    void setOdometry (const Odometry&);
    bool waitForOdometry (boost::posix_time::time_duration timeout);
    /** get (and erase) all messages, add a new message, and wait 
        for a new message */
    std::string getMessage ();
    void addMessage (const std::string&);
    bool waitForMessage (boost::posix_time::time_duration timeout);
    /** get (and erase) all plot commands, add a new plot command, and 
        wait for a new plot command */
    std::string getPlotCommand ();
    void addPlotCommand (const std::string&);
    bool waitForPlotCommand (boost::posix_time::time_duration timeout);
  };

  /** returns a pointer to the blackboard */
  #define BBOARD DerWeg::Blackboard::getInstance()

}