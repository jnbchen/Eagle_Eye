
#ifndef KOGMOLABORCOMMUNICATION_H_
#define KOGMOLABORCOMMUNICATION_H_

#include <vector>

#include "NonspecificTaggedUDPCommunication.h"
#include "../../Vehicle/vehicle.h"
#include "../ConfigReader.h"

namespace DerWeg {

  class CommMessage {
    std::string msg;
    unsigned int index;
  public:
    CommMessage ();
    void putFloat (float);
    void putString (std::string);
    void putStringList (const std::vector<std::string>&);
    void putBool (bool);
    void putUChar (unsigned char);
    void putLInt (long int);
    void putTimestamp (Timestamp);
    void putAngle (Angle);
    void putVec (Vec);

    float getFloat ();
    std::string getString ();
    std::vector<std::string> getStringList ();
    bool getBool ();
    unsigned char getUChar ();
    long int getLInt ();
    Timestamp getTimestamp ();
    Angle getAngle ();
    Vec getVec ();

    const char* getMsg () const;
    unsigned int getMsgLen () const;
    void setMsg (const char*, unsigned int);
  };

  class KogmolaborCommunication : public NonspecificTaggedUDPCommunication {
  protected:
    bool putCommMessage (unsigned char, const CommMessage&);
    bool getCommMessage (unsigned char, CommMessage&);

  public:
    KogmolaborCommunication();
    ~KogmolaborCommunication() throw ();

    bool sayGoodbye();
    bool goodbye();
    bool putPing();
    bool getPing();
    bool admitInitialization();
    bool getAdmitInitialization();

    bool putMotorCommand (const MotorCommand&);
    bool getMotorCommand (MotorCommand&);
    bool putMotorCommand_oldcar (const MotorCommand_oldcar&);
    bool getMotorCommand_oldcar (MotorCommand_oldcar&);

    bool putMotorFeedback (const MotorFeedback&);
    bool getMotorFeedback (MotorFeedback&);
    bool putMotorFeedback_oldcar (const MotorFeedback_oldcar&);
    bool getMotorFeedback_oldcar (MotorFeedback_oldcar&);

    bool putOdometry(const Odometry&);
    bool getOdometry(Odometry&);
    bool putVelocity (const Velocity&);
    bool getVelocity (Velocity&);
    bool putPose (const Pose&);
    bool getPose (Pose&);
    bool putAddModule (const std::string&);
    bool getAddModule (std::string&);
    bool putRemoveModule (const std::string&);
    bool getRemoveModule (std::string&);

    bool putActive(const bool);
    bool getActive(bool&);

    bool putSaveImage();
    bool getSaveImage();

    bool putAllModules(const std::vector<std::string>&);
    bool getAllModules(std::vector<std::string>&);
    bool askForAllModules();
    bool askedForAllModules();

    bool putPresentModules(const std::vector<std::string>&);
    bool getPresentModules(std::vector<std::string>&);
    bool askForPresentModules();
    bool askedForPresentModules();

    bool putMessages(const std::string&);
    bool getMessages(std::string&);
    bool putPlotCommand(const std::string&);
    bool getPlotCommand(std::string&);
  };

} // DerWeg


#endif // KOGMOLABORCOMMUNICATION_H_
