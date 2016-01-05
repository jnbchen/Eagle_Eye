
#ifndef __KoginitivesLabor_MotorInterface_h__
#define __KoginitivesLabor_MotorInterface_h__

#include "../../Elementary/UDP/KogmolaborCommunication.h"
#include "../MotorInterface/IOWarriorIIC.h"

namespace DerWeg {

  /** Interface das einen UDP-Socket als Server öffnet und
  *   verbindet sich mit dem Motor
  */
  class MotorInterface {
    KogmolaborCommunication server;
    IOWarriorIIC* iic;
    unsigned char steer;
    unsigned char curspeed;  // For old motor
    unsigned char desiredspeed; // For old motor
    unsigned int deadlineInterval;  ///< the maximal number of msec between two points in time when a command was received

  public:
    MotorInterface(unsigned int port =34382, const char* device = "/dev/ttyUSB0", unsigned int mI = 1000);
    ~MotorInterface();
    void emergencyStop ();
    void setSteer (unsigned char);
    void setMotorSpeed (unsigned char); // For old motor
    void setSteerOff ();
    void setMotorOff (); // For old motor

    void run ();
  };

}

#endif
