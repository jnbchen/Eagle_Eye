
#ifndef __KoginitivesLabor_MotorInterface_h__
#define __KoginitivesLabor_MotorInterface_h__

#include "../../Elementary/UDP/KogmolaborCommunication.h"
#include "EPOS.h"
#include "IOWarriorIIC.h"

namespace DerWeg {

  /** Interface das einen UDP-Socket als Server öffnet und
  *   verbindet sich mit dem Motor
  */
  class MotorInterface {
    KogmolaborCommunication server;
    EPOS* epos;
    IOWarriorIIC* iic;
    unsigned char steer;
    unsigned int deadlineInterval;  ///< the maximal number of msec between two points in time when a command was received

  public:
    MotorInterface(unsigned int port =34382, const char* device = "/dev/ttyUSB0", unsigned int mI = 1000);
    ~MotorInterface();
    void emergencyStop ();
    void setSteer (unsigned char);
    void setSteerOff ();

    void run ();
  };

}

#endif
