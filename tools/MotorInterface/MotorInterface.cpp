
#include <iostream>
#include <fstream>
#include <signal.h>
#include <cstdlib>
#include <stdexcept>
#include "../../Elementary/Timestamp.h"
#include "MotorInterface.h"

using namespace std;
using namespace DerWeg;

const bool USEEPOS=true;
const bool USEIIC=true;

#define dout cerr
ofstream logout ("motorcontrol.log");

MotorInterface* ifc = NULL;
bool needExit = false;

void myexit (int ret=0) {
  if (ifc) ifc->emergencyStop();
  MotorInterface* ptr = ifc;
  ifc=NULL;
  if (ptr) delete ptr;
  exit(ret);
}

// Signal handler
void sigfuncexit(int sig) {
  dout << "MotorInterface: MotorInterface wird beendet aufgrund von Signal " << sig << std::endl;
  needExit=true;
}


MotorInterface::MotorInterface(unsigned int port, const char* device, unsigned int mI) : epos(NULL), iic(NULL), deadlineInterval (mI) {
  if (USEEPOS) epos=new EPOS (device);
  if (USEIIC) iic=new IOWarriorIIC;
  if (!server.init_as_server(port))
    throw std::invalid_argument ("MotorInterface::MotorInterface: cannot start UDP communication");
};

MotorInterface::~MotorInterface() {
  server.close();
  if (iic) setSteerOff();
  if (epos) epos->emergencyStop();
  if (iic) delete iic;
  if (epos) delete epos;
};

void MotorInterface::setSteer (unsigned char b) {
  if (iic) {
    iic->send3Bytes (0xC2, 0x01, b>0 ? b : 1);  // 0xC2=address of SD20, 0x00=output address 0, b=value
  } else {
    dout << "no iic\n";
  }
}

void MotorInterface::setSteerOff () {
  if (iic) {
    iic->send3Bytes (0xC2, 0x01, 0);  // 0xC2=address of SD20, 0x00=output address 0, b=value
  } else {
    dout << "no iic\n";
  }
}

void MotorInterface::emergencyStop () {
  if (epos) epos->emergencyStop();
  if (iic) setSteerOff();
}

void MotorInterface::run () {
  while (!server.receive() && !needExit)  // warten, bis motor interface angesprochen wurde
    { usleep (10000); }
  server.admitInitialization();
  server.send();

  MotorCommand mc;   ///< the motor command received
  MotorFeedback mf;   ///< the motor feedback sent
  Timestamp deadlineReceive;  ///< the timestamp until something should be received
  Timestamp deadlineSend;  ///< the timestamp until something should be sent
  deadlineReceive.add_msec (3*deadlineInterval+1000);  // am Anfang laenger warten, da DerWeg sich noch initialisiert
  Timestamp timestampStatus;
  
  while (!needExit) {
    if (needExit) {
      myexit(-1);
    }

    Timestamp now;
    if (server.receive()) {
      deadlineReceive.update();
      deadlineReceive.add_msec(deadlineInterval);
    } else {
      server.clear_send_buffer();
    }
    if (server.goodbye()) {
      dout << "MotorInterface: DerWeg hat sich verabschiedet" << endl;
      break;
    }
    if (deadlineReceive<now) {
      dout << "MotorInterface: Zu lange Zeit keine Pakete empfangen" << endl;
      myexit(-3);
    }

    bool receiveMC=server.getMotorCommand(mc);
    bool receivePing=server.getPing();

    bool needSend=false;
    if (receivePing) {  // send ping after receiving ping
      server.putPing();
      needSend=true;
    }

    if (receiveMC) {  // apply motor command
      if (epos) {
        Timestamp t1;
        bool success = epos->setVelocity (mc.rpm);
        Timestamp t2;
        logout << t1.get_msec() << '\t' << t2.diff_msec(t1) << '\t' << "SetVelocity" << '\t' << success;
        if (!success) {
          EPOS::State eps = epos->getState();
          logout << '\t' << eps;
        }
        logout << endl;
      }
      steer=mc.steer;
      setSteer (steer);
    }

    if (receiveMC || deadlineSend<now) {  // send odometry
      long int rpm=0;
      if (true /*(USEEPOS && epos->getVelocity (rpm)) || !USEEPOS*/) {
        Timestamp t1;
        bool success = epos->getVelocity (rpm);
        Timestamp t2;
        logout << t1.get_msec() << '\t' << t2.diff_msec(t1) << '\t' << "GetVelocity " << rpm << '\t' << success;
        if (!success) {
          EPOS::State eps = epos->getState();
          logout << '\t' << eps;
        }
        logout << endl;
        mf.rpm=rpm;
        mf.steer=steer;
        server.putMotorFeedback(mf);
        needSend=true;
        deadlineSend=now;
        deadlineSend.add_msec(500);
      }
    }

    if (needSend)
      server.send();

    if (timestampStatus.elapsed_msec()>1000) {
      timestampStatus.update();
      EPOS::State eps = epos->getState();
      logout << timestampStatus.get_msec() << '\t' << "STATE&ERROR" << '\t' << eps;
      unsigned char err=0;
      if (epos->getErrorRegister(err)) {
        logout << '\t';
        unsigned char mask=128;
        for (unsigned int i=0; i<8; ++i) {
          logout << ((err&mask)!=0);
          mask>>=1;
        }
      }
      logout << '\n';
    }
    // usleep (10000);  // evtl noch ein usleep einbauen
  }
}



int main (int argc, char** argv) {
  try{
    signal(SIGINT,sigfuncexit);
    signal(SIGTERM,sigfuncexit);
    dout << "initializing motor controller ...";
    std::string device = "/dev/ttyUSB0";
    unsigned int port = 34382;
    if (argc>=2) device = argv[1];
    if (argc>=3) port = atoi(argv[2]);
    try{
      ifc = new MotorInterface (port, device.c_str());
    }catch(std::exception&) {
      dout << "... second try ...";
      ifc = new MotorInterface (port, device.c_str());
    }
    dout << "... done" << endl;
    ifc->run ();

    dout << "MotorInterface: MotorInterface beendet sich durch Selbstaufgabe" << endl;
    myexit (0);
  }catch(std::exception& e) {
    dout << "... unsuccessful\n" << e.what() << endl;
    dout << "MotorInterface: MotorInterface beendet sich nach Exception" << endl;
    myexit (-2);
  }
}
