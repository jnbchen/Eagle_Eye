
#include <iostream>
#include <fstream>
#include <signal.h>
#include <cstdlib>
#include <stdexcept>
#include "../../Elementary/Timestamp.h"
#include "MotorInterfaceOldCar.h"

using namespace std;
using namespace DerWeg;

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


MotorInterface::MotorInterface(unsigned int port, const char* device, unsigned int mI) : iic(NULL), deadlineInterval (mI) {

  // For old motor
  desiredspeed = 128;
  curspeed = 128;
  
  if (USEIIC) iic=new IOWarriorIIC;
  if (!server.init_as_server(port))
    throw std::invalid_argument ("MotorInterface::MotorInterface: cannot start UDP communication");
};

MotorInterface::~MotorInterface() {
  server.close();
  if (iic) {
    setSteerOff();
    setMotorOff(); // For old motor
  } 
    
  if (iic) delete iic;
};

void MotorInterface::setSteer (unsigned char b) {
  if (iic) {
    iic->send3Bytes (0xC2, 0x01, b>0 ? b : 1);  // 0xC2=address of SD20, 0x00=output address 0, b=value
  } else {
    dout << "no iic\n";
  }
//  cerr << "STEER: " << static_cast<unsigned int>(b) << endl;
}

void MotorInterface::setSteerOff () {
  if (iic) {
    iic->send3Bytes (0xC2, 0x01, 0);  // 0xC2=address of SD20, 0x00=output address 0, b=value
  } else {
    dout << "no iic\n";
  }
}

/* For old motor  */

void MotorInterface::setMotorSpeed (unsigned char s) {
  if (iic) {
    iic->send3Bytes (0xC2, 0x02, s>0 ? s : 128);  // 0xC2=address of SD20, 0x02=output address 0, s=value
  } else {
    dout << "no iic\n";
  }
//  cerr << "PWM: " << static_cast<unsigned int>(s) << endl;
}

/* For old motor */

void MotorInterface::setMotorOff () {
  if (iic) {
    iic->send3Bytes (0xC2, 0x02, 128);  // 0xC2=address of SD20, 0x02=output address 0, b=128 Stop
  } else {
    dout << "no iic\n";
  }
}

void MotorInterface::emergencyStop () {
  if (iic) {
    setSteerOff();
    setMotorOff(); // For old motor
  }
}

void MotorInterface::run () {
  setMotorOff(); // For old motor
  while (!server.receive() && !needExit)  // warten, bis motor interface angesprochen wurde
    { usleep (10000); }
  server.admitInitialization();
  server.send();

  MotorCommand_oldcar mc_oldcar; // For old motor
  MotorFeedback_oldcar mf_oldcar; // For old motor
  
  Timestamp deadlineReceive;  ///< the timestamp until something should be received
  Timestamp deadlineSend;  ///< the timestamp until something should be sent
  deadlineReceive.add_msec (3*deadlineInterval+1000);  // am Anfang laenger warten, da DerWeg sich noch initialisiert

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

    bool receiveMC=server.getMotorCommand_oldcar(mc_oldcar);
    bool receivePing=server.getPing();

    bool needSend=false;
    if (receivePing) {  // send ping after receiving ping
      server.putPing();
      needSend=true;
    }

    if (receiveMC) {  // apply motor command
      // For old motor
      desiredspeed = mc_oldcar.pwmspeed;
      if ( desiredspeed > curspeed ) {
	while ( curspeed < desiredspeed ) {
	  ++curspeed;
	  setMotorSpeed (curspeed);
	}	
      }     
      else {
	while ( curspeed > desiredspeed ) {
	 --curspeed;
	  setMotorSpeed (curspeed); 
	}	  
      }    
      // For old motor
      steer=mc_oldcar.steer;
      setSteer (steer);
      
    }

    if (receiveMC || deadlineSend<now) { 	
	// For old motor
	mf_oldcar.pwmspeed= curspeed;
        mf_oldcar.steer=steer;
	
	server.putMotorFeedback_oldcar(mf_oldcar);
        needSend=true;
        deadlineSend=now;
        deadlineSend.add_msec(500);
      
    }

    if (needSend)
      server.send();

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
    ifc = new MotorInterface (port, device.c_str());
    dout << "... done" << endl;
    ifc->run ();

    dout << "MotorInterface: MotorInterface beendet sich durch Selbstaufgabe" << endl;
    myexit (0);
  }catch(std::exception& e) {
    dout << e.what() << endl;
    dout << "MotorInterface: MotorInterface beendet sich nach Exception" << endl;
    myexit (-2);
  }
}
