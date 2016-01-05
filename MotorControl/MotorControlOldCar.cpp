
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Elementary/UDP/KogmolaborCommunication.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>
#include <cmath>


namespace DerWeg {

  class MotorControlOldCar : public KogmoThread {
    KogmolaborCommunication client;
    unsigned int port;
    std::string device;
    double gearRatio;   ///< gear ratio: 1 revolution covers 'gearRatio' mm
    double steerRatio;   ///< steer ratio: 1 tick covers 'steerRatio' degree
    double steerOffset;   ///< 'steerOffset' ticks mean zero degree
  public:

    MotorControlOldCar() : port(34382), device ("/dev/ttyUSB0") {
      client.init_as_client("localhost", port);
    };

    ~MotorControlOldCar() {
      client.close();
    };

    bool testConnection () {
      for (unsigned int i=0; i<3; ++i) {
        client.putPing();
        client.send();
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        client.receive();
        if (client.getPing())
          return true;
      }
      return false;
    }
    bool startInterface () {
      int pid = fork ();
      if (pid<0) {
        throw std::invalid_argument ("fork gescheitert in MotorControlOldCar::startInterface");
      } else if (pid==0) {
        std::stringstream ss;
        ss << port;
        execlp("./MotorInterfaceOldCar","MotorInterfaceOldCar",device.c_str(),ss.str().c_str(),NULL);  // interface im Kindprozess starten
        execlp("MotorInterfaceOldCar","MotorInterfaceOldCar",device.c_str(),ss.str().c_str(),NULL);  // interface im Kindprozess starten
        throw std::invalid_argument ("execlp gescheitert in MotorControlOldCar::startInterface");
      } else {
        int t=0;
        while (t<10000) {
          client.receive();
          if (client.getAdmitInitialization()) {
            //client.admitInitialization();
            break;
          } else {
            client.putPing();
            client.send();
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            t += 10;
          }
        }
        if (t>=10000)
          throw std::invalid_argument ("Initialisierung des Interface gescheitert in MotorControlOldCar::startInterface");

        //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        return true;//testConnection();
      }
    }

    void init(const ConfigReader& cfg) {
      cfg.get ("MotorInterface::device", device);
      if (!client.started())
        throw std::invalid_argument ("MotorControlOldCar::init: Socket kann nicht geoeffnet werden");
      if (!startInterface())  // lazy evaluation sei Dank
        throw std::invalid_argument ("MotorControlOldCar::init: kann interface nicht erzeugen bzw. keine Kommunikation aufbauen");
      if (!cfg.get ("MotorInterfaceOldCar::gear_ratio", gearRatio))
        throw std::invalid_argument ("MotorControlOldCar::init: kann CFG-Parameter gear_ratio nicht finden");
      if (!cfg.get ("MotorInterfaceOldCar::steer_ratio", steerRatio))
        throw std::invalid_argument ("MotorControlOldCar::init: kann CFG-Parameter steer_ratio nicht finden");
      if (!cfg.get ("MotorInterfaceOldCar::steer_offset", steerOffset))
        throw std::invalid_argument ("MotorControlOldCar::init: kann CFG-Parameter steer_offset nicht finden");
    }
    void execute () {
      try {
        Timestamp timestamp_latest_communication;
        while (!client.goodbye()) {
          if (client.receive()) {
            timestamp_latest_communication.update();
            MotorFeedback_oldcar mf;
            if (client.getMotorFeedback_oldcar(mf)) {
              // Umsetzung Motordrehzahl/Sollposition -> Istgeschwindigkeiten
              Odometry odo;
              odo.steer = Angle::deg_angle(((static_cast<double>(mf.steer))-steerOffset)*steerRatio);
              odo.velocity = gearRatio*(static_cast<double>(mf.pwmspeed)-128);
              BBOARD->setOdometry(odo);
            }
          } else if (timestamp_latest_communication.elapsed_msec()>2000) {
            EOUT("restart interface");
            if (!startInterface()) {
              // Irgendwas sinnvolles tun, wenn auch nach Neustart von interface() nichts geht
              // aber was? Fehlermeldung, Exception, Programmstop, erneutes Probieren?
              EOUT("[restart failed]");
            }
            timestamp_latest_communication.update();
          }
          Velocity dv = BBOARD->getDesiredVelocity();
	  MotorCommand_oldcar mc_oldcar; // For old motor
	  
          // Umsetzung Sollgeschwindigkeiten -> Motordrehzahl/Sollposition
          double s = (dv.steer.get_deg_180()/steerRatio)+steerOffset;
          	  
	  // For old motor
	  mc_oldcar.steer = static_cast<unsigned char>(s<0 ? 0 : (s>255 ? 255 : std::floor(s+0.5)));
	  if (mc_oldcar.steer<30) mc_oldcar.steer=30;
	  if (mc_oldcar.steer>190) mc_oldcar.steer=190;	  
	  int impwm = 128+static_cast<int>(dv.velocity/gearRatio);
          mc_oldcar.pwmspeed = (impwm<40 ? 40 : (impwm>205 ? 205 : impwm));
	  
          if (!BBOARD->getActive()) {
            // Roboter ist nicht aktiviert
 	    
	    // For old motor
	    mc_oldcar.pwmspeed = 128;
	    mc_oldcar.steer = steerOffset;
          }
	  // For old motor
	  client.putMotorCommand_oldcar (mc_oldcar);
          client.send();
          boost::this_thread::sleep(boost::posix_time::milliseconds(25));
	  
          boost::this_thread::interruption_point();
          EOUT(".");  // nur zur Kontrolle, kann spaeter entfernt werden
        }
      }
      catch(boost::thread_interrupted&){;}
    }
    void deinit () {
      client.sayGoodbye();
      client.send();
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::MotorControlOldCar> motorcontrol_by_interface ("MotorInterfaceOldCar");

} // namespace
