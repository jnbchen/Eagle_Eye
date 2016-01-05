
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Elementary/UDP/KogmolaborCommunication.h"
#include "../Blackboard/Blackboard.h"
#include <sstream>
#include <cmath>


namespace DerWeg {

  class MotorControlByInterface : public KogmoThread {
    KogmolaborCommunication client;
    unsigned int port;
    std::string device;
    double gearRatio;   ///< gear ratio: 1 revolution covers 'gearRatio' mm
    double steerRatio;   ///< steer ratio: 1 tick covers 'steerRatio' degree
    double steerOffset;   ///< 'steerOffset' ticks mean zero degree
    double steerDirectionOffset;  ///< parameter to cover steering angle depending on previous steering angle
    Angle previousDesiredAngle;
    double previousTicks;
  public:

    MotorControlByInterface() : port(34382), device ("/dev/ttyUSB0"), previousTicks(127) {
      client.init_as_client("localhost", port);
    };

    ~MotorControlByInterface() {
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
        throw std::invalid_argument ("fork gescheitert in MotorControlByInterface::startInterface");
      } else if (pid==0) {
        std::stringstream ss;
        ss << port;
        execlp("./MotorInterface","MotorInterface",device.c_str(),ss.str().c_str(),NULL);  // interface im Kindprozess starten
        execlp("MotorInterface","MotorInterface",device.c_str(),ss.str().c_str(),NULL);  // interface im Kindprozess starten
        throw std::invalid_argument ("execlp gescheitert in MotorControlByInterface::startInterface");
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
          throw std::invalid_argument ("Initialisierung des Interface gescheitert in MotorControlByInterface::startInterface");

        //boost::this_thread::sleep(boost::posix_time::milliseconds(10));
        return true;//testConnection();
      }
    }

    void init(const ConfigReader& cfg) {
      cfg.get ("MotorInterface::device", device);
      if (!client.started())
        throw std::invalid_argument ("MotorControlByInterface::init: Socket kann nicht geoeffnet werden");
      if (!startInterface())  // lazy evaluation sei Dank
        throw std::invalid_argument ("MotorControlByInterface::init: kann interface nicht erzeugen bzw. keine Kommunikation aufbauen");
      if (!cfg.get ("MotorInterface::gear_ratio", gearRatio))
        throw std::invalid_argument ("MotorControlByInterface::init: kann CFG-Parameter gear_ratio nicht finden");
      if (!cfg.get ("MotorInterface::steer_ratio", steerRatio))
        throw std::invalid_argument ("MotorControlByInterface::init: kann CFG-Parameter steer_ratio nicht finden");
      if (!cfg.get ("MotorInterface::steer_offset", steerOffset))
        throw std::invalid_argument ("MotorControlByInterface::init: kann CFG-Parameter steer_offset nicht finden");
      if (!cfg.get ("MotorInterface::steer_direction_offset", steerDirectionOffset))
        throw std::invalid_argument ("MotorControlByInterface::init: kann CFG-Parameter steer_direction_offset nicht finden");
    }
    void execute () {
      try {
        Timestamp timestamp_latest_communication;
        while (!client.goodbye()) {
          if (client.receive()) {
            timestamp_latest_communication.update();
            MotorFeedback mf;
            if (client.getMotorFeedback(mf)) {
              // Umsetzung Motordrehzahl/Sollposition -> Istgeschwindigkeiten
              double deltaticks = (mf.steer-previousTicks)*(steerRatio>0 ? +1 : -1);
              double dir = 0;
              if (deltaticks>=1) {
                dir=-1;
                previousTicks=mf.steer;
              } else if (deltaticks<=-1) {
                dir=+1;
                previousTicks=mf.steer;
              }
              Odometry odo;
              double s=255-static_cast<double>(mf.steer);
              odo.steer = Angle::deg_angle((s-steerOffset-dir*steerDirectionOffset)*steerRatio);
              odo.velocity = gearRatio*mf.rpm;
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
          double dir=0;
          double deltaa = (dv.steer-previousDesiredAngle).get_deg_180();
          if (deltaa>0.5) {
            previousDesiredAngle=dv.steer;
            dir=-1;
          } else if (deltaa<-0.5) {
            previousDesiredAngle=dv.steer;
            dir=+1;
          }
          MotorCommand mc;
          // Umsetzung Sollgeschwindigkeiten -> Motordrehzahl/Sollposition
          double s = (dv.steer.get_deg_180()/steerRatio)+steerOffset+dir*steerDirectionOffset;
          mc.steer = 255-static_cast<unsigned char>(s<0 ? 0 : (s>255 ? 255 : std::floor(s+0.5)));
          mc.rpm = dv.velocity/gearRatio;
          if (!BBOARD->getActive()) {
            // Roboter ist nicht aktiviert
            mc.steer = 0;
            mc.rpm = 0;
          }
          client.putMotorCommand (mc);
          client.send();
          boost::this_thread::sleep(boost::posix_time::milliseconds(25));
          boost::this_thread::interruption_point();
          //EOUT(".");  // nur zur Kontrolle, kann spaeter entfernt werden
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
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::MotorControlByInterface> motorcontrol_by_interface ("MotorInterface");

} // namespace
