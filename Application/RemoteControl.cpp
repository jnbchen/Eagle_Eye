
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Elementary/Timestamp.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/UDP/KogmolaborCommunication.h"
#include <boost/interprocess/sync/named_mutex.hpp>

#include <iostream>
using std::cerr;

namespace DerWeg {

  /** eine Schnittstelle fuer eine entfernte Steuerung von AniCar;
      hoert auf ping, active, desiredVecolicy; liefert Zustand mit 
      active, pose, odometry, desiredVelocity */
  class RemoteControl : public KogmoThread {
  private:
    KogmolaborCommunication socket;
  public:
    RemoteControl () {
      if (!socket.init_as_server (34378))
        throw std::invalid_argument ("Cannnot create communication in module RemoteControl");
      boost::interprocess::named_mutex ipmutex (boost::interprocess::open_or_create, "IPMutex_DerWeg_RemoteControl");
    }
    ~RemoteControl () {
      boost::interprocess::named_mutex::remove("IPMutex_DerWeg_RemoteControl");
    }
    void execute () {
      try{
        Velocity latestVelocity;
        Odometry latestOdometry;
        Pose latestPose;
        Timestamp t;
        t.set_msec(-1000);
        bool alreadyComplained=false;
        bool first=true;
        while (true) {
          if (socket.receive ()) {
            t.update();
            alreadyComplained=false;
            Velocity dv;
            if (socket.getVelocity (dv)) {
              BBOARD->setDesiredVelocity(dv);
              t.update();
            }
            bool b;
            if (socket.getActive (b)) {
              BBOARD->setActive (b);
              t.update();
            }
            std::string s;
            if (socket.getMessages (s)) {
              BBOARD->addMessage (s);
            }
            if (socket.getPlotCommand (s)) {
              BBOARD->addPlotCommand (s);
            }
            if (socket.getPing()) {
              first=true;
            }
          }
          if (t.elapsed_msec()>500) {
            BBOARD->setActive (false);
            if (!alreadyComplained)
              EOUT("RemoteControl: Stop due to timeout. Did not receive messages for " << t.elapsed_msec() << " msec\n");
            alreadyComplained=true;
            first=true;
          } else {
            Velocity vel = BBOARD->getDesiredVelocity();
            if (vel!=latestVelocity || first) {
              latestVelocity=vel;
              socket.putVelocity (vel);
            }
            Odometry odo = BBOARD->getOdometry();
            if (odo!=latestOdometry || first) {
              latestOdometry = odo;
              socket.putOdometry (odo);
            }
            Pose ps = BBOARD->getVehiclePose();
            if (ps!=latestPose || first) {
              latestPose = ps;
              socket.putPose (ps);
            }
            first=false;
            socket.send();
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(5));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::RemoteControl> application ("RemoteControl");

}
