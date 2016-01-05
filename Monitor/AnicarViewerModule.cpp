
#include <iostream>
#include "../Elementary/KogmoThread.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Configuration.h"
#include "../Elementary/PluginFactory.h"
#include "../Elementary/Timestamp.h"
#include "../Elementary/ImageReaderWriter.h"
#include "../Elementary/UDP/KogmolaborCommunication.h"

using namespace std;

namespace DerWeg {

  class AnicarViewerModule : public KogmoThread {
  private:
    KogmolaborCommunication comm;

  public:
    AnicarViewerModule () {;}
    ~AnicarViewerModule () {;}

    void init (const ConfigReader&) {
      if (!comm.init_as_server(34396))
        throw std::invalid_argument ("AnicarViewerModule: cannot start socket");
    }
    void deinit () {
      comm.clear_send_buffer();
      comm.sayGoodbye();  // send goodbye to AnicarViewer
      comm.send();
      comm.close();
    }
    void execute () {
      try{
        ImageWriter iw;
        bool underRemoteControl = false;
        Timestamp latestReceive;
        while (true) {
          if (comm.receive()) {
            latestReceive.update();
            bool b;
            underRemoteControl = true;
            if (comm.getActive(b)) {
              BBOARD->setActive(b);
              if (!b) {
                Velocity nv;
                BBOARD->setDesiredVelocity(nv);
              }
            }
            if (comm.goodbye()) {
              underRemoteControl = false;
              EOUT("Stop vehicle when receiving goodbye from AnicarViewer\n");
              BBOARD->setActive(false);
            }
            Velocity dv;
            if (comm.getVelocity(dv))
              BBOARD->setDesiredVelocity(dv);
            comm.putPing();
            if (comm.askedForAllModules())
              comm.putAllModules(CONFIG->allModules());
            comm.putActive(BBOARD->getActive());
            comm.putPresentModules(CONFIG->presentModules());
            comm.putVelocity(BBOARD->getDesiredVelocity());
            comm.putOdometry(BBOARD->getOdometry());
            comm.putPose(BBOARD->getVehiclePose());
            string msg = BBOARD->getMessage();
            if (msg.length()>0)
              comm.putMessages(msg);
            msg = BBOARD->getPlotCommand();
            if (msg.length()>0)
              comm.putPlotCommand(msg);
            if (comm.getSaveImage()) {
              ImageBuffer ib = BBOARD->getImage();
              if (!ib.image.empty())
                iw.setImage (ib);
              else
                BBOARD->addMessage ("error: no image available");
            }
            std::string s1;
            if (comm.getAddModule(s1)) {
              if (!CONFIG->addModule(s1))
                BBOARD->addMessage (string("error: could not add module ")+s1);
            }
            std::string s2;
            if (comm.getRemoveModule(s2))
              CONFIG->removeModule(s2);

            comm.send();
          }

          if (underRemoteControl && latestReceive.elapsed_msec()>1000) {
            underRemoteControl = false;
            EOUT("Communication problems: stop vehicle\n");
            BBOARD->setActive(false);
          }

          boost::this_thread::sleep(boost::posix_time::milliseconds(50));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };


}

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::AnicarViewerModule> gui_monitor ("AnicarViewer");

}
