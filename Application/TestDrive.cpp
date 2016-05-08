
#include "../Elementary/KogmoThread.h"
#include "../Elementary/PluginFactory.h"
#include "../Blackboard/Blackboard.h"



namespace DerWeg {

  /** eine sehr einfache Beispiel- und Testanwendung, die das Fahrzeug lediglich auf einer Schlangelinie fahren laesst */
  class TestDrive : public KogmoThread {
    int counter=0;
  public:
    TestDrive () {;}
    ~TestDrive () {;}
    void setMotion(double v, double ang){
        Velocity dv;
        dv.velocity = v;
        dv.steer=Angle::deg_angle (ang);
        BBOARD->setDesiredVelocity(dv);
    }
    void execute () {
      try{
        while (true) {
            Pose vp;
            vp = BBOARD->getVehiclePose();
            if(vp.position.x>0 && vp.position.x<8000){
                setMotion(3, 0);
            }else{
                setMotion(0,0);
            }
            LOUT(vp.position.x << std::endl);
            //counter++;

          boost::this_thread::sleep(boost::posix_time::milliseconds(20));
          boost::this_thread::interruption_point();
        }
      }catch(boost::thread_interrupted&){;}
    }
  };

} // namespace DerWeg

namespace {

  // Plugin bei der Factory anmelden
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TestDrive> application ("TestDrive");

}
