
#include "../Elementary/KogmoThread.h"
#include "../Blackboard/Blackboard.h"
#include "../Elementary/Configuration.h"
#include "../Elementary/PluginFactory.h"
#include "../Elementary/Timestamp.h"
#include <termios.h>
#include <fcntl.h>

namespace {

  void writeStringList (const std::vector<std::string>& sl) {
    LOUT("[Modules]\n");
    for (unsigned int i=0; i<sl.size(); ++i)
      LOUT("  " << sl[i] << '\n');
  }

}

namespace DerWeg {

  /** eine auf termios basierende Implementierung. Fragt lediglich die
      Tastatur ab und bricht das Programm bei Betaetigen von 'q' ab */
  class TUIMonitor : public KogmoThread {
  private:
    struct termios termattr, save_termattr;
  public:
    TUIMonitor () {
      tcgetattr (STDIN_FILENO, &save_termattr);
      termattr=save_termattr;
      termattr.c_lflag&=~ICANON;
      termattr.c_lflag&=~ECHO;
      tcsetattr (STDIN_FILENO, TCSANOW, &termattr);
      fcntl(STDIN_FILENO, F_SETFL, fcntl(STDIN_FILENO, F_GETFL) | O_NONBLOCK);
    }
    ~TUIMonitor () {
      tcsetattr (STDIN_FILENO, TCSANOW, &save_termattr);
    }
    void infotext () const throw () {
      LOUT("[Tastatur-Codes]\n");
      LOUT("  'q' beendet das Programm" << '\n');
      LOUT("  'h' liefert diesen Hilfetext" << '\n');
      LOUT("  'g' Fahrzeug aktivieren" << '\n');
      LOUT("  <blank> Fahrzeug deaktivieren" << '\n');
      LOUT("  's' liefert aktuellen Fahrzeugstatus" << '\n');
      LOUT("  'c' liefert die ausgewaehlten Module" << '\n');
      LOUT("  '+' Fahrzeug beschleunigen" << '\n');
      LOUT("  '-' Fahrzeug abbremsen" << '\n');
      LOUT("  'y' nach links lenken" << '\n');
      LOUT("  'x' nach rechts lenken" << '\n');
      LOUT(std::flush);
    }
    void init (const ConfigReader&) {
      // Terminal auf zeichenweises Einlesen stellen und bisherige Konfiguration merken
      infotext();
    }
    void deinit () {
      //tcsetattr (STDIN_FILENO, TCSANOW, &save_termattr);
    }
    void execute () {
      try{
        while (true) {
          unsigned char c='.';
          ssize_t size = read (STDIN_FILENO, &c, 1);
          Velocity dv = BBOARD->getDesiredVelocity();
          if (size>0) {
            switch (c) {
              case 'Q': case 'q': // Programm beenden
                LOUT("[Quit]\n");
                BBOARD->setExitProgram(); return;
              case 'g': // Fahrzeug aktivieren
                LOUT("[Activate]\n");
                BBOARD->setActive(true); break;
              case ' ': // Fahrzeug deaktivieren
                LOUT("[Deactivate]\n";);
                BBOARD->setActive(false);
                dv.velocity=0;
                dv.steer=Angle::deg_angle(0);
                BBOARD->setDesiredVelocity(dv);
                break;
              case '+': // Fahrzeug beschleunigen
                dv.velocity+=0.1; BBOARD->setDesiredVelocity(dv);
                LOUT("[Velocity=" << dv.velocity << "]\n"); break;
              case '-': // Fahrzeug abbremsen/rueckwaertsfahren
                dv.velocity-=0.1; BBOARD->setDesiredVelocity(dv);
                LOUT("[Velocity=" << dv.velocity << "]\n"); break;
              case 'y': // nach links lenken
                dv.steer=dv.steer+Angle::deg_angle(5); BBOARD->setDesiredVelocity(dv);
                LOUT("[Steering=" << dv.steer.get_deg_180() << "]\n"); break;
              case 'x': // nach rechts lenken
                dv.steer=dv.steer-Angle::deg_angle(5); BBOARD->setDesiredVelocity(dv);
                LOUT("[Steering=" << dv.steer.get_deg_180() << "]\n"); break;
              case 'H': case 'h': // help
                infotext(); break;
              case 's': // Fahrzeugstatus
                {
                Timestamp now;
                unsigned int now_msec = now.get_msec()%1000;
                Odometry odo = BBOARD->getOdometry();
                Pose pos = BBOARD->getVehiclePose();
                LOUT("[Status]\n");
                LOUT((BBOARD->getActive() ? "  Fahrzeug aktiviert" : "  Fahrzeug deaktiviert") << '\n');
                LOUT("  Programmzeit = " << now.get_msec()/1000 << '.' << (now_msec<10 ? "0" : "") << (now_msec<100 ? "0" : "") << now_msec << '\n');
                LOUT("  v_soll = " << dv.velocity << "m/s\n");
                LOUT("  v_ist(odo) = " << odo.velocity << "m/s\n");
                LOUT("  v_ist(sl) = " << pos.velocity << "m/s\n");
                LOUT("  delta_soll = " << dv.steer.get_deg_180() << "°\n");
                LOUT("  delta_ist(odo) = " << odo.steer.get_deg_180() << "°\n");
                LOUT("  delta'(sl) = " << pos.yawrate << "rad/s\n");
                LOUT("  pos(sl) = (" << pos.position.x << ',' << pos.position.y << ") mm\n");
                LOUT("  ori(sl) = " << pos.orientation.get_deg() << "°\n");
                LOUT(std::flush);
                }
                break;
              case 'c': // Softwareconfiguration
                writeStringList (CONFIG->presentModules());
                break;
              default: break;
            }
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
  static DerWeg::PluginBuilder<DerWeg::KogmoThread, DerWeg::TUIMonitor> tui_monitor ("TUI");

}
