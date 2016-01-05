#ifndef _KOGMOTHREAD_H__
#define _KOGMOTHREAD_H__

#include "ConfigReader.h"
#include <boost/thread.hpp>

namespace DerWeg {

 /** Eine Klasse, die die Threaderzeugung und Beendigung kapselt und die man
     zusammen mit der PlauginFactory nutzen kann */
 class KogmoThread {
 public:
  KogmoThread ();
  virtual ~KogmoThread ();

  virtual void init (const ConfigReader&) {;}   ///< das Modul initialisieren, Treiber laden, etc. Parameter stehen im ConfigReader
  virtual void deinit () {;}   ///< das Modul deinitialisieren, Treiber freigeben

  virtual void runThread ();  ///< startet execute() in einem Thread
  virtual void stopThread ();  ///< beendet den Thread mittels interrupt()-Aufruf

 protected:
  virtual void execute () = 0;    ///< die Hauptfunktionalitaet des Moduls, z.B. eine Endlosschleife, Thread-sicher. Sollte einen Interrupt-Punkt enthalten

 private:
  boost::thread* the_thread; ///< Thread fuer Modul
 };

}

#endif // KOGMOTHREAD_H_
