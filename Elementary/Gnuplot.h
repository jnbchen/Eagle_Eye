
#ifndef _DerWeg_Gnuplot_h_
#define _DerWeg_Gnuplot_h_

#include <stdexcept>

namespace DerWeg {

  /** Klasse Gnuplot bindet Gnuplot in ein Programm ein */
  class Gnuplot {
  public:
    enum TerminalType { x11, eps, fig, epsmono, figmono };
    /** Erzeugt ein Gnuplot-Programm, arg1 ist der Terminal-Typ */
    Gnuplot (TerminalType = x11) throw (std::invalid_argument);
    ~Gnuplot () throw ();
    /** Sendet ein String-Kommando an gnuplot; 
        Fehler werden auf der Kommandozeile angezeigt */
    void operator() (const char*) throw ();
    /** Terminal setzen */
    void set_terminal (TerminalType) throw (std::invalid_argument);
    /** Ausgabe-Datei setzen */
    void set_output (const char*) throw ();
  protected:
    FILE* plotpipe;
  };

}

#endif
