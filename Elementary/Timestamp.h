/** Klasse zur Repraesentation der Zeit seit dem Programmstart
 entwickelt im Rahmen des Brainstormers Tribots Projektes durch
 Martin Lauer (lauer@mrt.uka.de) und geeignet fuer Linux und Windows */

#ifndef _DerWeg_Timestamp_h_
#define _DerWeg_Timestamp_h_

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>

#ifdef WIN32
#include <time.h>
#include <windows.h>
#else
#include <sys/time.h>
#endif

namespace DerWeg {

  /** Klasse zur Repraesentation von Zeitpunkten. Der Zeitpunkt Null ist der
    Zeitpunkt, zu dem das aktuelle Programm gestartet wurde. Die Zeitangaben
    werden in Mikrosekunden-Genauigkeit verwaltet. Ob die Angaben tatsaechlich
    Mikrosekunden-genau sind haengt vom Betriebssystem ab. */
  class Timestamp {
  private:
    long int sec;   ///< Anzahl Sekunden
    long int usec;  ///< Anzahl Mikrosekunden

    static const Timestamp starting_time;   ///< Startzeit des Programms
    static const boost::posix_time::ptime time_origin;  ///< 1.1.1970 als Referenzzeitpunkt
  public:
    /** Konstruktor: setze aktuelle Systemzeit */
    Timestamp () throw ();
    /** Copy-Konstruktor */
    Timestamp (const Timestamp&) throw ();
    /** Setzen der Zeit aus timeval */
    Timestamp (const timeval&) throw ();
    /** Setzen aus Boost ptime */
    Timestamp (const boost::posix_time::ptime&) throw ();
    /** Zuweisungsoperator */
    const Timestamp& operator= (const Timestamp&) throw ();
    /** Zuweisungsoperator */
    const Timestamp& operator= (const boost::posix_time::ptime&) throw ();

    /** Typumwandlung in Boost ptime */
    operator boost::posix_time::ptime () const throw ();

    /** Zeit in Mikrosekunden seit dem Programmstart */
    long int get_usec () const throw ();
    /** Zeit in Millisekunden seit dem Programmstart */
    long int get_msec () const throw ();
    /** Zeit in Sekunden seit dem Programmstart */
    long int get_sec () const throw ();

    /** Zeit in Mikrosekunden seit Programmstart setzen */
    void set_usec (const long int&) throw ();
    /** Zeit in Millisekunden seit Programmstart setzen */
    void set_msec (const long int&) throw ();
    /** Zeit in Sekunden seit Programmstart setzen */
    void set_sec (const long int&) throw ();

    /** aktuelle Systemzeit setzen */
    void update () throw ();

    /** Differenz zwischen jetzt und der in *this
      gespeicherten Zeit in Mikrosekunden */
    long int elapsed_usec () const throw ();
    /** Differenz zwischen jetzt und der in *this
      gespeicherten Zeit in Millisekunden */
    long int elapsed_msec () const throw ();
    /** Differenz zwischen jetzt und der in *this
      gespeicherten Zeit in Sekunden */
    long int elapsed_sec () const throw ();

    /** Addiere Arg1 Mikrosekunden zu *this */
    void add_usec (long int) throw ();
    /** Addiere Arg1 Millisekunden zu *this */
    void add_msec (long int) throw ();
    /** Addiere Arg1 Sekunden zu *this */
    void add_sec (long int) throw ();

    /** Zeit-Differenz in Mikrosekunden (*this)-Arg1 */
    long int diff_usec (const Timestamp&) const throw ();
    /** Zeit-Differenz in Millisekunden (*this)-Arg1 */
    long int diff_msec (const Timestamp&) const throw ();
    /** Zeit-Differenz in Sekunden (*this)-Arg1 */
    long int diff_sec (const Timestamp&) const throw ();

    /** Vergleichsoperator == */
    bool operator== (const Timestamp&) const throw ();
    /** Vergleichsoperator != */
    bool operator!= (const Timestamp&) const throw ();
    /** Vergleichsoperator <= */
    bool operator<= (const Timestamp&) const throw ();
    /** Vergleichsoperator < */
    bool operator< (const Timestamp&) const throw ();
    /** Vergleichsoperator >= */
    bool operator>= (const Timestamp&) const throw ();
    /** Vergleichsoperator > */
    bool operator> (const Timestamp&) const throw ();

    /** explizites Setzen der Zeit mit einem Linux timeval */
    void set (const timeval&) throw ();
    /** explizites Holen der Zeit als Linux timeval */
    void get (timeval&) const throw ();
  };

} // namespace DerWeg

std::ostream& operator<< (std::ostream&, const DerWeg::Timestamp&) throw();   ///< schreibe Zeit seit Programmstart in Millisekunden

#endif
