/** Klasse zur Repraesentation von Winkeln.
 Entwickelt im Rahmen des Brainstormers Tribots Projektes durch Martin
 Lauer (lauer@mrt.uka.de) und getestet unter Linux */

#ifndef _DerWeg_Angle_h_
#define _DerWeg_Angle_h_

#include <cmath>

namespace DerWeg {

  /** Klasse zur Repraesentation und zum Rechnen mit Winkeln */
  class Angle {
  private:
    double the_angle;    ///< Winkel im Bogenmass, normiert auf [0,2pi)
  public:
    /** Default Konstruktor */
    Angle () throw () { the_angle = 0.;}
    /** Copy-Konstruktor */
    Angle (const Angle& a) throw () : the_angle(a.the_angle) {;}
    /** Zuweisungsoperator */
    const Angle& operator= (const Angle& a) throw () { the_angle=a.the_angle; return *this; }

    /** Vergleichsoperator */
    bool operator== (const Angle&) const throw ();
    bool operator!= (const Angle&) const throw ();
    /** pruefe, ob im math. positiven Sinne der Winkel zwischen dem ersten und zweiten Argument liegt */
    bool in_between (const Angle, const Angle) const throw ();

    /** Setzen des Winkels im Bogenmass */
    void set_rad (double) throw ();
    /** Winkel im Bogenmass, normiert auf [0,2pi) */
    double get_rad () const throw ();

    /** einen Winkel direkt in rad erzeugen */
    static Angle rad_angle (double) throw ();
    /** einen Winkel direkt in Grad erzeugen */
    static Angle deg_angle (double) throw ();

    /** Setzen des Winkels im Gradmass */
    void set_deg (double) throw ();
    /** Winkel im Gradmass, normiert auf [0,360) */
    double get_deg () const throw ();

    /** Winkel im Bogenmass, normiert auf [-pi,pi) */
    double get_rad_pi () const throw ();
    /** Winkel im Gradmass, normiert auf [-180,180) */
    double get_deg_180 () const throw ();

    /** Addition */
    Angle operator+ (const Angle) const throw ();
    const Angle& operator+= (const Angle) throw ();
    /** Subtraktion */
    Angle operator- (const Angle) const throw ();
    const Angle& operator-= (const Angle) throw ();
    /** Komplement (2pi-this) */
    Angle operator- () const throw ();
    /** Skalarmultiplikation */
    const Angle& operator*= (double) throw();

    // Spezielle Winkel:
    static const Angle zero;           ///< 0 Grad
    static const Angle twelvth;        ///< 30 Grad
    static const Angle eighth;         ///< 45 Grad
    static const Angle sixth;          ///< 60 Grad
    static const Angle quarter;        ///< 90 Grad
    static const Angle three_eighth;   ///< 135 Grad
    static const Angle half;           ///< 180 Grad
    static const Angle five_eighth;    ///< 225 Grad
    static const Angle three_quarters; ///< 270 Grad
    static const Angle five_sixth;     ///< 300 Grad
    static const Angle seven_eighth;   ///< 315 Grad
    static const Angle eleven_twelvth; ///< 330 Grad
  };

  /** Skalarmultiplikationen */
  Angle operator* (const Angle, double) throw ();
  /** Skalarmultiplikationen */
  Angle operator* (double, const Angle) throw ();
} // namespace DerWeg

namespace std { // namespace std hier, da sonst bei Aufruf explizite Instantiierung des Namespaces notwendig
  /** Sinus */
  inline double sin (const DerWeg::Angle& a) throw() { return std::sin(a.get_rad()); }
  /** Cosinus */
  inline double cos (const DerWeg::Angle& a) throw() { return std::cos(a.get_rad()); }
  /** Tangens */
  inline double tan (const DerWeg::Angle& a) throw() { return std::tan(a.get_rad()); }
  /** Cotangens */
  inline double cot (const DerWeg::Angle& a) throw() { return std::cos(a.get_rad())/std::sin(a.get_rad()); }
} // namespace std

#endif // _DerWeg_Angle_h_

