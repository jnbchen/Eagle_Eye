/** Klasse zur Schaetzung einer Bewegung mit konstanter Geschwindigkeiten
 und Gierrate. Ursprunglich entwickelt im Rahmen des Brainstormers Tribots
 Projektes durch Martin Lauer (lauer@mrt.uka.de). Angepasst an
 Kognitives-Automobile-Labor */

#ifndef _DerWeg_SLVelocitySensor_h
#define _DerWeg_SLVelocitySensor_h

#include "../Elementary/RingBuffer.h"
#include "../Elementary/Timestamp.h"
#include "../Elementary/Vec.h"

namespace DerWeg {

  /** Klasse, um aus Selbstlokalisationspositionen die Robotergeschwindigkeit zu berechnen;
      Berechnung erfolgt mit Hilfe eines kleinsten Quadrateansatzes unter Annahme konstanter
      Rotations- und Translationsgeschwindigkeiten */
  class SLVelocitySensor {
  public:
    /** Konstruktor.
      arg1 = "Halbwertszeit" in sec der exponentiellen Glaettung, die verwendet wird
      arg2 = maximale Zeitintervall in sec bis zu dem Beobachtungen beruecksichtigt werden */
    SLVelocitySensor (double =0.25, double =1) throw (std::bad_alloc);
    /** eine neue Position arg1 und Orientierung arg2 zum Zeitpunkt arg3 einbinden */
    void update (const Vec&, const Angle&, Timestamp) throw ();
    /** die zuletzt berechnete Geschindigkeit bekommen;
        Arg1: Rueckgabewert Positionen (m)
        Arg2: Rueckgabewert Orientierung
        Arg3: Rueckgabewert Geschwindigkeit (m/s)
        Arg4: Rueckgabewert Winkelgeschwindigkeit (rad/s)
        Arg5: Rueckgabewert Zeitpunkt, fuer den die Berechnung erfolgt */
    void get (Vec&, Angle&, Vec&, double&, Timestamp&) const throw ();
  private:
    /** Struktur um Tripel aus Zeit, Position und Orientierung zu speichern */
    struct TPH {
      Timestamp timestamp;
      Vec pos;
      double heading;  // in rad
    };

    const unsigned int n;     ///< Groesse des Ringpuffers, Anzahl Beobachtungen
    unsigned int burn_in;     ///< ist >0, wenn der Ringpuffer noch teilweise uninitialisiert ist
    Vec pos;                  ///< zuletzt geschaetzte Position
    Angle heading;            ///< zuletzt geschaetzte Ausrichtung
    Vec vtrans;               ///< zuletzt geschaetzte Geschwindigkeit
    double vrot;              ///< zuletzt geschaetzte Winkelgeschwindigkeit
    Timestamp timestamp;      ///< Zeitpunkt der letzten Berechnung
    RingBuffer<TPH> buffer;   ///< Ringpuffer mit alten Positionen, Anker zeigt auf aeltestes Element

    double decay;             ///< Gewichtungsfaktor fuer exponentielle Gewichtung der Samples
    double max_delta_t;       ///< maximale Zeit zurueck, die beruecksichtigt wird
  };

}

#endif
