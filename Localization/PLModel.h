
#ifndef _Entzerrung_PLModel_h_
#define _Entzerrung_PLModel_h_

#include "Pos2.h"
#include <iostream>

namespace Entzerrung {

  struct PLModelParameterset {
    double cx;  ///< Mittelpunkt, x-Koordinate
    double cy;  ///< Mittelpunkt, y-Koordinate
    std::vector<double> r;  ///< Stuetzstellen, Radii
    std::vector<double> h;  ///< Stuetzstellen, Hoehe
  };

  class PLModel {
    PLModelParameterset parameters;

  public:
    /** Standardinitialisierung: 'cx', 'cy' definiert den Mittelpunkt, 
        'sd' den Stuetzstellenabstand, 'n' die Anzahl Stuetzintervalle */
    PLModel (double cx, double cy, double sd =100, unsigned int n=1);
    /** initialisierung mit Parameter-Vektor 'p' */
    PLModel (const PLModelParameterset& p);
    /** Parameter holen */
    const PLModelParameterset& getParameters() const;
    /** Parameter setzen */
    void setParameters (const PLModelParameterset&);
    /** Parameter nach Stream schreiben */
    void writeParametersToStream (std::ostream&);
    /** Parameter aus Stream lesen */
    void readParametersFromStream (std::istream&);
    /** Anzahl Stuetzstellen liefern (inkl. Stuetzstelle bei Null) */
    unsigned int numSegments () const;
    
    /** Punkt 'p' abbilden und auf Integer runden */
    Pos2 mapInt (Pos2 p) const;
    /** Punkt 'p' auf (x,y) abbilden */
    void mapDouble (double& x, double& y, Pos2 p) const;

    /** Abbildung als Klassenmethode. Arbeitet mit 'params' als Parametervektor, liefert den Ringindex in dem der Punkt liegt */
    static unsigned int mapDouble (double& x, double& y, Pos2 p, const PLModelParameterset& params);

    /** k-ten Ring entfernen */
    void eraseRing (unsigned int k);
    /** k-ten Ring in zwei aufspalten */
    void splitRing (unsigned int k);

    /** invertiere die Funktion */
    void invert ();
  };
  
}

#endif

