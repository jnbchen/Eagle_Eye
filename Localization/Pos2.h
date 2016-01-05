
#ifndef _Entzerrung_Pos2_h_
#define _Entzerrung_Pos2_h_

#include <cmath>
#include <vector>
#include <iostream>

namespace Entzerrung {

  /** Struktur, um 2-dimensionale ganzzahlige Koordinaten darzustellen */
  struct Pos2 {
    int x;
    int y;

    /** Default-Konstruktor */
    Pos2 (int i =0, int j=0) throw () : x(i), y(j) {;}
    /** Copy-Konstruktor */
    Pos2 (const Pos2& c) throw () : x(c.x), y(c.y) {;}
    /** Vergleich auf Gleichheit */
    bool operator== (const Pos2& c) const throw () { return x==c.x && y==c.y; }
    /** Vergleich auf Ungleichheit */
    bool operator!= (const Pos2& c) const throw () { return x!=c.x || y!=c.y; }
    /** Zuweisung */
    const Pos2& operator= (const Pos2& c) throw () { x=c.x; y=c.y; return *this; }
    /** elementweise Addition */
    const Pos2& operator+= (const Pos2& c) throw () { x+=c.x; y+=c.y; return *this; }
    /** elementweise Subtraktion */
    const Pos2& operator-= (const Pos2& c) throw () { x-=c.x; y-=c.y; return *this; }
    /** Skalierung (mal) */
    const Pos2& operator*= (double d) throw () { x=static_cast<int>(static_cast<double>(x)*d); y=static_cast<int>(static_cast<double>(y)*d); return *this; }
    /** Skalierung (geteilt) */
    const Pos2& operator/= (double d) throw () { x=static_cast<int>(static_cast<double>(x)/d); y=static_cast<int>(static_cast<double>(y)/d); return *this; }
    /** Addition */
    Pos2 operator+ (const Pos2& c) const throw () { Pos2 r (*this); r+=c; return r; }
    /** Subtraktion */
    Pos2 operator- (const Pos2& c) const throw () { Pos2 r (*this); r-=c; return r; }
    /** Skalierung (mal) */
    Pos2 operator* (double d) const throw () { Pos2 r (*this); r*=d; return r; }
    /** Skalierung (geteilt) */
    Pos2 operator/ (double d) const throw () { Pos2 r (*this); r/=d; return r; }
    /** Euklidischer Abstand zweier Punkte */
    double distance (const Pos2& c) const throw () { return std::sqrt ((double)((c.x-x)*(c.x-x)+(c.y-y)*(c.y-y))); }
  };

  /** Klasse, um Liste von Pixelpositionen zu repraesentieren */
  class Pixelset : public std::vector<Pos2> {
  public:
    void writeToStream (std::ostream&);
    void readFromStream (std::istream&);
  };

}

#endif

