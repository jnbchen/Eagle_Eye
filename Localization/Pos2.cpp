
#include "Pos2.h"

using namespace Entzerrung;
using namespace std;

void Pixelset::writeToStream (std::ostream& os) {
  os << size() << '\n';
  for (const_iterator it=begin(); it!=end(); ++it)
    os << it->x << ' ' << it->y << '\n';
}

void Pixelset::readFromStream (std::istream& is) {
  unsigned int s;
  is >> s;
  clear();
  if (is.fail())
    return;
  reserve(s);
  for (unsigned int i=0; i<s; i++) {
    Pos2 p;
    is >> p.x >> p.y;
    if (is.fail())
      return;
    push_back (p);
  }
}
 
