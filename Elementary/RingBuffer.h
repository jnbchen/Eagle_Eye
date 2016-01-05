/** Klasse zur Implementierung eines Ringpuffers
 Entwickelt im Rahmen des Brainstormers Tribots Projektes durch Martin
 Lauer (lauer@mrt.uka.de) und getestet unter Linux */

#ifndef _DerWeg_RingBuffer_h_
#define _DerWeg_RingBuffer_h_

#ifdef WIN32
 #pragma warning(disable: 4290)
#endif

#include <vector>

namespace DerWeg {

  /** Klasse RingBuffer implementiert einen Ringpuffer fester Groesse */
  template<class T> class RingBuffer {
  private:
    int n;                    ///< Groesse des Puffers
    std::vector<T> elem;      ///< Speicherstruktur
    int anchor;               ///< Index des momentan ersten Elements

  public:
    /** Konstruktor, uebergibt die Groesse des Puffers */
    RingBuffer (unsigned int) throw (std::bad_alloc);
    /** Destruktor */
    ~RingBuffer () throw ();

    /** Groesse des Puffers liefern */
    unsigned int size () const throw ();
    /** Groesse des Puffers veraendern */
    void resize (unsigned int) throw (std::bad_alloc);
    /** Weiterbewegen der Verankerung um arg1 Elemente */
    void step (int =1) throw ();
    /** liefert das Ankerelement */
    const T& get () const throw ();
    /** liefert das Ankerelement */
    T& get () throw ();
    /** liefert das Element an Position Anker+arg */
    const T& operator[] (int) const throw ();
     /** liefert das Element an Position Anker+arg */
    T& operator[] (int) throw ();
    /** Element links vom Ankerelement durch neues überschreiben und Anker neu setzen */
    void add (T&) throw ();
  };

} // namespace DerWeg





// Implementierung, wegen template-Deklaration:
template<class T> DerWeg::RingBuffer<T>::RingBuffer (unsigned int n1) throw (std::bad_alloc) : n(n1), elem(n1), anchor(0) {;}

template<class T> DerWeg::RingBuffer<T>::~RingBuffer () throw () {;}

template<class T> unsigned int DerWeg::RingBuffer<T>::size () const throw () { return n; }

template<class T> void DerWeg::RingBuffer<T>::resize (unsigned int n1) throw (std::bad_alloc) {
  n=n1;
  elem.resize (n);
  if (anchor>=n)
    anchor=0;
}

template<class T> void DerWeg::RingBuffer<T>::step (int n1) throw () {
  anchor+=n1;
  while (anchor<0)
    anchor+=n;
  while (anchor>=n)
    anchor-=n;
}

template<class T> const T& DerWeg::RingBuffer<T>::get () const throw () { return elem[anchor]; }

template<class T> T& DerWeg::RingBuffer<T>::get () throw () { return elem[anchor]; }

template<class T> const T& DerWeg::RingBuffer<T>::operator[] (int i) const throw () {
  i+=anchor;
  while (i<0)
    i+=n;
  while (i>=n)
    i-=n;
  return elem[i];
}

template<class T> T& DerWeg::RingBuffer<T>::operator[] (int i) throw () {
  i+=anchor;
  while (i<0)
    i+=n;
  while (i>=n)
    i-=n;
  return elem[i];
}

template<class T> void DerWeg::RingBuffer<T>::add (T& e) throw () {
	step(-1);
	elem[anchor] = e;
	return;
}



#endif // _DerWeg_RingBuffer_h_

