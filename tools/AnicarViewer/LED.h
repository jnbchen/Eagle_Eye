
#ifndef _DerWeg_LED_h_
#define _DerWeg_LED_h_

#include <QtGui/QWidget>
#include <QtGui/QPaintEvent>
#include <QtCore/QTimer>
#include <vector>

namespace DerWeg {

  /** Widget, das eine mehrfarbige LED zeichnet */
  class LED : public QWidget {
    Q_OBJECT

   public:
    /** Konstruktor */
    LED ( QWidget* = 0, Qt::WFlags = 0);
    ~LED ();

    /** liefert den aktuellen Zustand */
    unsigned int state () const throw ();
    /** liefere true, wenn geblinkt wird */
    bool isBlinking () const throw ();
    /** liefert die maximal verfuegbare Zahl Zustaende */
    unsigned int numStates () const throw ();
    /** setzt die Zeichenfarbe für den Zustand n auf c */
    void setColor (unsigned int n, const QColor& c) throw ();

  public slots:
    /** setze den Zustand n */
    void setState (unsigned int n) throw ();
    /** setze Zustand n und blinke */
    void setBlinkState (unsigned int n) throw ();
    /** setze den Zustand 1 oder 0 */
    void setOn (bool b) throw ();
    /** setze den Zustand 1 */
    void setOn () throw ();
    /** setze den Zustand 0 */
    void setOff () throw ();

  protected slots:
    void paintEvent(QPaintEvent *);
    
  private slots:
    void blinkEvent();

  private:
    std::vector<QColor> color;
    unsigned int presentState;
    int presentBlinkState;  // 0: no blink, 1: blink on, 2: blink off
    QTimer blinkTimer;    
 };

}

#endif

