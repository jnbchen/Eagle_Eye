
#include "LED.h"
#include <QtGui/QPainter>

using namespace DerWeg;

LED::LED ( QWidget* p, Qt::WFlags f) : QWidget (p,f) {
  color.push_back (Qt::lightGray);
  color.push_back (Qt::red);
  presentState=0;
  presentBlinkState=0;
  connect (&blinkTimer, SIGNAL(timeout()), this, SLOT(blinkEvent()));
}

LED::~LED () {;}

unsigned int LED::state () const throw () {
  return presentState;
}

bool LED::isBlinking () const throw () {
  return presentBlinkState!=0;
}

unsigned int LED::numStates () const throw () {
  return color.size();
}

void LED::setColor (unsigned int n, const QColor& c) throw () {
  while (color.size()<=n)
    color.push_back (Qt::lightGray);
  color[n]=c;
}

void LED::setState (unsigned int n) throw () {
  if (presentBlinkState!=0 || presentState!=n) {
    blinkTimer.stop ();
    presentState=(n>=color.size() ? color.size()-1 : n);
    presentBlinkState=0;
    repaint ();
  }
}

void LED::setBlinkState (unsigned int n) throw () {
  if (presentBlinkState==0 || presentState!=n) {
    presentState=(n>=color.size() ? color.size()-1 : n);
    presentBlinkState=1;
    blinkTimer.start (333);
    repaint ();
  }
}

void LED::setOn (bool b) throw () {
  setState (static_cast<unsigned int>(b));
}

void LED::setOn () throw () {
  setState (1);
}

void LED::setOff () throw () {
  setState (0);
}

void LED::paintEvent(QPaintEvent *) {
  QPainter paint (this);
  int w = width();
  int h = height();
  int r = ((h>w ? w : h )-5)/2;
  QMatrix mapping (1,0,0,1,w/2, h/2);
  paint.setWorldMatrix (mapping);

  QPen pen;
  pen.setWidth (2);
  pen.setColor (Qt::darkGray);
  QBrush brush;
  brush.setColor (color[presentState]);
  brush.setStyle (Qt::SolidPattern);
  paint.setPen (pen);
  if ((presentBlinkState==1) || (presentBlinkState==0))
    paint.setBrush (brush);
  paint.drawChord (-r,-r,2*r,2*r,0,5760);
  paint.setPen (pen);
  paint.drawArc (-r,-r,2*r,2*r,0,5760);
}

void LED::blinkEvent() {
  if (presentBlinkState==1)
    presentBlinkState=2;
  else if (presentBlinkState==2)
    presentBlinkState=1;
  repaint();
}

