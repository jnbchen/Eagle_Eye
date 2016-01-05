
#include "Joystick.h"
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>

using namespace std;
using namespace DerWeg;

namespace {
  const double MAX_AXIS_VALUE = 32768;   // Groesste Auslenkung der Achsen
}

Joystick::Joystick (const char* devname) throw (std::invalid_argument, std::bad_alloc) : axis (6), button (12) {
  file_descriptor = open(devname, O_RDONLY);
  if (file_descriptor<0)
    throw std::invalid_argument ("invalid device name in Joystick::Joystick");

  fcntl(file_descriptor, F_SETFL, O_NONBLOCK); //don't block
  unsigned char n;
  ioctl(file_descriptor, JSIOCGAXES, &n);  // Anzahl Achsen abfragen
  axis.resize (n);
  ioctl(file_descriptor, JSIOCGBUTTONS, &n);  // Anzahl Knoepfe abfragen
  button.resize (n);
}

Joystick::~Joystick () throw () {
  close (file_descriptor);
}

void Joystick::update () throw () {
  js_event ev;
  int evsize = sizeof (struct js_event);
  while (true) {
    int size = read (file_descriptor, &ev, evsize);
    if (size!=evsize)
      break;

    if (ev.type==JS_EVENT_BUTTON)
      button[ev.number]=(ev.value==1);
    else if (ev.type==JS_EVENT_AXIS)
      axis[ev.number]=static_cast<double>(ev.value)/MAX_AXIS_VALUE;
  }
}

const std::vector<double>& Joystick::get_axis_state () throw () {
  update ();
  return axis;
}

const std::vector<bool>& Joystick::get_button_state () throw () {
  update ();
  return button;
}

std::string Joystick::get_type () throw (std::bad_alloc) {
  char buffer [500];
  ioctl(file_descriptor, JSIOCGNAME(500), buffer);
  return std::string (buffer);
}

int Joystick::get_version () throw () {
  int n;
  ioctl(file_descriptor, JSIOCGVERSION, &n);
  return n;
}

