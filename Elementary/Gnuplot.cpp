
#include "Gnuplot.h"
#include <string>
#include <cstdio>

using namespace std;
using namespace DerWeg;

Gnuplot::Gnuplot (TerminalType tt) throw (std::invalid_argument) : plotpipe (NULL) {
  plotpipe = popen ("/usr/bin/gnuplot > /tmp/gnuplot_dump", "w");
  set_terminal (tt);
  if (plotpipe==NULL)
    throw std::invalid_argument ("Gnuplot: could not open gnuplot pipe");
}

Gnuplot::~Gnuplot () throw () {
  if (plotpipe)
    pclose (plotpipe);
}

void Gnuplot::operator() (const char* cmd) throw () {
  fprintf (plotpipe, "%s\n", cmd);
  fflush (plotpipe);
}

void Gnuplot::set_terminal (TerminalType tt) throw (std::invalid_argument) {
  switch (tt) {
  case x11 : operator() ("set terminal x11"); break;
  case eps : operator() ("set terminal postscript eps color solid"); break;
  case fig : operator() ("set terminal fig color pointsmax 20000 solid"); break;
  case epsmono : operator() ("set terminal postscript eps monochrome dashed"); break;
  case figmono : operator() ("set terminal fig monochrome pointsmax 20000 dashed"); break;
  default: throw std::invalid_argument ("Gnuplot: unknown terminal type");
  }
  operator() ("set output");
}

void Gnuplot::set_output (const char* file) throw () {
  std::string cmd = "set output '";
  cmd += file;
  cmd += "'";
  operator() (cmd.c_str());
}
