
#include "MapWidget.h"
#include <QtGui/QPainter>
#include <cmath>

using namespace std;
using namespace DerWeg;

namespace {
  inline QPointF vec2qpointf (const Vec& v) {
    return QPointF (v.x, v.y);
  }
  const string plotCommands [] = {
    string("dashed"),
    string("dotted"),
    string("thick"),
    string("thin"),
    string("black"),
    string("white"),
    string("red"),
    string("green"),
    string("blue"),
    string("yellow"),
    string("cyan"),
    string("magenta"),
    string("gray"),
    string("darkRed"),
    string("darkGreen"),
    string("darkBlue"),
    string("darkYellow"),
    string("darkCyan"),
    string("darkMagenta"),
    string("darkGray"),
    string("lightGray"),
    string("cross"),
    string("plus"),
    string("dot"),
    string("line"),
    string("arrow"),
    string("end_of_command_list"),
  };
  const unsigned numPlotCommands = 27;
}

DerWeg::MapWidget::MapWidget(QWidget* p, Qt::WindowFlags f) : QWidget (p,f), background (NULL), vehicle_centered_display (false), zoom_factor(1), do_trace(false), orientation(0)
{
  setBackgroundPixmap (NULL, Vec(-1000,-1000), Vec(13000,8000));
}


MapWidget::~MapWidget() {
  if (background) {
    delete background;
  }
}


void DerWeg::MapWidget::setBackgroundPixmap (const QPixmap* pm, Vec leftlower, Vec rightupper) {
  if (background) {
    delete background;
  }
  if (pm)
    background = new QPixmap (pm->transformed (QTransform(1,0,0,0,-1,0,0,0,1)));  // flip image to transform from left handed coordinate system to right handed
  else {
    QPixmap* pm = new QPixmap (1,1);
    pm->fill (Qt::white);
    background = pm;
  }
  pixmap_left_lower=leftlower;
  pixmap_right_upper=rightupper;
  if (leftlower.x>rightupper.x) {
    double x=leftlower.x;
    leftlower.x=rightupper.x;
    rightupper.x=x;
  }
  if (leftlower.y>rightupper.y) {
    double y=leftlower.y;
    leftlower.y=rightupper.y;
    rightupper.y=y;
  }
  update();
}

void DerWeg::MapWidget::setOrientation (unsigned int ori) {
  orientation=ori%4;
  repaint ();
}

unsigned int DerWeg::MapWidget::getOrientation () {
  return orientation;
}

void DerWeg::MapWidget::setVehiclePose (Vec pos, Angle ori, double vel, Angle steer) {
  vehicle_position=pos;
  vehicle_orientation=ori;
  vehicle_velocity=vel;
  vehicle_steering_angle=steer;
  if (do_trace)
    trace_pos.push_back (pos);
}


void DerWeg::MapWidget::paintEvent (QPaintEvent*) {
  // calculate optimal scaling and offset
  double widgetheight = height();
  double widgetwidth = width();
  double backgroundheight = pixmap_right_upper.y-pixmap_left_lower.y;
  double backgroundwidth = pixmap_right_upper.x-pixmap_left_lower.x;
  QPainter paint (this);
  double s=1;
  if (orientation==0 || orientation==2) {
    double sx = widgetwidth/backgroundwidth;
    double sy = widgetheight/backgroundheight;
    s = min(sx, sy);
    if (vehicle_centered_display) {
      s*=zoom_factor;
      double offx = 0.5*widgetwidth-s*vehicle_position.x;
      double offy = 0.5*widgetheight-s*vehicle_position.y;
      if (orientation==0)
        paint.setWorldMatrix (QMatrix (s, 0, 0, -s, offx, widgetheight-1-offy));
      else
        paint.setWorldMatrix (QMatrix (-s, 0, 0, s, widgetwidth-1-offx, offy));
    } else {
      double offx = 0.5*(widgetwidth-backgroundwidth*s)-s*pixmap_left_lower.x;
      double offy = 0.5*(widgetheight-backgroundheight*s)-s*pixmap_left_lower.y;
      if (orientation==0)
        paint.setWorldMatrix (QMatrix (s, 0, 0, -s, offx, widgetheight-1-offy));
      else
        paint.setWorldMatrix (QMatrix (-s, 0, 0, s, widgetwidth-1-offx, offy));
    }
  } else {
    double sx = widgetwidth/backgroundheight;
    double sy = widgetheight/backgroundwidth;
    s = min(sx, sy);
    if (vehicle_centered_display) {
      s*=zoom_factor;
      double offx = 0.5*widgetwidth-s*vehicle_position.y;
      double offy = 0.5*widgetheight-s*vehicle_position.x;
      if (orientation==1)
        paint.setWorldMatrix (QMatrix (0, -s, -s, 0, widgetwidth-1-offx, widgetheight-1-offy));
      else
        paint.setWorldMatrix (QMatrix (0, s, s, 0, offx, offy));
    } else {
      double offx = 0.5*(widgetwidth-backgroundheight*s)-s*pixmap_left_lower.y;
      double offy = 0.5*(widgetheight-backgroundwidth*s)-s*pixmap_left_lower.x;
      if (orientation==1)
        paint.setWorldMatrix (QMatrix (0, -s, -s, 0, widgetwidth-1-offx, widgetheight-1-offy));
      else
        paint.setWorldMatrix (QMatrix (0, s, s, 0, offx, offy));
    }
  }

  // draw background pixmap (or rectangle if pixmap unavailable)
  QRectF targetRect (min(pixmap_left_lower.x, pixmap_right_upper.x), min(pixmap_left_lower.y, pixmap_right_upper.y), backgroundwidth, backgroundheight);
  if (background) {
    QRectF sourceRect (0,0,background->width(),background->height());
    paint.drawPixmap (targetRect, *background, sourceRect);
  }

  // draw trace
  if (do_trace) {
    QPen pen (Qt::blue);
    pen.setStyle (Qt::DotLine);
    pen.setWidth (sqrt(zoom_factor)/s);
    paint.setPen (pen);
    QPointF* cps = new QPointF [trace_pos.size()];
    for (unsigned int i=0; i<trace_pos.size(); ++i)
      cps[i]=vec2qpointf (trace_pos[i]);
    paint.drawPolyline (cps, trace_pos.size());
    delete [] cps;
  }

  // draw vehicle
  double vhw = 210;  // 1/2 width of vehicle
  double vfl = 160;  // distance front axle to front
  double vbl = 670;  // distance front axle to rear

  Vec unitlong = Vec::unit_vector (vehicle_orientation);
  Vec unitortho = unitlong.rotate_quarter ();
  Vec cfl = vehicle_position+vfl*unitlong+vhw*unitortho;
  Vec cfr = vehicle_position+vfl*unitlong-vhw*unitortho;
  Vec cbl = vehicle_position-vbl*unitlong+vhw*unitortho;
  Vec cbr = vehicle_position-vbl*unitlong-vhw*unitortho;
  Vec ml = vehicle_position+vhw*unitortho;
  Vec mr = vehicle_position-vhw*unitortho;
  Vec mf = vehicle_position+vfl*unitlong;

  QPointF cornerpoints [4] = {
    vec2qpointf (cfl),
    vec2qpointf (cfr),
    vec2qpointf (cbr),
    vec2qpointf (cbl) };

  QPen pen (Qt::blue);
  pen.setStyle (Qt::SolidLine);
  pen.setWidth (2*sqrt(zoom_factor)/s);
  paint.setPen (pen);
  paint.drawPolygon (cornerpoints, 4);

  // draw wheels:
  Vec unitwheel = Vec::unit_vector (vehicle_orientation+vehicle_steering_angle);
  Vec wlf = vehicle_position-0.8*vhw*unitortho+100*unitwheel;
  Vec wlb = vehicle_position-0.8*vhw*unitortho-100*unitwheel;
  Vec wrf = vehicle_position+0.8*vhw*unitortho+100*unitwheel;
  Vec wrb = vehicle_position+0.8*vhw*unitortho-100*unitwheel;
  pen.setColor (Qt::red);
  pen.setWidth (2.5*sqrt(zoom_factor)/s);
  paint.setPen (pen);
  paint.drawLine (vec2qpointf(wlf), vec2qpointf(wlb));
  paint.drawLine (vec2qpointf(wrf), vec2qpointf(wrb));

  // draw velocity vector
  pen.setCapStyle (Qt::RoundCap);
  if (vehicle_velocity<0)
    pen.setColor(Qt::darkRed);
  paint.setPen (pen);
  Vec unitwheelortho=unitwheel.rotate_quarter();
  Vec af = vehicle_position+500*vehicle_velocity*unitwheel;
  Vec al = af+70*unitwheelortho;
  Vec ar = af-70*unitwheelortho;
  if (vehicle_velocity>1e-5) {
    al = af-100*unitwheel+70*unitwheelortho;
    ar = af-100*unitwheel-70*unitwheelortho;
  } else if (vehicle_velocity<-1e-5) {
    al = af+100*unitwheel+70*unitwheelortho;
    ar = af+100*unitwheel-70*unitwheelortho;
  }
  paint.drawLine (vec2qpointf(vehicle_position), vec2qpointf(af));
  paint.drawLine (vec2qpointf(al), vec2qpointf(af));
  paint.drawLine (vec2qpointf(ar), vec2qpointf(af));

  if (plotcmd.length()>0) {
    pen.setColor (Qt::black);
    pen.setStyle (Qt::SolidLine);
    pen.setWidth (1.0*sqrt(zoom_factor)/s);
    paint.setPen (pen);

    // plotcmd aufspalten
    vector<string> parts;
    unsigned int index=0;
    for (unsigned int i=0; i<=plotcmd.length(); ++i) {
      if (i==plotcmd.length() || plotcmd[i]==' ' || plotcmd[i]=='\t' || plotcmd[i]=='\n') {  // lazy evaluation sei Dank
        if (index<i) {
          parts.push_back (plotcmd.substr(index,i-index));
        }
        index=i+1;
      }
    }
    parts.push_back ("end_of_command_list");

    vector<string> args;
    string latest_cmd="";
    for (unsigned int i=0; i<parts.size(); ++i) {
      string cmd="";
      for (unsigned int j=0; j<numPlotCommands; ++j) {
        if (parts[i]==plotCommands[j]) {
          cmd=parts[i];
          break;
        }
      }

      if (cmd=="") {
        args.push_back (parts[i]);
        continue;
      }
      int n=args.size();

      if (latest_cmd=="dashed") {
        pen.setStyle (Qt::DashLine);
      } else if (latest_cmd=="dotted") {
        pen.setStyle (Qt::DotLine);
      } else if (latest_cmd=="thick") {
        pen.setWidth (2.5*sqrt(zoom_factor)/s);
      } else if (latest_cmd=="thin") {
        pen.setWidth (1.0*sqrt(zoom_factor)/s);
      } else if (latest_cmd=="black") {
        pen.setColor (Qt::black);
      } else if (latest_cmd=="white") {
        pen.setColor (Qt::white);
      } else if (latest_cmd=="red") {
        pen.setColor (Qt::red);
      } else if (latest_cmd=="green") {
        pen.setColor (Qt::green);
      } else if (latest_cmd=="blue") {
        pen.setColor (Qt::blue);
      } else if (latest_cmd=="yellow") {
        pen.setColor (Qt::yellow);
      } else if (latest_cmd=="cyan") {
        pen.setColor (Qt::cyan);
      } else if (latest_cmd=="magenta") {
        pen.setColor (Qt::magenta);
      } else if (latest_cmd=="gray") {
        pen.setColor (Qt::gray);
      } else if (latest_cmd=="darkRed") {
        pen.setColor (Qt::darkRed);
      } else if (latest_cmd=="darkGreen") {
        pen.setColor (Qt::darkGreen);
      } else if (latest_cmd=="darkBlue") {
        pen.setColor (Qt::darkBlue);
      } else if (latest_cmd=="darkYellow") {
        pen.setColor (Qt::darkYellow);
      } else if (latest_cmd=="darkCyan") {
        pen.setColor (Qt::darkCyan);
      } else if (latest_cmd=="darkMagenta") {
        pen.setColor (Qt::darkMagenta);
      } else if (latest_cmd=="darkGray") {
        pen.setColor (Qt::darkGray);
      } else if (latest_cmd=="lightGray") {
        pen.setColor (Qt::lightGray);
      } else if (latest_cmd=="cross") {
        for (int j=0; j<n/2; ++j) {
          int x = static_cast<int>(QString(args[2*j].c_str()).toDouble());
          int y = static_cast<int>(QString(args[2*j+1].c_str()).toDouble());
          paint.drawLine (x-100, y-100, x+100, y+100);
          paint.drawLine (x-100, y+100, x+100, y-100);
        }
      } else if (latest_cmd=="plus") {
        for (int j=0; j<n/2; ++j) {
          int x = static_cast<int>(QString(args[2*j].c_str()).toDouble());
          int y = static_cast<int>(QString(args[2*j+1].c_str()).toDouble());
          paint.drawLine (x-140, y, x+140, y);
          paint.drawLine (x, y+140, x, y-140);
        }
      } else if (latest_cmd=="dot") {
        for (int j=0; j<n/2; ++j) {
          int x = static_cast<int>(QString(args[2*j].c_str()).toDouble());
          int y = static_cast<int>(QString(args[2*j+1].c_str()).toDouble());
          paint.drawLine (x-1, y, x+1, y);
          paint.drawLine (x, y+1, x, y-1);
        }
      } else if (latest_cmd=="line") {
        if (n>=4) {
          int x = static_cast<int>(QString(args[0].c_str()).toDouble());
          int y = static_cast<int>(QString(args[1].c_str()).toDouble());
          for (int j=0; j<(n-2)/2; ++j) {
            int x1 = static_cast<int>(QString(args[2*j+2].c_str()).toDouble());
            int y1 = static_cast<int>(QString(args[2*j+3].c_str()).toDouble());
            paint.drawLine (x1, y1, x, y);
            x=x1;
            y=y1;
          }
        }
      } else if (latest_cmd=="arrow") {
        for (int j=0; j<n/4; ++j) {
          double x = QString(args[4*j+0].c_str()).toDouble();
          double y = QString(args[4*j+1].c_str()).toDouble();
          double x1 = QString(args[4*j+2].c_str()).toDouble();
          double y1 = QString(args[4*j+3].c_str()).toDouble();
          paint.drawLine (static_cast<int>(x), static_cast<int>(y), static_cast<int>(x1), static_cast<int>(y1));
          if (x!=x1 || y!=y1) {
            double l=std::sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
            double rvx = (x1-x)/l;
            double rvy = (y1-y)/l;
            paint.drawLine (static_cast<int>(x1), static_cast<int>(y1), static_cast<int>(x1-100*rvx+80*rvy), static_cast<int>(y1-100*rvy-80*rvx));
            paint.drawLine (static_cast<int>(x1), static_cast<int>(y1), static_cast<int>(x1-100*rvx-80*rvy), static_cast<int>(y1-100*rvy+80*rvx));
          }
        }
      }
      paint.setPen (pen);
      args.clear();
      latest_cmd=cmd;
    }
  }
}

void DerWeg::MapWidget::setFullCourse () {
  vehicle_centered_display=false;
  zoom_factor=1;
  repaint();
}

void DerWeg::MapWidget::setZoom (double f) {
  vehicle_centered_display=true;
  zoom_factor=f;
  repaint();
}

double DerWeg::MapWidget::getZoom () {
  return zoom_factor;
}

void DerWeg::MapWidget::setTrace (bool b) {
  do_trace=b;
  if (!do_trace)
    trace_pos.clear();
}

void DerWeg::MapWidget::setPlotcmd (const std::string& s) {
  plotcmd=s;
}
