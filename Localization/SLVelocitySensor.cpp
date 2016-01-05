
#include "SLVelocitySensor.h"
#include <cmath>

#define ZWEIPI 6.283185307179586232

using namespace DerWeg;
using namespace std;

SLVelocitySensor::SLVelocitySensor (double e1, double md) throw (std::bad_alloc) : n(30), burn_in(30), buffer(30), decay(log(2)/(1000*e1)), max_delta_t(1e3*md) {
  pos = Vec::zero_vector;
  heading = Angle::zero;
  vtrans = Vec::zero_vector;
  vrot = 0;
}

void SLVelocitySensor::get (Vec& p, Angle& a, Vec& v, double& w, Timestamp& t) const throw () {
  p=pos;
  a=heading;
  v=vtrans;
  w=vrot;
  t=timestamp;
}

void SLVelocitySensor::update (const Vec& pos1, const Angle& heading1, Timestamp t_ref) throw () {
  // neue Beobachtung in Puffer einfuegen
  TPH new_tph;
  new_tph.timestamp = t_ref;
  new_tph.pos=pos1;  // weil aus historischen Gruenden intern mit mm gerechnet wird
  new_tph.heading=heading1.get_rad();
  buffer[0]=new_tph;
  buffer.step ();

  if (burn_in>0)
    burn_in--;

  if (burn_in==0) {
    // Modell neu berechnen, andernfalls Initialisierungphase, in der keine Modelle berechnet werden

    // 1. Schritt: Rotationsgeschwindigkeit und Anfangsorientierung berechnen
    double num=0;
    double sum_t=0;        // sum_i t_i
    double sum_tt=0;       // sum_i (t_i)^2
    double sum_phi=0;      // sum_i phi_i
    double sum_phit=0;     // sum_i (phi_i t_i)
    double latest_phi=0;
    for (unsigned int i=0; i<n; i++) {
      double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
      double w = exp(t*decay);
      if (abs(t)<max_delta_t) {
        double phi = buffer.get().heading;
        while (phi>=latest_phi+M_PI)
          phi-=ZWEIPI;
        while (phi<latest_phi-M_PI)
          phi+=ZWEIPI;
        latest_phi=phi;
        sum_t+=w*t;
        sum_tt+=w*t*t;
        sum_phi+=w*phi;
        sum_phit+=w*t*phi;
        num+=w;
      }
      buffer.step();
    }
    double det = num*sum_tt-sum_t*sum_t;
    if (det==0) {
      heading.set_rad (sum_phi/num);
      vrot = 0;
    } else {
      heading.set_rad ((sum_tt*sum_phi-sum_t*sum_phit)/det);
      vrot = ((-sum_t*sum_phi+num*sum_phit)/det);
    }

    if (abs(vrot)<=0.1e-3) {
      // Fall 2.1: vrot~=~0; berechne Anfangsposition und Geschwindigkeiten aus geradliniger Bewegung
      double sum_x=0;   // sum_i x_i
      double sum_y=0;   // sum_i y_i
      double sum_xt=0;  // sum_i (x_i*t_i)
      double sum_yt=0;  // sum_i (y_i*t_i)
      for (unsigned int i=0; i<n; i++) {
        double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
        double w = exp(t*decay);
        double x = buffer.get().pos.x;
        double y = buffer.get().pos.y;
        if (abs(t)<max_delta_t) {
          sum_x+=w*x;
          sum_y+=w*y;
          sum_xt+=w*t*x;
          sum_yt+=w*t*y;
        }
        buffer.step();
      }
      if (det==0) {
        pos.x = sum_x/num;
        pos.y = sum_y/num;
        vtrans = Vec::zero_vector;
      } else {
        pos.x = (sum_tt*sum_x-sum_t*sum_xt)/det;
        pos.y = (sum_tt*sum_y-sum_t*sum_yt)/det;
        vtrans = (Vec ( (-sum_t*sum_x+num*sum_xt)/det,  (-sum_t*sum_y+num*sum_yt)/det));
      }
    } else {
      // Fall 2.2: vrot!=0; berechne x0, y0, vx, vy aus Fahrt auf einer Kreisbahn
      double sum_sin=0;   // sum_i sin(model.vrot t_i)
      double sum_cos=0;   // sum_i (cos(model.vrot t_i)-1)
      double sum_sin2=0;  // sum_i (sin(model.vrot t_i))^2
      double sum_cos2=0;  // sum_i (cos(model.vrot t_i)-1)^2
      double sum_x=0;   // sum_i x_i
      double sum_y=0;   // sum_i y_i
      double sum_sinx=0;  // sum_i (sin(model.vrot t_i) x_i)
      double sum_cosx=0;  // sum_i ((cos(model.vrot t_i)-1) x_i)
      double sum_siny=0;  // sum_i (sin(model.vrot t_i) y_i)
      double sum_cosy=0;  // sum_i ((cos(model.vrot t_i)-1) y_i)
      for (unsigned int i=0; i<n; i++) {
        double t = static_cast<double>(buffer.get().timestamp.diff_msec(t_ref));
        double x = buffer.get().pos.x;
        double y = buffer.get().pos.y;
        double s = sin(vrot*t);
        double c = cos(vrot*t)-1;
        double w = exp(t*decay);
        if (abs(t)<max_delta_t) {
          sum_sin+=w*s;
          sum_cos+=w*c;
          sum_sin2+=w*s*s;
          sum_cos2+=w*c*c;
          sum_x+=w*x;
          sum_y+=w*y;
          sum_sinx+=w*s*x;
          sum_cosx+=w*c*x;
          sum_siny+=w*s*y;
          sum_cosy+=w*c*y;
        }
        buffer.step();
      }
      sum_sin/=vrot;
      sum_cos/=vrot;
      sum_sin2/=(vrot*vrot);
      sum_cos2/=(vrot*vrot);
      sum_sinx/=vrot;
      sum_cosx/=vrot;
      sum_siny/=vrot;
      sum_cosy/=vrot;
      double denom = num*(sum_sin2+sum_cos2)-sum_sin*sum_sin-sum_cos*sum_cos;
      if (denom==0) {
        pos.x = sum_x/num;
        pos.y = sum_y/num;
        vtrans = Vec::zero_vector;
      } else {
        pos.x = ((sum_sin2+sum_cos2)*sum_x-sum_sin*(sum_sinx-sum_cosy)-sum_cos*(sum_cosx+sum_siny))/denom;
        pos.y = ((sum_sin2+sum_cos2)*sum_y+sum_cos*(sum_sinx-sum_cosy)-sum_sin*(sum_cosx+sum_siny))/denom;
        vtrans = (Vec ((-sum_sin*sum_x+sum_cos*sum_y+num*(sum_sinx-sum_cosy))/denom, (-sum_cos*sum_x-sum_sin*sum_y+num*(sum_cosx+sum_siny))/denom));
      }
    }

    timestamp = t_ref;
    vrot*=1e3;   // umrechnen von rad/ms in rad/s
  }

}
