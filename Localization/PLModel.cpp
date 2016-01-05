
#include "PLModel.h"
#include <iostream>

using namespace std;
using namespace Entzerrung;

PLModel::PLModel (double cx, double cy, double sd, unsigned int n) {
  parameters.cx=cx;
  parameters.cy=cy;
  parameters.r.resize (n+1);
  parameters.h.resize (n+1);
  for (unsigned int i=0; i<=n; i++) {
    parameters.r[i]=i*sd;
    parameters.h[i]=i*sd;
  }
}

PLModel::PLModel (const PLModelParameterset& p) : parameters (p) {;}

const PLModelParameterset& PLModel::getParameters() const { return parameters; }

void PLModel::setParameters(const PLModelParameterset& p) {
  parameters=p;
}

unsigned int PLModel::numSegments () const {
  return parameters.r.size();
}

void PLModel::writeParametersToStream (std::ostream& os) {
  os << parameters.r.size() << ' ' << parameters.cx << ' ' << parameters.cy;
  for (unsigned int i=0; i<parameters.r.size(); i++)
    os << ' ' << parameters.r[i] << ' ' << parameters.h[i];
}

void PLModel::readParametersFromStream (std::istream& is) {
  unsigned int n=0;
  is >> n;
  is >> parameters.cx >> parameters.cy;
  parameters.r.clear();
  parameters.h.clear();
  parameters.r.reserve (n);
  parameters.h.reserve (n);
  for (unsigned int i=0; i<n; i++) {
    double r, h;
    is >> r >> h;
    parameters.r.push_back (r);
    parameters.h.push_back (h);
  }
}

void PLModel::mapDouble (double& x, double& y, Pos2 p) const {
  mapDouble (x,y,p,parameters);
}

Pos2 PLModel::mapInt (Pos2 p) const {
  double x, y;
  mapDouble (x,y,p,parameters);
  p.x=static_cast<int>(x+0.5);
  p.y=static_cast<int>(y+0.5);
  return p;
}

unsigned int PLModel::mapDouble (double& x, double& y, Pos2 p, const PLModelParameterset& params) {
  double dx=p.x-params.cx;
  double dy=p.y-params.cy;
  double len=sqrt(dx*dx+dy*dy);
  // (dx, dy) ist der relative Vektor vom Mittelpunkt zu p, len dessen Laenge
  int n=params.r.size();
  int i=0;
  while (i+1<n && params.r[i+1]<len)  // lazy evaluation sei Dank
    ++i;
  unsigned int res=i;
  if (i+1>=n) {
    i=n-2;
  }
  double len2=params.h[i]+(params.h[i+1]-params.h[i])/(params.r[i+1]-params.r[i])*(len-params.r[i]);
  double lenf=len2/len;
  x=params.cx+dx*lenf;
  y=params.cy+dy*lenf;
  return res;
}

void PLModel::eraseRing (unsigned int k) {
  if (k==0) {
    parameters.r.erase (parameters.r.begin()+1);
    parameters.h.erase (parameters.h.begin()+1);
  } else if (k+1==parameters.r.size()) {
    parameters.r.erase (parameters.r.end()-1);
    parameters.h.erase (parameters.h.end()-1);
  } else {
    parameters.r[k]=0.5*(parameters.r[k]+parameters.r[k+1]);
    parameters.h[k]=0.5*(parameters.h[k]+parameters.h[k+1]);
    parameters.r.erase (parameters.r.begin()+k+1);
    parameters.h.erase (parameters.h.begin()+k+1);  
  }
}

void PLModel::splitRing (unsigned int k) {
  if (k+1>=parameters.r.size()) {
    unsigned int n=parameters.r.size();
    parameters.r.push_back (2*parameters.r[n-1]-parameters.r[n-2]);
    parameters.h.push_back (2*parameters.h[n-1]-parameters.h[n-2]);      
  
  } else {
    parameters.r.insert (parameters.r.begin()+k+1, 0.5*(parameters.r[k]+parameters.r[k+1]));
    parameters.h.insert (parameters.h.begin()+k+1, 0.5*(parameters.h[k]+parameters.h[k+1]));
  }
}

void PLModel::invert () {
  vector<double> sw = parameters.r;
  parameters.r=parameters.h;
  parameters.h=sw;
}

