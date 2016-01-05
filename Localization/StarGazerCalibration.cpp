
#include "StarGazerCalibration.h"
#include <sstream>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace DerWeg;

void RobotPositionList::readPositions (const ConfigReader& cfg) throw (std::invalid_argument) {
  clear();
  vector<string> ids;
  if (!cfg.get ("StarGazer::robot_positions", ids))
    throw invalid_argument ("readPositions: key StarGazer::robot_positions not found");
  for (unsigned int i=0; i<ids.size(); i++) {
    vector<string> params;
    if (!cfg.get ((string("StarGazer::P")+ids[i]).c_str(), params))
      throw invalid_argument (string("readPositions: key P")+ids[i]+string(" not found"));
    if (params.size()<4)
      throw invalid_argument (string("readPositions: key P")+ids[i]+string(" incomplete"));
    StarGazerRobotPosition rpos;
    stringstream inout;
    inout << params[0] << ' ' << params[1] << ' ' << params[2] << ' ' << params[3] << std::flush;
    double aa;
    inout >> rpos.id >> rpos.position.x >> rpos.position.y >> aa;
    rpos.orientation.set_deg(aa);
    rpos.fixed=(params.size()>=5 && params[4]=="*");
    push_back (rpos);
  }
}

void RobotPositionList::writePositions (std::ostream& os) const {
  os << "[StarGazer]\n";
  os << "robot_positions =";
  for (unsigned int i=0; i<size(); i++) {
    os << ' ' << (*this)[i].id;
  }
  os << '\n';
  os << "# Syntax: Pid = position_id position.x position.y orientation(deg) fixed(*)\n";
  for (unsigned int i=0; i<size(); i++) {
    os << "P" << (*this)[i].id << " = " << (*this)[i].id << ' ' << (*this)[i].position.x << ' ' << (*this)[i].position.y << ' ' << (*this)[i].orientation.get_deg();
    if ((*this)[i].fixed) {
      os << " *";
    }
    os << '\n';
  }
}

void MeasurementList::readMeasurements (const ConfigReader& cfg) throw (std::invalid_argument) {
  unsigned int sz;
  clear();
  if (!cfg.get ("StarGazer::measurements", sz))
    throw invalid_argument ("readMeasurements: key StarGazer::measurements not found");
  for (unsigned int i=0; i<sz; i++) {
    stringstream ss;
    ss << "StarGazer::M" << i;
    vector<double> params;
    if (!cfg.get (ss.str().c_str(), params))
      throw invalid_argument (string("readMeasurements: key ")+ss.str()+string(" not found"));
    if (params.size()<5)
      throw invalid_argument (string("readMeasurements: key ")+ss.str()+string(" incomplete"));
    StarGazerCalibrationMeasurement mm;
    mm.landmark_id = static_cast<unsigned int>(params[0]);
    mm.position_id = static_cast<unsigned int>(params[1]);
    mm.position.x = params[2];
    mm.position.y = params[3];
    mm.orientation = Angle::deg_angle (params[4]);
    push_back (mm);
  }
}

void MeasurementList::writeMeasurements (std::ostream& os) const {
  os << "[StarGazer]\n";
  os << "measurements = " << size() << '\n';
  os << "# Syntax: Mid = landmark_id position_id position.x position.y orientation(deg)\n";
  for (unsigned int i=0; i<size(); i++)
    os << "M" << i << " = " << (*this)[i].landmark_id << ' ' << (*this)[i].position_id << ' ' << (*this)[i].position.x << ' ' << (*this)[i].position.y << ' ' << (*this)[i].orientation.get_deg() << '\n';
}


const LandmarkList& StarGazerCalibration::getLandmarks() const { return landmarks; }
const RobotPositionList& StarGazerCalibration::getPositions() const { return positions; }
const MeasurementList& StarGazerCalibration::getMeasurements() const { return measurements; }


TensionTriple::TensionTriple () : landmark_id(0), position_id(0), error(0), length(0), std(0), num(0), fixed(false) {;}

TensionTriple::TensionTriple (const TensionTriple& tt) { operator= (tt); }

const TensionTriple& TensionTriple::operator= (const TensionTriple& tt) {
  landmark_id = tt.landmark_id;
  position_id = tt.position_id;
  error = tt.error;
  length = tt.length;
  num = tt.num;
  fixed = tt.fixed;
  std = tt.std;
  return *this;
}

bool TensionTriple::operator< (const TensionTriple& tt) const {
  return error<tt.error;
}


void StarGazerCalibration::addAndMerge (const LandmarkList& landmarks1, const RobotPositionList& positions1, const MeasurementList& measurements1) {
  for (unsigned int il1=0; il1<landmarks1.size(); ++il1) {
    if (lidmap.find (landmarks1[il1].id)==lidmap.end()) {
      landmarks.push_back (landmarks1[il1]);
      lidmap[landmarks1[il1].id]=landmarks.size()-1;
    }
  }

  unsigned int pidoffset=positions.size();
  for (unsigned int ip1=0; ip1<positions1.size(); ++ip1) {
    StarGazerRobotPosition pos=positions1[ip1];
    pos.id+=pidoffset;
    positions.push_back (pos);
    pidmap[pos.id]=positions.size()-1;
  }

  for (unsigned int im1=0; im1<measurements1.size(); ++im1) {
    StarGazerCalibrationMeasurement m = measurements1[im1];
    m.position_id+=pidoffset;
    measurements.push_back (m);
  }

  numLandmarks = landmarks.size();
  numPositions = positions.size();
  numMeasurements = measurements.size();
}

void StarGazerCalibration::determinePositions () {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;

  vector<Vec> directionsPositions (numPositions);
  for (unsigned int im=0; im<numMeasurements; im++) {
    unsigned int lid = lidmap.find (measurements[im].landmark_id)->second;
    unsigned int pid = pidmap.find (measurements[im].position_id)->second;
    directionsPositions[pid]+=Vec::unit_vector (landmarks[lid].orientation+measurements[im].orientation);
  }
  for (unsigned int ip=1; ip<numPositions; ip++) {
    if (directionsPositions[ip].squared_length()>0.01) {
      positions[ip].orientation=directionsPositions[ip].angle();
    }
  }

  vector<Vec> positionsPositions (numPositions);
  vector<unsigned int> positionsNumPMeasurements (numPositions,0);
  for (unsigned int im=0; im<numMeasurements; im++) {
    unsigned int lid = lidmap[measurements[im].landmark_id];
    unsigned int pid = pidmap[measurements[im].position_id];
    positionsPositions[pid]+=landmarks[lid].position+measurements[im].position.rotate(landmarks[lid].orientation);
    positionsNumPMeasurements[pid]++;
  }
  for (unsigned int ip=1; ip<numPositions; ip++) {
    if (positionsNumPMeasurements[ip]>0) {
      positions[ip].position=positionsPositions[ip]/positionsNumPMeasurements[ip];
    }
  }
}

void StarGazerCalibration::calibrateLandmarks () {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;
  bool nofixed=true;
  for (unsigned ip=0; ip<numPositions; ++ip) {
    nofixed = nofixed && !positions[ip].fixed;
  }
  for (unsigned il=0; il<numLandmarks; ++il) {
    nofixed = nofixed && !landmarks[il].fixed;
  }

  vector<bool> landmarksUsed (numLandmarks, false);
  vector<bool> positionsUsed (numPositions, false);
  for (unsigned int il=0; il<numLandmarks; il++) {
    landmarksUsed[il]=landmarks[il].fixed;
  }
  for (unsigned ip=0; ip<numPositions; ++ip) {
    positionsUsed[ip]=positions[ip].fixed;  // indicating that fixed positions are used as reference position
  }
  if (nofixed) {
    positionsUsed[0]=true;  // if no position nor landmark is fixed, specify remaining degree of freedom fixing position 0
  }

  // first, estimate the orientation of landmarks and positions iteratively
  double change=1e100;
  unsigned int iter=0;
  while (change/numLandmarks>0.01 && iter++<1000) {
    change=0; // measures the sum of changes in landmark orientation (in deg)
    // calculate average direction of landmarks:
    vector<Vec> directionsLandmarks (numLandmarks);
    for (unsigned int im=0; im<numMeasurements; im++) {
      unsigned int lid = lidmap[measurements[im].landmark_id];
      unsigned int pid = pidmap[measurements[im].position_id];
      if (positionsUsed[pid]) {
        directionsLandmarks[lid]+=Vec::unit_vector (positions[pid].orientation-measurements[im].orientation);
      }
    }
    for (unsigned int il=0; il<numLandmarks; il++) {
      if (!landmarks[il].fixed) {
        if (directionsLandmarks[il].squared_length()>0.01) {
          landmarksUsed[il]=true;
          change+=abs((landmarks[il].orientation-directionsLandmarks[il].angle()).get_deg_180());
          landmarks[il].orientation=directionsLandmarks[il].angle();
        } else {
          change+=1e3;
        }
      }
    }

    // calculate the average direction of positions:
    vector<Vec> directionsPositions (numPositions);
    for (unsigned int im=0; im<numMeasurements; im++) {
      unsigned int lid = lidmap.find (measurements[im].landmark_id)->second;
      unsigned int pid = pidmap.find (measurements[im].position_id)->second;
      if (landmarksUsed[lid]) {
        directionsPositions[pid]+=Vec::unit_vector (landmarks[lid].orientation+measurements[im].orientation);
      }
    }
    for (unsigned int ip=0; ip<numPositions; ip++) {
      if (!positions[ip].fixed && (ip!=0 || !nofixed)) {
        if (directionsPositions[ip].squared_length()>0.01) {
          positionsUsed[ip]=true;
          positions[ip].orientation=directionsPositions[ip].angle();
        }
      }
    }
  }

  // second, estimate the position of landmarks and positions iteratively
  change=1e100;
  for (unsigned int il=0; il<numLandmarks; il++) {
    landmarksUsed[il]=landmarks[il].fixed;
  }
  for (unsigned int ip=0; ip<numPositions; ip++) {
    positionsUsed[ip]=positions[ip].fixed;
  }
  if (nofixed) {
    positionsUsed[0]=true;  // if no position nor landmark is fixed, specify remaining degree of freedom fixing position 0
  }
  iter=0;
  while (change/numLandmarks>0.01 && iter++<1000) {
    change=0; // measures the sum of changes in landmark positions
    // calculate average position of landmarks:
    vector<Vec> positionsLandmarks (numLandmarks);
    vector<unsigned int> positionsNumLMeasurements (numLandmarks,0);
    for (unsigned int im=0; im<numMeasurements; im++) {
      unsigned int lid = lidmap[measurements[im].landmark_id];
      unsigned int pid = pidmap[measurements[im].position_id];
      if (positionsUsed[pid]) {
        positionsLandmarks[lid]+=positions[pid].position-measurements[im].position.rotate(landmarks[lid].orientation);
        positionsNumLMeasurements[lid]++;
      }
    }
    for (unsigned int il=0; il<numLandmarks; il++) {
      if (!landmarks[il].fixed) {
        if (positionsNumLMeasurements[il]>0) {
          landmarksUsed[il]=true;
          positionsLandmarks[il]/=positionsNumLMeasurements[il];
          change+=(landmarks[il].position-positionsLandmarks[il]).length();
          landmarks[il].position=positionsLandmarks[il];
        } else {
          change+=1e3;
        }
      }
    }

    // calculate the average position of positions:
    vector<Vec> positionsPositions (numPositions);
    vector<unsigned int> positionsNumPMeasurements (numPositions,0);
    for (unsigned int im=0; im<numMeasurements; im++) {
      unsigned int lid = lidmap[measurements[im].landmark_id];
      unsigned int pid = pidmap[measurements[im].position_id];
      if (landmarksUsed[lid]) {
        positionsPositions[pid]+=landmarks[lid].position+measurements[im].position.rotate(landmarks[lid].orientation);
        positionsNumPMeasurements[pid]++;
      }
    }
    for (unsigned int ip=0; ip<numPositions; ip++) {
      if (!positions[ip].fixed && (ip!=0 || !nofixed)) {
        if (positionsNumPMeasurements[ip]>0) {
          positionsUsed[ip]=true;
          positions[ip].position=positionsPositions[ip]/positionsNumPMeasurements[ip];
        }
      }
    }
  }
}


void StarGazerCalibration::calculateAverageAngleDeviation (unsigned int& num, Angle& avg, double& std, const MeasurementList& m) {
  num=m.size();
  Vec ss = Vec(0,0);
  vector<Vec> unitvecs (m.size());
  for (unsigned int im=0; im<m.size(); ++im) {
    Vec uv = Vec::unit_vector (m[im].orientation);
    ss += uv;
    unitvecs[im]=uv;
  }
  if (num==0) return;
  double sum2=0;
  for (unsigned int im=0; im<m.size(); ++im) {
    double a = ss.angle(unitvecs[im]).get_deg_180();
    sum2+=a*a;
  }
  avg = ss.angle();
  std = sqrt(sum2/num);
}

void StarGazerCalibration::calculateAverageDeviation (unsigned int& num, Vec& avg, Vec& std, const MeasurementList& m) {
  num = 0;
  avg = Vec(0,0);
  Vec sum2 (0,0);
  double sumCov = 0;
  for (unsigned int im=0; im<m.size(); ++im) {
    num++;
    avg += m[im].position;
    sum2 += Vec(m[im].position.x*m[im].position.x,m[im].position.y*m[im].position.y);
    sumCov += m[im].position.x*m[im].position.y;
  }
  if (num==0) return;
  avg/=num;
  double xx = sum2.x/num-avg.x*avg.x;
  double yy = sum2.y/num-avg.y*avg.y;
  double xy = sumCov/num-avg.x*avg.y;
  double rt = sqrt((xx+yy)*(xx+yy)/4-xx*yy+xy*xy);
  double s1 = sqrt((xx+yy)/2+rt);
  double s2 = (rt<(xx+yy)/2 ? sqrt((xx+yy)/2-rt) : 0);
  std = Vec (s1, s2);
}

void StarGazerCalibration::removeLargeDeviation (double maxstd) {
  MeasurementList newlist;
  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      if (std.x<=maxstd) {
        newlist.insert (newlist.end(), m.begin(), m.end());
      }
    }
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

void StarGazerCalibration::removeLargeDistances (double maxdist) {
  MeasurementList newlist;
  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      if (avg.length()<=maxdist) {
        newlist.insert (newlist.end(), m.begin(), m.end());
      }
    }
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

std::vector<TensionTriple> StarGazerCalibration::getTensions () {
  std::vector<TensionTriple> result;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        Vec pexpect (0,0);
        for (unsigned int im=0; im<m.size(); ++im) {
          pexpect += landmarks[il].position+m[im].position.rotate(landmarks[il].orientation);
        }
        pexpect /= m.size();
        TensionTriple tt;
        tt.landmark_id = landmarks[il].id;
        tt.position_id = positions[ip].id;
        tt.error = (pexpect-positions[ip].position).length();
        tt.fixed = positions[ip].fixed || landmarks[il].fixed;
        Vec v1, v2;
        calculateAverageDeviation (tt.num, v1, v2, m);
        tt.length = v1.length();
        tt.std = v2.x;
        result.push_back (tt);
      }
    }
  }
  return result;
}

void StarGazerCalibration::removeMostTensioningMeasurements (bool ignore_fixed, std::ostream& os) {
  std::vector<TensionTriple> tensions = getTensions();
  std::sort (tensions.begin(), tensions.end());
  std::vector<TensionTriple>::reverse_iterator it = tensions.rbegin();
  while (it!=tensions.rend()) {
    if (!it->fixed || !ignore_fixed) {
      os << "removing relationship L" << it->landmark_id << " - P" << it->position_id << " with " << it->num << " measurements and error " << it->error << std::endl;
      MeasurementList newlist;
      newlist.reserve (measurements.size());
      for (unsigned int im=0; im<measurements.size(); ++im) {
        if (measurements[im].landmark_id!=it->landmark_id || measurements[im].position_id!=it->position_id) {
          newlist.push_back (measurements[im]);
        }
      }
      measurements=newlist;
      numMeasurements=measurements.size();
      return;
    }
    ++it;
  }
}

std::vector<unsigned int> StarGazerCalibration::connectedLandmarks() const {
  std::vector<unsigned int> num_landmarks_per_position (numPositions,0);  
  for (unsigned int il=0; il<landmarks.size(); ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) ++(num_landmarks_per_position[ip]);
    }
  }
  return num_landmarks_per_position;
}

void StarGazerCalibration::removeIrrelevantMeasurements (std::ostream& os) {
  std::vector<unsigned int> num_landmarks_per_position = connectedLandmarks ();
  MeasurementList newlist;
  newlist.reserve (measurements.size());
  for (unsigned int ip=0; ip<numPositions; ++ip) {
    for (unsigned int il=0; il<numLandmarks; ++il) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        if (num_landmarks_per_position[ip]>=2 || positions[ip].fixed) {
          newlist.insert(newlist.end(), m.begin(), m.end());
        } else {
          os << "removing relationship L" << landmarks[il].id << " - P" << positions[ip].id << "\n";
        }
      }
    }
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

void StarGazerCalibration::printStatisticsLandmarks (std::ostream& os) const {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;

  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  Angle aa;
  double aastd=0;
  for (unsigned int il=0; il<landmarks.size(); ++il) {
    os << "L" << landmarks[il].id << ":\n";
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      calculateAverageAngleDeviation (num, aa, aastd, m);
      if (num>0) {
        os << "  P" << positions[ip].id << ": n=" << num;
        os << "; avg_len=" << avg.length();
        os << "; max_std=" << std.x;
        os << "; avg_angle=" << aa.get_deg();
        os << "; std_angle=" << aastd << endl;
      }
    }
  }
}

void StarGazerCalibration::printStatisticsPositions (std::ostream& os) const {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;

  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  Angle aa;
  double aastd=0;
  for (unsigned int ip=0; ip<numPositions; ++ip) {
    bool first = true;
    for (unsigned int il=0; il<landmarks.size(); ++il) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      calculateAverageAngleDeviation (num, aa, aastd, m);
      if (num>0) {
        if (first) {
          os << "P" << positions[ip].id;
          if (positions[ip].fixed)
            os << "(*)";
          os << ":\n";
          first = false;
        }
        os << "  L" << landmarks[il].id << ": n=" << num;
        os << "; avg_len=" << avg.length();
        os << "; max_std=" << std.x;
        os << "; avg_angle=" << aa.get_deg();
        os << "; std_angle=" << aastd << endl;
      }
    }
  }
}

void StarGazerCalibration::extractMeasurements (MeasurementList& dest, unsigned int lid, unsigned int pid) const {
  dest.clear();
  for (unsigned int im=0; im<numMeasurements; ++im) {
    if (measurements[im].landmark_id==lid && measurements[im].position_id==pid) {
      dest.push_back (measurements[im]);
    }
  }
}

void StarGazerCalibration::removeRelationship (unsigned int lid, unsigned int pid) {
  MeasurementList newlist;
  for (unsigned int im=0; im<numMeasurements; ++im) {
    if (measurements[im].landmark_id==lid && measurements[im].position_id==pid)
      continue;
    newlist.push_back (measurements[im]);
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

void StarGazerCalibration::removeWeakRelationships (unsigned int nmin) {
  MeasurementList newlist;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>=nmin) {
        newlist.insert (newlist.end(), m.begin(), m.end());
      }
    }
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

namespace {

  class AdjacencyMatrix {
    vector<bool> m;
    unsigned int n;
  public:
    AdjacencyMatrix (unsigned int n1) : m(n1*n1,false), n(n1) {;}
    void setConnection (unsigned int i, unsigned int j) { m[i*n+j]=m[j*n+i]=true; }
    bool getConnection (unsigned int i, unsigned int j) { return m[i*n+j]; }
    void reflexiveHull () { for (unsigned int i=0; i<n; ++i) setConnection (i,i); }
    void transitiveHull () {  // (Floyed Warshal)
      bool change=true;
      for (unsigned int iter=0; iter<n && change; ++iter) {
        bool change=false;
        for (unsigned int i=0; i<n; ++i) {
          for (unsigned int j=0; j<i; ++j) {
            if (!getConnection (i,j)) {
              for (unsigned int k=0; k<n; ++k) {
                if (getConnection(i,k) && getConnection(k,j)) {
                  setConnection(i,j);
                  change=true;
                }
              }
            }
          }
        }
      }
    }
    vector<vector<unsigned int> > connectedComponents () {
      vector<bool> init (n,true);
      vector<vector<unsigned int> > dest;
      for (unsigned int i=0; i<n; ++i) {
        if (init[i]) {
          vector<unsigned int> elems;
          for (unsigned int j=0; j<n; ++j) {
            if (getConnection (i,j)) {
              init[j]=false;
              elems.push_back (j);
            }
          }
          dest.push_back (elems);
        }
      }
      return dest;
    }
  };
}

std::vector<std::vector<std::string> > StarGazerCalibration::connectedComponents() const {
  AdjacencyMatrix admat (numLandmarks+numPositions);
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0)
        admat.setConnection (il, numLandmarks+ip);
    }
  }
  admat.transitiveHull ();
  admat.reflexiveHull ();
  vector<vector<unsigned int> > comp = admat.connectedComponents ();
  vector<vector<string> > comps (comp.size());
  for (unsigned int i=0; i<comp.size(); ++i) {
    for (unsigned int j=0; j<comp[i].size(); ++j) {
      stringstream inout;
      inout << (comp[i][j]<numLandmarks ? 'L' : 'P') << (comp[i][j]<numLandmarks ? landmarks[comp[i][j]].id : comp[i][j]-numLandmarks);
      comps[i].push_back (inout.str());
    }
  }
  return comps;
}

double StarGazerCalibration::totalAngleError (double exponent) const {
  double sum=0;
  unsigned int n=0;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        Vec pexpect (0,0);
        for (unsigned int im=0; im<m.size(); ++im) {
          pexpect += Vec::unit_vector (landmarks[il].orientation+m[im].orientation);
        }
        double e = abs(pexpect.angle().get_deg()-positions[ip].orientation.get_deg());
        while (e>180) e-=360;
        e=abs(e);
        sum+=pow (e, exponent);
        n++;
      }
    }
  }
  return (n>0 ? sum/n : 0);
}

double StarGazerCalibration::totalPositionError (double exponent) const {
  double sum=0;
  unsigned int n=0;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        Vec pexpect (0,0);
        for (unsigned int im=0; im<m.size(); ++im) {
          pexpect += landmarks[il].position+m[im].position.rotate(landmarks[il].orientation);
        }
        pexpect/=m.size();
        sum+=pow ((pexpect-positions[ip].position).length(), exponent);
        n++;
      }
    }
  }
  return (n>0 ? sum/n : 0);
}

namespace {
  struct MDS {
    StarGazerCalibrationMeasurement measurement;
    double error;
    bool select;

    bool operator< (const MDS& mds) const { return error<mds.error; }
  };
  class MDSList : public vector<MDS> {
  public:
    void create (const MeasurementList& ml) {
      resize(ml.size());
      for (unsigned int i=0; i<size(); ++i) {
        operator[](i).measurement=ml[i];
        operator[](i).select=false;
        operator[](i).error=0;
      }
    }
    void appendSelected (MeasurementList& ml) {
      for (unsigned int i=0; i<size(); ++i) {
        if (operator[](i).select) {
          ml.push_back (operator[](i).measurement);
        }
      }
    }
    void calcError (Vec target) {
      for (unsigned int i=0; i<size(); ++i) {
        operator[](i).error = (operator[](i).measurement.position-target).squared_length();
      }
    }
    Vec calcAverageSelected () {
      unsigned int n=0;
      Vec sum (0,0);
      for (unsigned int i=0; i<size(); ++i) {
        if (operator[](i).select) {
          n++;
          sum+=operator[](i).measurement.position;
        }
      }
      return sum/n;
    }
    bool selectBest (double& error, unsigned int nsel) { // return true is selection was changed
      if (nsel>size())
        nsel=size();
      sort (begin(), end());
      error=0;
      bool selchanged=false;
      for (unsigned int i=0; i<nsel; ++i) {
        if (!operator[](i).select) {
          operator[](i).select=true;
          selchanged=true;
        }
        error+=operator[](i).error;
      }
      for (unsigned int i=nsel; i<size(); ++i) {
        if (operator[](i).select) {
          operator[](i).select=false;
          selchanged=true;
        }
      }
      return selchanged;
    }
  };
}


unsigned int randomselect (unsigned int nmax) {
  return static_cast<unsigned int>(floor(static_cast<double>(rand())/(static_cast<double>(RAND_MAX)+1)*nmax));
}

void StarGazerCalibration::trimMeasurements (double acceptRate) {
  MeasurementList newlist;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        MDSList best_mds;
        double best_error=1e300;
        for (unsigned int trial=0; trial<20; ++trial) {
          MDSList mds;
          mds.create (m);
          unsigned int irand = randomselect (mds.size());
          mds[irand].select=true;
          Vec avg=mds[irand].measurement.position;
          bool docont=true;
          double error=1e300;
          unsigned int iter=0;
          while (docont && (iter++<100)) {
            avg = mds.calcAverageSelected ();
            mds.calcError (avg);
            docont = mds.selectBest (error, static_cast<unsigned int>(mds.size()*acceptRate));
          }
          if (error<best_error) {
            best_error=error;
            best_mds=mds;
          }
        }
        best_mds.appendSelected (newlist);
      }
    }
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

