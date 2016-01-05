
#include "CameraLocalizationMapping.h"
#include <sstream>
#include <set>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>

using namespace std;
using namespace DerWeg;

int LandmarkList::findLandmark (unsigned int lid) const {
  for (unsigned int i=0; i<size(); i++) {
    if ((*this)[i].id==lid) {
      return i;
    }
  }
  return -1;
}

void LandmarkList::readLandmarks (const ConfigReader& cfg) throw (invalid_argument) {
  (*this).clear();
  vector<string> ids;
  if (!cfg.get ("CameraLocalization::landmarks", ids)) {
    throw invalid_argument ("readLandmarks: key CameraLocalization::landmarks not found");
  }
  for (unsigned int i=0; i<ids.size(); i++) {
    vector<string> params;
    if (!cfg.get ((string("CameraLocalization::L")+ids[i]).c_str(), params)) {
      throw invalid_argument (string("readLandmarks: key L")+ids[i]+string(" not found"));
    }
    if (params.size()<4) {
      throw invalid_argument (string("readLandmarks: key L")+ids[i]+string(" incomplete"));
    }
    Landmark lm;
    stringstream inout;
    inout << params[0] << ' ' << params[1] << ' ' << params[2] << ' ' << params[3] << std::flush;
    inout >> lm.id >> lm.height >> lm.position.x >> lm.position.y;
    lm.fixed=(params.size()>=6 && params[5]=="*");
    (*this).push_back (lm);
  }
}

void LandmarkList::writeLandmarks (ostream& os) const {
  os << "[CameraLocalization]\n";
  os << "# parameters for landmarks\n";
  os << "# automatically created\n";
  os << "landmarks =";
  for (unsigned int i=0; i<(*this).size(); i++) {
    os << ' ' << (*this)[i].id;
  }
  os << '\n';
  os << "# Syntax: Lid = landmark_id height position.x position.y fixed(*)\n";
  for (unsigned int i=0; i<(*this).size(); i++) {
    os << 'L' << (*this)[i].id << " = " << (*this)[i].id << ' ' << (*this)[i].height << ' ' << (*this)[i].position.x << ' ' << (*this)[i].position.y;
    if ((*this)[i].fixed) {
      os << " *";
    }
    os << '\n';
  }
}


void RobotPositionList::readPositions (const ConfigReader& cfg) throw (std::invalid_argument) {
  clear();
  vector<string> ids;
  if (!cfg.get ("CameraLocalization::robot_positions", ids))
    throw invalid_argument ("readPositions: key CameraLocalization::robot_positions not found");
  for (unsigned int i=0; i<ids.size(); i++) {
    vector<string> params;
    if (!cfg.get ((string("CameraLocalization::P")+ids[i]).c_str(), params))
      throw invalid_argument (string("readPositions: key P")+ids[i]+string(" not found"));
    if (params.size()<4)
      throw invalid_argument (string("readPositions: key P")+ids[i]+string(" incomplete"));
    RobotPosition rpos;
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
  os << "[CameraLocalization]\n";
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
  if (!cfg.get ("CameraLocalization::measurements", sz))
    throw invalid_argument ("readMeasurements: key CameraLocalization::measurements not found");
  for (unsigned int i=0; i<sz; i++) {
    stringstream ss;
    ss << "CameraLocalization::M" << i;
    vector<double> params;
    if (!cfg.get (ss.str().c_str(), params))
      throw invalid_argument (string("readMeasurements: key ")+ss.str()+string(" not found"));
    if (params.size()<4)
      throw invalid_argument (string("readMeasurements: key ")+ss.str()+string(" incomplete"));
    CameraLocalizationMeasurement mm;
    mm.landmark_id = static_cast<unsigned int>(params[0]);
    mm.position_id = static_cast<unsigned int>(params[1]);
    mm.position.x = params[2];
    mm.position.y = params[3];
    push_back (mm);
  }
}

void MeasurementList::writeMeasurements (std::ostream& os) const {
  os << "[CameraLocalization]\n";
  os << "measurements = " << size() << '\n';
  os << "# Syntax: Mid = landmark_id position_id position.x position.y\n";
  for (unsigned int i=0; i<size(); i++)
    os << "M" << i << " = " << (*this)[i].landmark_id << ' ' << (*this)[i].position_id << ' ' << (*this)[i].position.x << ' ' << (*this)[i].position.y << '\n';
}


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


const LandmarkList& CameraLocalizationMapping::getLandmarks() const { return landmarks; }
const RobotPositionList& CameraLocalizationMapping::getPositions() const { return positions; }
const MeasurementList& CameraLocalizationMapping::getMeasurements() const { return measurements; }


void CameraLocalizationMapping::addAndMerge (const LandmarkList& landmarks1, const RobotPositionList& positions1, const MeasurementList& measurements1) {
  for (unsigned int il1=0; il1<landmarks1.size(); ++il1) {
    if (lidmap.find (landmarks1[il1].id)==lidmap.end()) {
      landmarks.push_back (landmarks1[il1]);
      lidmap[landmarks1[il1].id]=landmarks.size()-1;
    }
  }

  unsigned int pidoffset=positions.size();
  for (unsigned int ip1=0; ip1<positions1.size(); ++ip1) {
    RobotPosition pos=positions1[ip1];
    pos.id+=pidoffset;
    positions.push_back (pos);
    pidmap[pos.id]=positions.size()-1;
  }

  for (unsigned int im1=0; im1<measurements1.size(); ++im1) {
    CameraLocalizationMeasurement m = measurements1[im1];
    m.position_id+=pidoffset;
    measurements.push_back (m);
  }

  numLandmarks = landmarks.size();
  numPositions = positions.size();
  numMeasurements = measurements.size();
}

double CameraLocalizationMapping::determinePositions (bool keep_fixed) {
  std::vector<bool> landmarks_used (numLandmarks, true);
  std::vector<bool> positions_used (numPositions, true);
  return determinePositions (keep_fixed, positions_used, landmarks_used);
}


double CameraLocalizationMapping::determinePositions (bool keep_fixed, std::vector<bool>& positions_used, const std::vector<bool>& landmarks_used) {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return 0;
  
  double change = 0;

  vector<Vec> sum_l (numPositions, Vec::zero_vector);
  vector<Vec> sum_v (numPositions, Vec::zero_vector);
  vector<Vec> sum_mx (numPositions, Vec::zero_vector);
  vector<Vec> sum_my (numPositions, Vec::zero_vector);
  vector<unsigned int> num (numPositions, 0);
  vector<set<unsigned int> > connected_landmarks (numPositions);
  for (unsigned int im=0; im<numMeasurements; im++) {
    unsigned int lid = lidmap.find (measurements[im].landmark_id)->second;
    unsigned int pid = pidmap.find (measurements[im].position_id)->second;
    if (landmarks_used[lid]) {
      connected_landmarks[pid].insert (lid);
      num [pid] += 1;
      sum_l [pid] += landmarks[lid].position;
      sum_v [pid] += measurements[im].position;
      sum_mx [pid].x += landmarks[lid].position.x*measurements[im].position.x;
      sum_mx [pid].y += landmarks[lid].position.x*measurements[im].position.y;
      sum_my [pid].x += landmarks[lid].position.y*measurements[im].position.x;
      sum_my [pid].y += landmarks[lid].position.y*measurements[im].position.y;
    }
  }
  for (unsigned int ip=0; ip<numPositions; ip++) {
    if ((!keep_fixed || !positions[ip].fixed) && (connected_landmarks[ip].size()>=2)) {
      Vec p_old = positions[ip].position;
      Vec ab (sum_mx [ip].y-sum_my [ip].x-sum_l [ip].x*sum_v [ip].y/num [ip]+sum_l [ip].y*sum_v [ip].x/num [ip],
              sum_mx [ip].x+sum_my [ip].y-sum_l [ip].x*sum_v [ip].x/num [ip]-sum_l [ip].y*sum_v [ip].y/num [ip]);
      positions[ip].orientation = ab.angle()-Angle::quarter;
      positions[ip].position = (sum_l [ip]-sum_v [ip].rotate (positions[ip].orientation))/num [ip];
      positions_used[ip] = true;
      change += (p_old-positions[ip].position).length();
    }
  }
  return change;
}

double CameraLocalizationMapping::determineLandmarks () {
  std::vector<bool> landmarks_used (numLandmarks, true);
  std::vector<bool> positions_used (numPositions, true);
  return determineLandmarks (landmarks_used, positions_used);
}

double CameraLocalizationMapping::determineLandmarks (std::vector<bool>& landmarks_used, const std::vector<bool>& positions_used) {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return 0;
  
  double change = 0;
  
  vector<Vec> sum_l (numLandmarks, Vec::zero_vector);
  vector<unsigned int> num (numLandmarks, 0);
  for (unsigned int im=0; im<numMeasurements; im++) {
    unsigned int lid = lidmap.find (measurements[im].landmark_id)->second;
    unsigned int pid = pidmap.find (measurements[im].position_id)->second;
    if (positions_used[pid]) {
      num [lid] += 1;
      sum_l [lid] += positions[pid].position + measurements[im].position.rotate (positions[pid].orientation);
    }
  }
  for (unsigned int il=0; il<numLandmarks; il++) {
    if (num[il]>=1) {
      Vec l_old = landmarks[il].position;
      landmarks[il].position = sum_l [il]/num [il];
      change += (l_old-landmarks[il].position).length();
      landmarks_used[il] = true;
    }
  }
  return change;
}


void CameraLocalizationMapping::calibrateLandmarks (unsigned int nmax) {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;
  
  std::vector<bool> landmarks_used (numLandmarks, false);
  std::vector<bool> positions_used (numPositions, false);
  for (unsigned int il=0; il<numLandmarks; ++il) {
    landmarks_used[il]=landmarks[il].fixed;
  }
  for (unsigned int ip=0; ip<numPositions; ++ip) {
    positions_used[ip]=positions[ip].fixed;
  }
  
  unsigned int i=0;
  double change = 0;
  do {
    change = 0;
    double change1 = determineLandmarks (landmarks_used, positions_used);
    double change2 = determinePositions (true, positions_used, landmarks_used);
    change = change1+change2;
    ++i;
  } while ((change>0.1 || i<numLandmarks) && (nmax==0 || i<nmax));
}


void CameraLocalizationMapping::calculateAverageDeviation (unsigned int& num, Vec& avg, Vec& std, const MeasurementList& m) {
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
  double rt = sqrt(std::max(0.0,(xx+yy)*(xx+yy)/4-xx*yy+xy*xy));
  double s1 = sqrt(std::max(0.0,(xx+yy)/2+rt));
  double s2 = (rt<(xx+yy)/2 ? sqrt((xx+yy)/2-rt) : 0);
  std = Vec (s1, s2);
}

void CameraLocalizationMapping::removeLargeDeviation (double maxstd) {
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

void CameraLocalizationMapping::removeLargeDistances (double maxdist) {
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

std::vector<TensionTriple> CameraLocalizationMapping::getTensions () {
  std::vector<TensionTriple> result;
  for (unsigned int il=0; il<numLandmarks; ++il) {
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        Vec pexpect (0,0);
        for (unsigned int im=0; im<m.size(); ++im) {
          pexpect += positions[ip].position+m[im].position.rotate (positions[ip].orientation);
        }
        pexpect /= m.size();
        TensionTriple tt;
        tt.landmark_id = landmarks[il].id;
        tt.position_id = positions[ip].id;
        tt.error = (pexpect-landmarks[il].position).length();
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

void CameraLocalizationMapping::removeMostTensioningMeasurements (bool ignore_fixed, std::ostream& os) {
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

std::vector<unsigned int> CameraLocalizationMapping::connectedLandmarks() const {
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

void CameraLocalizationMapping::removeIrrelevantMeasurements (std::ostream& os) {
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

void CameraLocalizationMapping::printStatisticsLandmarks (std::ostream& os) const {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;

  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  for (unsigned int il=0; il<landmarks.size(); ++il) {
    os << "L" << landmarks[il].id << ":\n";
    for (unsigned int ip=0; ip<numPositions; ++ip) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      double map_error = (positions[ip].position+avg.rotate(positions[ip].orientation)-landmarks[il].position).length();
      if (num>0) {
        os << "  P" << positions[ip].id << ": n=" << num;
        os << "; avg_len=" << avg.length();
        os << "; max_std=" << std.x;
        os << "; map_error=" << map_error;
        os << endl;
      }
    }
  }
}

void CameraLocalizationMapping::printStatisticsPositions (std::ostream& os) const {
  if (numLandmarks==0 || numMeasurements==0 || numPositions==0)
    return;

  unsigned int num=0;
  Vec avg (0,0);
  Vec std (0,0);
  for (unsigned int ip=0; ip<numPositions; ++ip) {
    bool first = true;
    for (unsigned int il=0; il<landmarks.size(); ++il) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      calculateAverageDeviation (num, avg, std, m);
      double map_error = (positions[ip].position+avg.rotate(positions[ip].orientation)-landmarks[il].position).length();
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
        os << "; map_error=" << map_error;
        os << endl;
      }
    }
  }
}

void CameraLocalizationMapping::extractMeasurements (MeasurementList& dest, unsigned int lid, unsigned int pid) const {
  dest.clear();
  for (unsigned int im=0; im<numMeasurements; ++im) {
    if (measurements[im].landmark_id==lid && measurements[im].position_id==pid) {
      dest.push_back (measurements[im]);
    }
  }
}

void CameraLocalizationMapping::removeRelationship (unsigned int lid, unsigned int pid) {
  MeasurementList newlist;
  for (unsigned int im=0; im<numMeasurements; ++im) {
    if (measurements[im].landmark_id==lid && measurements[im].position_id==pid)
      continue;
    newlist.push_back (measurements[im]);
  }
  measurements=newlist;
  numMeasurements=measurements.size();
}

void CameraLocalizationMapping::removeWeakRelationships (unsigned int nmin) {
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
        change=false;
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

std::vector<std::vector<std::string> > CameraLocalizationMapping::connectedComponents() const {
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

double CameraLocalizationMapping::totalPositionError (double exponent) const {
  double sum=0;
  unsigned int n=0;
  for (unsigned int ip=0; ip<numPositions; ++ip) {
    for (unsigned int il=0; il<numLandmarks; ++il) {
      MeasurementList m;
      extractMeasurements (m, landmarks[il].id, positions[ip].id);
      if (m.size()>0) {
        Vec pexpect (0,0);
        for (unsigned int im=0; im<m.size(); ++im) {
          pexpect += positions[ip].position+m[im].position.rotate(positions[ip].orientation);
        }
        pexpect/=m.size();
        sum+=pow ((pexpect-landmarks[il].position).length(), exponent);
        n++;
      }
    }
  }
  return (n>0 ? sum/n : 0);
}

namespace {
  struct MDS {
    CameraLocalizationMeasurement measurement;
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

void CameraLocalizationMapping::trimMeasurements (double acceptRate) {
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
