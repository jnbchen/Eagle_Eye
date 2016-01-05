
#include "StarGazerLandmarkList.h"
#include <sstream>

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

bool LandmarkList::determinePosition (Vec& pos, Angle& ori, unsigned int lid, Vec mpos, Angle mori) const {
  int i = findLandmark (lid);
  if (i<0)
    return false;
  pos=(*this)[i].position+(*this)[i].scale*mpos.rotate ((*this)[i].orientation);
  ori=(*this)[i].orientation+mori;
  return true;
}

void LandmarkList::readLandmarks (const ConfigReader& cfg) throw (invalid_argument) {
  (*this).clear();
  vector<string> ids;
  if (!cfg.get ("StarGazer::landmarks", ids)) {
    throw invalid_argument ("readLandmarks: key StarGazer::landmarks not found");
  }
  for (unsigned int i=0; i<ids.size(); i++) {
    vector<string> params;
    if (!cfg.get ((string("StarGazer::L")+ids[i]).c_str(), params)) {
      throw invalid_argument (string("readLandmarks: key L")+ids[i]+string(" not found"));
    }
    if (params.size()<5) {
      throw invalid_argument (string("readLandmarks: key L")+ids[i]+string(" incomplete"));
    }
    StarGazerLandmark lm;
    stringstream inout;
    inout << params[0] << ' ' << params[1] << ' ' << params[2] << ' ' << params[3] << ' ' << params[4] << std::flush;
    double aa;
    inout >> lm.id >> lm.scale >> lm.position.x >> lm.position.y >> aa;
    lm.orientation = Angle::deg_angle (aa);
    lm.fixed=(params.size()>=6 && params[5]=="*");
    (*this).push_back (lm);
  }
}

void LandmarkList::writeLandmarks (ostream& os) const {
  os << "[StarGazer]\n";
  os << "# parameters for multi landmark star gazer applications\n";
  os << "# automatically created\n";
  os << "landmarks =";
  for (unsigned int i=0; i<(*this).size(); i++) {
    os << ' ' << (*this)[i].id;
  }
  os << '\n';
  os << "# Syntax: Lid = landmark_id scaling position.x position.y orientation(deg) fixed(*)\n";
  for (unsigned int i=0; i<(*this).size(); i++) {
    os << 'L' << (*this)[i].id << " = " << (*this)[i].id << ' ' << (*this)[i].scale << ' ' << (*this)[i].position.x << ' ' << (*this)[i].position.y << ' ' << (*this)[i].orientation.get_deg();
    if ((*this)[i].fixed) {
      os << " *";
    }
    os << '\n';
  }
}
