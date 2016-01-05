
#include "StarGazerProxy.h"
#include "../Elementary/ThreadSafeLogging.h"
#include <sys/stat.h>
#include <cmath>

using namespace std;
using namespace DerWeg;

namespace {
  unsigned int hamming_distance (unsigned int a, unsigned int b) {
    unsigned int res=0;
    while (a!=0 || b!=0) {
      res += (((a&1)!=(b&1)) ? 1 : 0);
      a>>=1;
      b>>=1;
    }
    return res;
  }
}


DerWeg::StarGazerPose::StarGazerPose () {
  valid_measurement = false;
  id = 1;
  landmark_exists = false;
  hamming_id = 0;
  height = 1;
  stddev_position = 1e6;
  stddev_orientation = 1e6;
}

DerWeg::StarGazerPose::StarGazerPose (const DerWeg::StarGazerPose& p) {
  operator=(p);
}

const DerWeg::StarGazerPose& DerWeg::StarGazerPose::operator= (const DerWeg::StarGazerPose& p) {
  valid_measurement = p.valid_measurement;
  id = p.id;
  landmark_exists = p.landmark_exists;
  hamming_id = p.hamming_id;
  height = p.height;
  position_relative = p.position_relative;
  orientation_relative = p.orientation_relative;
  position_global = p.position_global;
  orientation_global = p.orientation_global;
  time = p.time;
  stddev_position = p.stddev_position;
  stddev_orientation = p.stddev_orientation;
  return *this;
}


StarGazerProxy::StarGazerProxy () :
  sg(NULL),
  pp_offset(1e-3*150.920144, 1e-3*17.214267),
  pp_slope(-0.057906,0.041776),
  position_offset_mounting (0,0),
  orientation_offset_mounting (DerWeg::Angle::zero),
  far_parameters(5,0),
  latest_landmark_id (1),
  stddev_position (2000),
  stddev_orientation (15*M_PI/180)
{
  far_parameters[0]=2;
  far_parameters[1]=2.5;
  far_parameters[2]=0.2;
  far_parameters[3]=-0.8;
  far_parameters[4]=1.8;
}

StarGazerProxy::~StarGazerProxy () {
  if (sg) {
    delete sg;
  }
}

void StarGazerProxy::init (const ConfigReader& cfg) {
  std::string device;
  cfg.get("StarGazer::device", device);
  try{
    struct stat file_info;
    int r = stat(device.c_str(),&file_info);
    if (r!=0)
      throw runtime_error (string("openStarGazer: device ")+device+string(" does not exist."));
    if (!S_ISCHR(file_info.st_mode))
      throw runtime_error (string("openStarGazer: device ")+device+string(" is not a character device."));
    if ((file_info.st_mode & 6) != 6)
      throw runtime_error (string("openStarGazer: device ")+device+string(" does not have read/write access."));
    sg = new StarGazer (device);

    // Don't remove the following line although it seems to be needless!
    // Doing some gentle communication motivates the star gazer to work properly.
    sg->read_parameter( "MarkMode" );
  }catch(std::runtime_error& e){
    std::string emsg1=e.what();
    try{
      // try it a second time since the star gazer often behaves like a feisty donkey
      sg = new StarGazer (device);
      sg->read_parameter( "MarkMode" );
    }catch(std::runtime_error& e){
      EOUT("StarGazer did not start twice. Error messages were:\n" << emsg1 << "\n" << e.what() << std::endl);
      sg=NULL;
    }
  }
  std::vector<double> parameters;
  if (cfg.get("StarGazer::pp_bug_parameters", parameters) && parameters.size()>=4) {
    pp_offset.x = parameters[0];
    pp_offset.y = parameters[1];
    pp_slope.x = parameters[2];
    pp_slope.y = parameters[3];
  }
  if (cfg.get("StarGazer::far_bug_parameters", parameters) && parameters.size()>=5) {
    far_parameters = parameters;
  }
  if (cfg.get("StarGazer::mounting_offset", parameters) && parameters.size()>=3) {
    position_offset_mounting.x=1e-3*parameters[0];
    position_offset_mounting.y=1e-3*parameters[1];
    orientation_offset_mounting=Angle::deg_angle(parameters[2]);
  }
  if (!sg)
    throw runtime_error ("could not connect to StarGazer");
  sg->start_calc();
  landmarks.readLandmarks (cfg);
}

StarGazerPose StarGazerProxy::filterSGMeasurement (StarGazer::PositionData& pd) {
  return filterSGMeasurement (pd, pd.id);
}

void StarGazerProxy::updateStddev (StarGazer::PositionData& pd) {
  if (!pd.dead && static_cast<int>(latest_landmark_id)==pd.id) {
    // exponentielles Abklingen der Standardabweichung, wenn immer die selbe
    // ID gesehen wird, da interner Filter im StarGazer dann konvergiert
    stddev_position = 0.7*stddev_position+0.3*100;
    stddev_orientation = 0.7*stddev_orientation+0.3*1*M_PI/180;
  } else {
    // grosse Standardabweichung, wenn die ID wechselt, da interner Filter im
    // StarGazer diesen Fall nicht vernuenftig behandelt und deshalb Phantom-
    // Positionen liefert
    stddev_position = 2000;
    stddev_orientation = 15*M_PI/180;
  }
  if (!pd.dead) {
    latest_landmark_id = pd.id;
  } else {
    latest_landmark_id = 1;  // Landmarken-ID 1 existiert nicht, daher unkritisch
  }
}

StarGazerPose StarGazerProxy::filterSGMeasurement (StarGazer::PositionData& pd, unsigned int lid) {
  // zu den Einheiten:
  // aus dem StarGazer kommen "virtuelle" Laengeneinheiten, je nach
  // Landmarkengroesse. Die Umrechung in mm steht in landmarks[i].scale
  // typischerweise ist der Umrechungsfaktor 1000
  // Winkel kommen vom StarGazer in rad

  StarGazerPose res;
  res.valid_measurement = !pd.dead;
  if (!res.valid_measurement) return res; // keine Landmarke detektiert

  res.id = lid;
  int lindex = landmarks.findLandmark (lid);
  res.landmark_exists = (lindex>=0) && (lindex<static_cast<int>(landmarks.size()));
  res.hamming_id = hamming_distance (lid, pd.id);
  double scale = lindex<0 ? 1000 : landmarks[lindex].scale;  
  res.height = scale*pd.z;
  res.orientation_relative = Angle::rad_angle(pd.theta);

  // Beruecksichtigung des PrinciplePoint-bugs
  res.position_relative = (Vec(pd.x, pd.y)+(pp_offset+pd.z*pp_slope).rotate(res.orientation_relative));
  double len = res.position_relative.length();
  if (len>far_parameters[1]) {
    // unglaubwuerdig grosser Abstand
    res.valid_measurement = false;
    if (len>far_parameters[0]) {
      // Kompensation des Kipp-Effektes bei grossen Abstaenden
      res.position_relative *= 1.0/(far_parameters[2]*len*len+far_parameters[3]*len+far_parameters[4]);
    }
  }
  // Umrechnung in mm durch Skalierung:
  res.position_relative*=scale;

  // Kompensation des Montagefehlers
  res.orientation_relative+=orientation_offset_mounting;
  res.position_relative-=position_offset_mounting.rotate(res.orientation_relative);
  if (res.landmark_exists) {
    res.position_global=landmarks[lindex].position+res.position_relative.rotate(landmarks[lindex].orientation);
    res.orientation_global=landmarks[lindex].orientation+res.orientation_relative;
  }
  res.stddev_orientation = stddev_orientation;
  res.stddev_position = stddev_position;
  return res;
}

StarGazerPose StarGazerProxy::getPose () {
  StarGazerPose res;
  if (sg) {
    StarGazer::PositionData pd = sg->get_position();
    updateStddev (pd);
    res = filterSGMeasurement (pd);
  } else {
    res.valid_measurement = false;
  }
  return res;
}

std::vector<StarGazerPose> StarGazerProxy::getAllPoses () {
  std::vector<StarGazerPose> res (landmarks.size());
  if (sg) {
    StarGazer::PositionData pd = sg->get_position();
    updateStddev (pd);
    for (unsigned int i=0; i<res.size(); ++i) {
      res[i] = filterSGMeasurement (pd, landmarks[i].id);
    }
  } else {
    for (unsigned int i=0; i<res.size(); ++i) {
      res[i].valid_measurement = false;
    }
  }
  return res;
}
