
#ifndef _DerWeg_StarGazerLandmarkList_h_
#define _DerWeg_StarGazerLandmarkList_h_

#include <vector>
#include <iostream>
#include "../Elementary/Vec.h"
#include "../Elementary/ConfigReader.h"

namespace DerWeg {

  /** description of one StarGazer landmark */
  struct StarGazerLandmark {
    unsigned int id;  ///< landmark id
    double scale;  ///< scaling factor, e.g. 1000 to get mm instead of m
    Vec position;  ///< relative to a fixed reference coordinate system
    Angle orientation;  ///< relative to a fixed reference coordinate system
    bool fixed;  ///< indicate whether landmark position is fixed during calibration

    StarGazerLandmark () : id(0), scale(1), fixed(false) {;}
    StarGazerLandmark (unsigned int id1, double scale1, Vec p1, Angle o1, bool fx1 = false) : id(id1), scale(scale1), position(p1), orientation(o1), fixed(fx1) {;}
    StarGazerLandmark (const StarGazerLandmark& sgl) { operator= (sgl); }
    const StarGazerLandmark& operator= (const StarGazerLandmark& sgl) {
      id=sgl.id;
      scale=sgl.scale;
      position=sgl.position;
      orientation=sgl.orientation;
      fixed=sgl.fixed;
      return *this;
    }
  };

  /** class to manage landmarks for StarGazer */
  class LandmarkList : public std::vector<StarGazerLandmark> {
  public:
    /** determine robot pose (pos, ori) observing landmark lid at position mpos and angle mori. Return true on success */
    bool determinePosition (Vec& pos, Angle& ori, unsigned int lid, Vec mpos, Angle mori) const;
    /** return array index of landmark with lid or a negative number, if no landmark lid exists */
    int findLandmark (unsigned int lid) const;
    /** read landmarks from ConfigReader cfg. Throw exception in case of error */
    void readLandmarks (const ConfigReader& cfg) throw (std::invalid_argument);
    /** write landmarks to a stream os */
    void writeLandmarks (std::ostream& os) const;
  };

}

#endif
