
#ifndef _DerWeg_CameraLocalizationMapping_h_
#define _DerWeg_CameraLocalizationMapping_h_

#include <iostream>
#include <map>
#include <vector>
#include "../../Elementary/Vec.h"
#include "../../Elementary/ConfigReader.h"

namespace DerWeg {

  /** description of one landmark for camera localization */
  struct Landmark {
    unsigned int id;  ///< landmark id
    double height;  ///< the height of the landmark (same length unit as position)
    Vec position;  ///< relative to a fixed reference coordinate system
    bool fixed;  ///< indicate whether landmark position is fixed during calibration
    
    Landmark () : id(0), height(1), fixed(false) {;}
    Landmark (unsigned int id1, double height1, Vec p1,  bool fx1 = false) : id(id1), height(height1), position(p1), fixed(fx1) {;}
    Landmark (const Landmark& sgl) { operator= (sgl); }
    const Landmark& operator= (const Landmark& sgl) {
      id=sgl.id;
      height=sgl.height;
      position=sgl.position;
      fixed=sgl.fixed;
      return *this;
    }
  };
  
  /** class to manage landmarks for camera localization */
  class LandmarkList : public std::vector<Landmark> {
  public:
    /** return array index of landmark with lid or a negative number, if no landmark lid exists */
    int findLandmark (unsigned int lid) const;
    /** read landmarks from ConfigReader cfg. Throw exception in case of error */
    void readLandmarks (const ConfigReader& cfg) throw (std::invalid_argument);
    /** write landmarks to a stream os */
    void writeLandmarks (std::ostream& os) const;
  };
  
  /** a struct to describe a robot position which is used for calibration */
  struct RobotPosition {
    unsigned int id;  ///< an id for the position
    Vec position;  ///< the position relative to a fixed coordinate system
    Angle orientation;  ///< the orientation relative to a fixed coordinate system
    bool fixed;  ///< fixed position?

    RobotPosition () : id(0), fixed(false) {;}
    RobotPosition (unsigned int id1, Vec p1, Angle o1, bool fix1 =false) : id(id1), position(p1), orientation(o1), fixed(fix1) {;}
    RobotPosition (const RobotPosition& sgp) { operator= (sgp); }
    const RobotPosition& operator= (const RobotPosition& sgp) {
      id=sgp.id;
      position=sgp.position;
      orientation=sgp.orientation;
      fixed=sgp.fixed;
      return *this;
    }
  };

  class RobotPositionList : public std::vector<RobotPosition> {
  public:
    void readPositions (const ConfigReader& cfg) throw (std::invalid_argument);
    void writePositions (std::ostream& os) const;
  };

  /** a struct to decribe measurements containing the ids of the
    position id, the landmark id, and the position of the landmark given the robot position */
  struct CameraLocalizationMeasurement {
    Vec position;
    unsigned int landmark_id;
    unsigned int position_id;

    CameraLocalizationMeasurement () : landmark_id (0), position_id (0) {;}
    CameraLocalizationMeasurement (Vec p1, unsigned int lid, unsigned int pid) : position (p1), landmark_id (lid), position_id (pid) {;}
    CameraLocalizationMeasurement (const CameraLocalizationMeasurement& rcp) { operator= (rcp); }
    const CameraLocalizationMeasurement& operator= (const CameraLocalizationMeasurement& rcp) {
      position=rcp.position;
      landmark_id=rcp.landmark_id;
      position_id=rcp.position_id;
      return *this;
    }
  };

  class MeasurementList : public std::vector<CameraLocalizationMeasurement> {
  public:
    void readMeasurements (const ConfigReader& cfg) throw (std::invalid_argument);
    void writeMeasurements (std::ostream& os) const;
  };
  
  /** struct to model the tension in the network */
  struct TensionTriple {
    TensionTriple ();
    TensionTriple (const TensionTriple&);
    const TensionTriple& operator= (const TensionTriple&);
    bool operator< (const TensionTriple&) const; ///< compare errors

    unsigned int landmark_id;
    unsigned int position_id;
    double error;
    double length;
    double std;
    unsigned int num;
    bool fixed;
  };

  /** Class contains the calibration procedure */
  class CameraLocalizationMapping {
    LandmarkList landmarks;
    RobotPositionList positions;
    MeasurementList measurements;
    
    unsigned int numLandmarks;
    unsigned int numPositions;
    unsigned int numMeasurements;

    std::map<unsigned int,unsigned int> lidmap;
    std::map<unsigned int,unsigned int> pidmap;

    void extractMeasurements (MeasurementList& dest, unsigned int lid, unsigned int pid) const;
    static void calculateAverageDeviation (unsigned int& num, Vec& avg, Vec& std, const MeasurementList& m);
  public:
    CameraLocalizationMapping () {;}
    ~CameraLocalizationMapping () {;}

    /** add the following landmarks, positions, and measurements for calibration */
    void addAndMerge (const LandmarkList& landmarks1, const RobotPositionList& positions1, const MeasurementList& measurements1);
    /** obtain all landmarks */
    const LandmarkList& getLandmarks() const;
    /** obtain all positions */
    const RobotPositionList& getPositions() const;
    /** obtain all measurements */
    const MeasurementList& getMeasurements() const;

    /** registering all landmarks and positions using an iterative
        calculation. Position 0 is used as reference position that
        defines the coordinate system */
    void calibrateLandmarks (unsigned int nmax =0);
    /** only determine robot positions and return total movement of positions.
     * If keep_fixed is set, keep those positions unchanged which are declared to be fixed positions */
    double determinePositions (bool keep_fixed);
    double determinePositions (bool keep_fixed, std::vector<bool>& positions_used, const std::vector<bool>& landmarks_used);
    /** only determine landmark positions and return total movement of positions */
    double determineLandmarks ();
    double determineLandmarks (std::vector<bool>& landmarks_used, const std::vector<bool>& positions_used);
    /** remove all landmark-position-relationsships with less than nmin measurements */
    void removeWeakRelationships (unsigned int nmin);
    /** remove landmark-position-relationships with standard deviation larger than maxstd */
    void removeLargeDeviation (double maxstd);
    /** remove landmark-position-relationships with average distance larger than maxdist */
    void removeLargeDistances (double maxdist);
    /** remove all measurements for a given landmark-position-relationsship specified by the landmark id lid and position id pid */
    void removeRelationship (unsigned int lid, unsigned int pid);
    /** for each landmark-position-relationship, remove 1-acceptRate of all measurements */
    void trimMeasurements (double acceptRate);
    /** return list of tensions in network */
    std::vector<TensionTriple> getTensions ();
    /** remove measurement relationship with largest error */
    void removeMostTensioningMeasurements (bool ignore_fixed, std::ostream& os);
    /** remove relationships that support only one position */
    void removeIrrelevantMeasurements (std::ostream& os);
    /** print a statistics on the relative position and standard deviation of all landmark-position-relationships to os */
    void printStatisticsLandmarks (std::ostream& os) const;
    void printStatisticsPositions (std::ostream& os) const;
    /** calculate the position error of the registration */
    double totalPositionError (double exponent) const;
    /** calculate connected components. Returns one vector per component
        containing the landmark ids and position ids */
    std::vector<std::vector<std::string> > connectedComponents() const;
    /** return for each position the number of connected landmarks */
    std::vector<unsigned int> connectedLandmarks() const;
  };
}

#endif
