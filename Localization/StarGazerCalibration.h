
#ifndef _DerWeg_StarGazerCalibration_h_
#define _DerWeg_StarGazerCalibration_h_

#include "StarGazerLandmarkList.h"
#include <iostream>
#include <map>

namespace DerWeg {

  /** a struct to describe a robot position which is used for calibration */
  struct StarGazerRobotPosition {
    unsigned int id;  ///< an id for the position
    Vec position;  ///< the position relative to a fixed coordinate system
    Angle orientation;  ///< the orientation relative to a fixed coordinate system
    bool fixed;  ///< fixed position?

    StarGazerRobotPosition () : id(0), fixed(false) {;}
    StarGazerRobotPosition (unsigned int id1, Vec p1, Angle o1, bool fix1 =false) : id(id1), position(p1), orientation(o1), fixed(fix1) {;}
    StarGazerRobotPosition (const StarGazerRobotPosition& sgp) { operator= (sgp); }
    const StarGazerRobotPosition& operator= (const StarGazerRobotPosition& sgp) {
      id=sgp.id;
      position=sgp.position;
      orientation=sgp.orientation;
      fixed=sgp.fixed;
      return *this;
    }
  };

  class RobotPositionList : public std::vector<StarGazerRobotPosition> {
  public:
    void readPositions (const ConfigReader& cfg) throw (std::invalid_argument);
    void writePositions (std::ostream& os) const;
  };

  /** a struct to decribe StarGazer measurements containing the ids of the
    position id, the landmark id, and the position and orientation sensed by StarGazer */
  struct StarGazerCalibrationMeasurement {
    Vec position;
    Angle orientation;
    unsigned int landmark_id;
    unsigned int position_id;

    StarGazerCalibrationMeasurement () : landmark_id (0), position_id (0) {;}
    StarGazerCalibrationMeasurement (Vec p1, Angle o1, unsigned int lid, unsigned int pid) : position (p1), orientation (o1), landmark_id (lid), position_id (pid) {;}
    StarGazerCalibrationMeasurement (const StarGazerCalibrationMeasurement& rcp) { operator= (rcp); }
    const StarGazerCalibrationMeasurement& operator= (const StarGazerCalibrationMeasurement& rcp) {
      position=rcp.position;
      orientation=rcp.orientation;
      landmark_id=rcp.landmark_id;
      position_id=rcp.position_id;
      return *this;
    }
  };

  class MeasurementList : public std::vector<StarGazerCalibrationMeasurement> {
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
  class StarGazerCalibration {
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
    static void calculateAverageAngleDeviation (unsigned int& num, Angle& avg, double& std, const MeasurementList& m);
  public:
    StarGazerCalibration () {;}
    ~StarGazerCalibration () {;}

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
    void calibrateLandmarks ();
    /** only determine positions, do not update landmarks */
    void determinePositions ();
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
    /** calculate the orientation error of the registration (in degree) */
    double totalAngleError (double exponent) const;
    /** calculate connected components. Returns one vector per component
        containing the landmark ids and position ids */
    std::vector<std::vector<std::string> > connectedComponents() const;
    /** return for each position the number of connected landmarks */
    std::vector<unsigned int> connectedLandmarks() const;
  };
}

#endif
