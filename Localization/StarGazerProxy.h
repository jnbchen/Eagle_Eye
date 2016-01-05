
#ifndef _DerWeg_StarGazerProxy_h_
#define _DerWeg_StarGazerProxy_h_

#include "StarGazer.hpp"
#include "StarGazerLandmarkList.h"
#include "../Elementary/Timestamp.h"

namespace DerWeg {

  /** Struct, um die StarGazer-Messung zu repraesentieren */
  struct StarGazerPose {
    bool valid_measurement;      ///< wurde ueberhaupt eine Messung gemacht?
    unsigned int id;             ///< ID der beobachteten Landmarke
    bool landmark_exists;        ///< ist Landmarke in Karte (LandmarkList) enthalten?
    unsigned int hamming_id;     ///< Hamming-Distanz zwischen beobachteter Landmarken-ID und referenzierter ID
    double height;               ///< Hoehe der Landmarke (in mm)
    Vec position_relative;       ///< Relativposition zur Landmarke (in mm)
    Angle orientation_relative;  ///< relative Orientierung zur Landmarke
    Vec position_global;         ///< Fahrzeugposition in Weltkoordinaten (in mm)
    Angle orientation_global;    ///< Fahrzeugorientierung in Weltkoordinaten
    Timestamp time;              ///< Referenzzeitpunkt
    double stddev_position;      ///< (heuristische) Standardabweichung der Position in mm
    double stddev_orientation;   ///< (heuristische) Standardabweichung der Orientierung in rad

    StarGazerPose ();
    StarGazerPose (const StarGazerPose&);
    const StarGazerPose& operator= (const StarGazerPose&);
  };

  /** class that integrates the StarGazer and provides methods
   * to filter landmarks and map poses to global coordinates */
  class StarGazerProxy {
    StarGazer* sg;  ///< pointer to StarGazer object (or NULL)
    Vec pp_offset;  ///< position offset due to "principle point bug" in StarGazer (in StarGazer-Laengeneinheiten)
    Vec pp_slope;   ///< position slope offset due to "principle point bug" in StarGazer (in StarGazer-Laengeneinheiten)
    Vec position_offset_mounting;       ///< Aufbau-bedingter Offset des StarGazers (in mm)
    Angle orientation_offset_mounting;  ///< Aufbau-bedingter Offset des StarGazers
    std::vector<double> far_parameters; ///< parameters to compensate the "far distance bug" in StarGazer (in StarGazer-Laengeneinheiten)
    unsigned int latest_landmark_id;    ///< ID of the landmark that was seen last time
    double stddev_position;             ///< a kind of pseudo standard deviation of the position measurement (in mm)
    double stddev_orientation;          ///< a kind of pseudo standard deviation of the orientation measurement (in rad)

    StarGazerPose filterSGMeasurement (StarGazer::PositionData& pd);  ///< apply heuristics and calculate global position
    StarGazerPose filterSGMeasurement (StarGazer::PositionData& pd, unsigned int lid);  ///< same as above, but refer to landmark 'lid' instead of landmark 'pd.id'
    void updateStddev (StarGazer::PositionData& pd); ///< update internal standard deviation variables

  public:
    LandmarkList landmarks;     ///< List of Landmarks (the map)
    StarGazerPose getPose ();   ///< return next StarGazer measurement, apply heuristics and map it to global coordinates
    std::vector<StarGazerPose> getAllPoses ();   ///< return next StarGazer measurement, apply heuristics and map it to global coordinates. Check all landmarks known and return the possible pose for each

    StarGazerProxy ();
    ~StarGazerProxy ();
    /** init StarGazer, read landmarks, and parameters.
     * Throws runtime_error if StarGazer cannot be initialized. */
    void init (const ConfigReader& cfg);
  };

}

#endif
