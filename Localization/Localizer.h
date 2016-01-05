#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <Eigen/Core>
#include "math.h"
#include "iostream"
#include "vector"
#include <opencv/highgui.h>
//#include <opencv2/core/eigen.hpp>
#include <stdarg.h>

#include "calibio/CalibIO.hpp"
#include "LandmarkFinder.h"
#include "../Elementary/ConfigReader.h"
#include "../Elementary/ThreadSafeLogging.h"
#include "libmv/numeric/levenberg_marquardt.h"



typedef struct
{
    int ID;
    float H;
    float X;
    float Y;
    float Theta;
} LandmarkPosition;

/* State Vector is defined as:
 * X:           X-Coordinate
 * Y:           Y-Coordinate
 * Theta        Orientation
 * v            Speed
 * theta_dot    Yawrate
 * */

/// todo: a kalman filter for angles within [-pi pi]?
class Localizer
{
public:
    /// constructors and destructors
    Localizer(const DerWeg::ConfigReader& cfg, const CalibIO& calibration);
    ~Localizer();

    /// accessors
    cv::Mat     GetState();
    std::vector<LandmarkPosition> GetLandmarks() const;
    cv::Mat     getCameraIntrinsics() const;

    /// Kalman Filter processing
    int         FindPosition(std::vector<Landmark>& i_voLandmarks, float fDeltaT);
    int         InitKF(cv::Mat& i_oInitialState, cv::Mat& i_oInitialCovariance);
    int         PredictKF(float fDeltaT);
    int         UpdateKF(cv::Mat& i_oMeasurement, cv::Mat& i_oMeasurementNoise);

    /// Public Helper Functions
    void        GetPointsFromID(int ID, std::vector<cv::Mat> &corner_points, std::vector<cv::Mat> &id_points);
    cv::Mat     calcReprojectionError(Landmark &lm, cv::Mat position);
    cv::Mat     points2mat(int count, ...);

private:
    /// Known landmarks
    std::vector<LandmarkPosition>    m_asLandmarkLUT;

    /// Kalman Filter Definitions
    cv::Mat     m_oStateVector;
    cv::Mat     m_oCovarianceMatrix;
    cv::Mat     m_oStateTransitionMatrix;
    cv::Mat     m_oProcessNoise;
    cv::Mat     m_oMeasModel;

    /// Camera intrinsics
    cv::Mat     m_oCameraIntrinsics;
    bool        m_bLocalizerInit;
    float       m_fFocalLength;
    float       m_fCx;
    float       m_fCy;

    /// private helper functions
    LandmarkPosition    GetLandmarkFromID(int ID);
    cv::Mat             TriangulateTwoPoints(cv::Mat& i_pointOneWorld,cv::Mat& i_pointOneImg,cv::Mat& i_pointTwoWorld,cv::Mat& i_pointTwoImg);
    int                 int_pow(int x, int p);
    void                visualizeLandmarks(std::vector<Landmark> &, cv::Mat);
};

/////////////////////////////////////
/// ACCESSORS
/////////////////////////////////////
inline cv::Mat Localizer::GetState()
{
    return m_oStateVector;
}

inline cv::Mat Localizer::getCameraIntrinsics() const
{
    return m_oCameraIntrinsics;
}

inline std::vector<LandmarkPosition> Localizer::GetLandmarks() const
{
    return m_asLandmarkLUT;
}

/////////////////////////////////////
/// OTHER
/////////////////////////////////////
inline int Localizer::int_pow(int x, int p) {
  if (p == 0) return 1;
  if (p == 1) return x;
  return x * int_pow(x, p-1);
}

// LibMv minimization
struct PoseFunctor
{
    typedef Eigen::VectorXd FMatrixType;
    typedef Eigen::Vector3d XMatrixType;
    Eigen::VectorXd operator()(Eigen::Vector3d Input) const;
    std::vector<Landmark>*  pLandmarks;
    std::vector<int>*       pSupporters;
    Localizer*              localizer;
};


#endif //#ifndef LOCALIZER_H

