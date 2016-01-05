#ifndef LANDMARKFINDER_H
#define LANDMARKFINDER_H

#include <opencv/cv.h>
#include "../Elementary/ConfigReader.h"
#include "calibio/CalibIO.hpp"

typedef std::vector<cv::Point> Cluster;

typedef struct
{
    int nID;
    int nPointCount;
    int nErrors;
    std::vector<cv::Point> voCorners;
    std::vector<cv::Point> voIDPoints;
    //cv::Vec<unsigned int, 2> oPosition;
    cv::Point oPosition;
} Landmark;



class LandmarkFinder
{
public:
    /// constructors and destructors
    LandmarkFinder(const DerWeg::ConfigReader&, CalibIO &, std::string);
    ~LandmarkFinder();

    /// accessors
    void SetImage(cv::Mat& i_oImage);
    void SetMaxRadiusForPixelCluster(float i_fMaxRadiusForPixelCluster);
    void SetMinPixelForCluster(unsigned int i_nMinPixelForCluster);
    void SetMaxPixelForCluster(unsigned int i_nMaxPixelForCluster);
    void SetMaxRadiusForCluster(float i_fMaxRadiusForCluster);
    void SetMinPointsPerLandmark(unsigned int i_nMinPointsPerLandmark);
    void SetMaxPointsPerLandmark(unsigned int i_nMaxPointsPerLandmark);
    /// methods
    int FindLandmarks(std::vector<Landmark>& o_vLandmarks);

    cv::Mat_<cv::Vec3b>         m_oImage;
    cv::Mat                     m_oOutputImage;
    cv::Mat                     m_oGrayImage;

private:
    cv::Mat                       m_oCalibMap_u;
    cv::Mat                       m_oCalibMap_v;

    char                        m_cThreshold;
    float                       m_fMaxRadiusForPixelCluster;
    unsigned int                m_nMinPixelForCluster;
    unsigned int                m_nMaxPixelForCluster;
    float                       m_fMaxRadiusForCluster;
    unsigned int                m_nMinPointsPerLandmark;
    unsigned int                m_nMaxPointsPerLandmark;
    float                       m_fMaxCosForRightAngle;
    std::vector<int>            m_vnIDs;
    std::vector<cv::Point>      m_vStuckPixels;

    std::vector<Landmark>       FindCorners(std::vector<Cluster>& Clusters);
    std::vector<cv::Point>      FindPoints(cv::Mat& i_oGrayImage);
    void                        FindClusters(std::vector<cv::Point>& i_voPoints, std::vector<Cluster>& o_voCluster, const float i_fRadiusThreshold, const unsigned int i_nMinPointsThreshold, const unsigned int i_nMaxPointsThreshold);
    int                         ThisLandmarkSucks(Landmark& io_oLandmark);
    int                         GetIDs(std::vector<Landmark>& io_voLandmarks);
    void                        Check(cv::Mat& Filtered,int XPos, int YPos, int Threshold, int& Pixelcount, int& SummedX, int& SummedY);
    void                        vec_sort(std::vector<int>& ids, std::vector<cv::Point>& points);
};

/// inlined accessors




inline void LandmarkFinder::SetMaxRadiusForPixelCluster(float i_fMaxRadiusForPixelCluster)
{
    m_fMaxRadiusForPixelCluster = i_fMaxRadiusForPixelCluster;
}

inline void LandmarkFinder::SetMinPixelForCluster(unsigned int i_nMinPixelForCluster)
{
    m_nMinPixelForCluster = i_nMinPixelForCluster;
}

inline void LandmarkFinder::SetMaxPixelForCluster(unsigned int i_nMaxPixelForCluster)
{
    m_nMaxPixelForCluster = i_nMaxPixelForCluster;
}

inline void LandmarkFinder::SetMaxRadiusForCluster(float i_fMaxRadiusForCluster)
{
    m_fMaxRadiusForCluster = i_fMaxRadiusForCluster;
}

inline void LandmarkFinder::SetMinPointsPerLandmark(unsigned int i_nMinPointsPerLandmark)
{
    m_nMinPointsPerLandmark = i_nMinPointsPerLandmark;
}

inline void LandmarkFinder::SetMaxPointsPerLandmark(unsigned int i_nMaxPointsPerLandmark)
{
    m_nMaxPointsPerLandmark = i_nMaxPointsPerLandmark;
}

#endif /// ifndef LANDMARKFINDER_H
