
#ifndef _DerWeg_MAPPING_H__
#define _DerWeg_MAPPING_H__

#include "../Elementary/Vec.h"
#include <eigen3/Eigen/Dense>

using namespace std;

namespace DerWeg {

typedef Eigen::Matrix<double, 2, 2> Matrix2d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;

class Cluster {
private:
    vector<Vec> measurements;
    Vec mean;
    double mean_xy;
    Vec S_n;
    Matrix2d cov;
    Matrix2d cov_inv;
    double accumulated_weights;

public:
    Vec get_position();
    void update(Vec measurement, double weight);
    double get_distance(Vec measurement);
};

class Mapping {

private:
    vector<Cluster> clusters;
    double new_cluster_distance;


public:
    Mapping(const ConfigReader& cfg);

    void add_measurement(Vec measurement, double distance);

};

} // namespace

#endif
