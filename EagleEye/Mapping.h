
#ifndef _DerWeg_MAPPING_H__
#define _DerWeg_MAPPING_H__

#include "../Blackboard/Blackboard.h"
#include "../Elementary/ThreadSafeLogging.h"

#include "DataObjects.h"
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
    // Rad angle
    double alpha;

    //double mean_xy;
    //Vec S_n;
    //Matrix2d cov;
    Matrix2d cov_inv;

    double accumulated_weights;

    Matrix2d cov_inv_non_oriented;
    //double covar_ratio;
    //double scaling_factor;

public:
    Cluster() {;}
    Cluster(double covar_ratio, double scaling_factor);

    Vec get_position();
    void update(Vec measurement, double rad_viewing_angle, double weight);
    double get_distance(Vec measurement);
};

class Mapping {

private:
    vector<Cluster> clusters;
    double covar_ratio;
    double cone_distance;
    double scaling_factor;
    double new_cluster_distance;
    double cone_radius;

    void add_cluster();

public:
    Mapping(){}
    Mapping(const ConfigReader& cfg);

    void add_measurement(Vec measurement, double rad_viewing_angle, double distance);

    void write_obstacles();

};

} // namespace

#endif
