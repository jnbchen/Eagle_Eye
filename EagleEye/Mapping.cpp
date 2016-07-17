
#include "Mapping.h"
#include <cmath>


using namespace std;
using namespace DerWeg;

/**
CLUSTERING CLASS
*/

Vec Cluster::get_position() {
    return mean;
}

/**
See http://www-uxsup.csx.cam.ac.uk/~fanf2/hermes/doc/antiforgery/stats.pdf
*/
void Cluster::update(Vec measurement, double weight) {
    measurements.push_back(measurement);

    // Add new weight
    accumulated_weights += weight;

    Vec old_mean = mean;
    // Update mean of x,y and xy
    mean += weight/accumulated_weights * (measurement - mean);
    mean_xy += weight/accumulated_weights * (measurement.x*measurements.y - mean_xy);

    // Update S_n
    S_n += weight * ( (measurement - old_mean).componentwise_mult(measurement - mean) );

    // Set covariance entries
    cov(0,0) = S_n.x/accumulated_weights;
    cov(1,1) = S_n.y/accumulated_weights;
    cov(1,0) = mean_xy - mean.x * mean.y;
    cov(0,1) = cov(1,0);

    try {
        if (measurements.size() > 1) {
            cov_inv = cov.inverse();
        } else {
            cov_inv(0,0)=1;
            cov_inv(0,1)=0;
            cov_inv(1,0)=0;
            cov_inv(1,1)=1;
        }
    } catch {
        EOUT("Error in Clustering, matrix not invertible\n")
        cov_inv(0,0)=1;
        cov_inv(0,1)=0;
        cov_inv(1,0)=0;
        cov_inv(1,1)=1;
    }
}

/**
Mahalanobis Distanz
*/
double Cluster::get_distance(Vec measurement) {
    Vec _diff = measurement - mean;
    vector2d diff(_diff.x,_diff.y);
    double prod = diff.transpose() * cov_inv * diff;
    if (prod >= 0) {
        return sqrt(prod);
    } else {
        EOUT("Error, attempting to sqrt negative number\n");
        return _diff.length();
    }
}




/**
MAPPING CLASS
*/

Mapping::Mapping(const ConfigReader& cfg) {
    cfg.get("Mapping::new_cluster_distance", new_cluster_distance);
}

void Mapping::add_measurement(Vec measurement, double distance) {
    double min_dist = 1000000000;
    int arg_min = -1;
    for (unsigned int i=0; i<clusters.size(); i++){
        double d = clusters[i].get_distance(measurement);
        if (d<min_dist || arg_min < 0) {
            min_dist = d;
            arg_min = i;
        }
    }
    if (min_dist > new_cluster_distance || arg_min < 0) {
        // generate new cluster
        Cluster c;
        c.update(measurement, 1/distance);
        clusters.push_back(c);
    } else {
        // add to existing cluster
        clusters[i].update(measurement, 1/distance);
    }

}

