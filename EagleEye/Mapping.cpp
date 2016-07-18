
#include "Mapping.h"
#include <cmath>


using namespace std;
using namespace DerWeg;

/**
CLUSTERING CLASS
*/

Cluster::Cluster(double covar_ratio, double eff_scaling_factor) {
    Matrix2d cov_non_oriented;
    // Set covariance entries
    cov_non_oriented(0,0) = covar_ratio; // square root of eigenvalues of covariance matrix
    cov_non_oriented(1,1) = 1;
    cov_non_oriented(1,0) = 0;
    cov_non_oriented(0,1) = 0;

    cov_non_oriented *= eff_scaling_factor; // scaling to have correct size
    cov_non_oriented *= cov_non_oriented;

    if (covar_ratio != 0) {
        cov_inv_non_oriented = cov_non_oriented.inverse();
    } else {
        cov_inv_non_oriented(0,0) = 1;
        cov_inv_non_oriented(1,1) = 1;
        cov_inv_non_oriented(1,0) = 0;
        cov_inv_non_oriented(0,1) = 0;
        cov_inv_non_oriented /= pow(eff_scaling_factor, 2);
    }
}

Vec Cluster::get_position() {
    return mean;
}

/**
See http://www-uxsup.csx.cam.ac.uk/~fanf2/hermes/doc/antiforgery/stats.pdf
*/
void Cluster::update(Vec measurement, double rad_viewing_angle, double weight) {
    measurements.push_back(measurement);

    // Add new weight
    accumulated_weights += weight;

    Vec old_mean = mean;
    // Update mean of x,y and xy, alpha
    mean += weight/accumulated_weights * (measurement - mean);
    //mean_xy += weight/accumulated_weights * (measurement.x*measurement.y - mean_xy);

    alpha += weight/accumulated_weights * (rad_viewing_angle - alpha);

    double c = cos(alpha);
    double s = sin(alpha);
    Matrix2d rot;
    rot(0,0) = c;
    rot(0,1) = s;
    rot(1,0) = -s;
    rot(1,1) = c;

    cov_inv = rot.transpose() * cov_inv_non_oriented * rot;

    // Update S_n
    //S_n += weight * ( (measurement - old_mean).componentwise_mult(measurement - mean) );

    // Set covariance entries
//    cov(0,0) = S_n.x/accumulated_weights;
//    cov(1,1) = S_n.y/accumulated_weights;
//    cov(1,0) = mean_xy - mean.x * mean.y;
//    cov(0,1) = cov(1,0);

//    try {
//        if (measurements.size() > 1) {
//            cov_inv = cov.inverse();
//        } else {
//            cov_inv(0,0)=1;
//            cov_inv(0,1)=0;
//            cov_inv(1,0)=0;
//            cov_inv(1,1)=1;
//        }
//    } catch {
//        EOUT("Error in Clustering, matrix not invertible\n")
//        cov_inv(0,0)=1;
//        cov_inv(0,1)=0;
//        cov_inv(1,0)=0;
//        cov_inv(1,1)=1;
//    }
}

/**
Mahalanobis Distanz
*/
double Cluster::get_distance(Vec measurement) {
    Vec _diff = measurement - mean;
    Vector2d diff(_diff.x, _diff.y);
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
    new_cluster_distance = 1;
    cfg.get("Mapping::covar_ratio", covar_ratio);
    cfg.get("Mapping::cone_distance", cone_distance);
    cfg.get("Mapping::scaling_factor", scaling_factor);
    double cone_diameter;
    cfg.get("Mapping::cone_diameter", cone_diameter);
    cone_radius = cone_diameter / 2.0;
}

void Mapping::add_measurement(Vec measurement, double rad_viewing_angle, double distance) {
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
        Cluster c(covar_ratio, cone_distance/2 * scaling_factor);
        c.update(measurement, rad_viewing_angle, 1/distance);
        clusters.push_back(c);
    } else {
        // add to existing cluster
        clusters[arg_min].update(measurement, rad_viewing_angle, 1/distance);
    }

}

void Mapping::write_obstacles() {
    PylonMap map;
    for (unsigned i=0; clusters.size(); i++) {
        map.circles.push_back(Circle(clusters[i].get_position(), cone_radius));
    }
    BBOARD->setPylonMap(map);
}
