
#ifndef _DerWeg_UKF_H__
#define _DerWeg_UKF_H__


#include <math.h>
#include <limits>
#include <vector>
#include <string>
#include <fstream>

#include "../Elementary/Eigen/Dense"


namespace DerWeg {

    using Eigen::MatrixXd;

    /** Class to represent UKF */
    class UKF {
    public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    MatrixXd H_sg_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Measurement noise standard deviation position1 in m
    double std_x_;

    ///* Measurement noise standard deviation position2 in m
    double std_y_;

    ///* Measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;


    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(VectorXd raw_measurements_, double delta_t);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

     /** @param meas_package The measurement at k+1
     */
    void Update(VectorXd raw_measurements_);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */

    void AugmentedSigmaPoints(MatrixXd *Xsig_out);

    void SigmaPointPrediction(MatrixXd &Xsig_aug, double delta_t);

    void PredictMeanAndCovariance();

    void PredictMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z);

    void UpdateState(VectorXd &z, VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z);

    };
};

#endif //UKF_UKF_H
