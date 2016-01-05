
#ifndef _DerWeg_HeadingFilter_h_
#define _DerWeg_HeadingFilter_h_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <eigen2/Eigen/Dense>
#include <stdexcept>

namespace DerWeg {

  /** Kalman-Filter zum Schaetzen der Fahrzeugorientierung aus einem Gyroskop
   * state(1): Gierwinkel
   * state(2): Gierrate
   * state(3): Bias auf der Gierrate */
  class HeadingFilter {
    Eigen::Matrix<double, 3, 1> state;
    Eigen::Matrix<double, 3, 3> state_covariance;
    boost::posix_time::ptime reference_time;

    Eigen::Matrix<double, 3, 3> state_transition_matrix;
    Eigen::Matrix<double, 3, 3> state_transition_covariance;

    Eigen::Matrix<double, 1, 3> gyro_observation_matrix;
    Eigen::Matrix<double, 1, 1> gyro_observation_covariance;
    Eigen::Matrix<double, 1, 1> gyro_observation;
    Eigen::Matrix<double, 2, 3> gyro_nomotion_observation_matrix;
    Eigen::Matrix<double, 2, 2> gyro_nomotion_observation_covariance;
    Eigen::Matrix<double, 2, 1> gyro_nomotion_observation;
    Eigen::Matrix<double, 1, 3> yaw_observation_matrix;
    Eigen::Matrix<double, 1, 1> yaw_observation_covariance;
    Eigen::Matrix<double, 1, 1> yaw_observation;

    template<int ODIM>
    void innovation (const Eigen::Matrix<double, ODIM, 1>& z, const Eigen::Matrix<double, ODIM, 3>& H, const Eigen::Matrix<double, ODIM, ODIM>& R);
    template<int DIMX, int DIMY>
    void y_x_yt (Eigen::Matrix<double, DIMY, DIMY>& z, const Eigen::Matrix<double, DIMX, DIMX>& x, const Eigen::Matrix<double, DIMY, DIMX>& y);  // z=y*x*y^T wobei x eine symmetrische Matrix ist
    template<int DIM>
    bool is_symmetric (Eigen::Matrix<double, DIM, DIM> m);  // checks whether matrix is symmetric or not (up to numerical precision). If matrix contains Inf or NaN, it returns false
  public:
    HeadingFilter ();
    ~HeadingFilter ();

    /** prediction step. Provide the point in time until which you want to predict */
    void predict (boost::posix_time::ptime t);
    /** innovation step using a gyro measurment in rad/s */
    void observe_gyro (double yaw_rate_observed);
    /** innovation step using an gyro measurment in rad/s assuming non-moving vehicle */
    void observe_gyro_nomotion (double yaw_rate_observed);
    /** innovation step using yaw angle measurement in rad */
    void observe_yaw_angle (double yaw_angle_observed, double stddev);

    /** the present time until prediction has been done yet */
    boost::posix_time::ptime get_time () throw ();
    /** the present state */
    Eigen::Matrix<double, 3, 1> get_state () throw ();
    /** the present covariance matrix */
    Eigen::Matrix<double, 3, 3> get_covariance () throw ();
    /** return the present yaw angle in rad */
    double get_yaw_angle () const throw ();
  };

}

template<int ODIM>
void DerWeg::HeadingFilter::innovation (const Eigen::Matrix<double, ODIM, 1>& z, const Eigen::Matrix<double, ODIM, 3>& H, const Eigen::Matrix<double, ODIM, ODIM>& R) {
  Eigen::Matrix<double, ODIM, 1> dz = z-H*state;
  Eigen::Matrix<double, ODIM, ODIM> S;
  y_x_yt<3,ODIM> (S, state_covariance, H);
  S += R;
  double lambda = 1e-10;
  Eigen::Matrix<double, ODIM, ODIM> Sinv;
  while (true) {
    Sinv = S.inverse();
    bool is_okay = is_symmetric<ODIM> (Sinv);
    if (is_okay) {
      break;
    } else {
      for (unsigned int i=0; i<ODIM; ++i)
        S(i,i)+=lambda;
      lambda*=10;
    }
  }
  Eigen::Matrix<double, 3, ODIM> K = state_covariance*H.transpose()*Sinv;
  state+=K*dz;
  state_covariance-=K*S*K.transpose();
}

template<int DIMX, int DIMY>
void DerWeg::HeadingFilter::y_x_yt (Eigen::Matrix<double, DIMY, DIMY>& z, const Eigen::Matrix<double, DIMX, DIMX>& x, const Eigen::Matrix<double, DIMY, DIMX>& y) {
  for (int v=0; v<DIMY; ++v) {
    for (int u=0; u<=v; ++u) {
      double r=0;
      for (int l=0; l<DIMX; ++l) {
        for (int k=0; k<DIMX; ++k) {
          r+=y(v,k)*x(k,l)*y(u,l);
        }
      }
      z(u,v)=z(v,u)=r;
    }
  }
}

template<int DIM>
bool DerWeg::HeadingFilter::is_symmetric (Eigen::Matrix<double, DIM, DIM> m) {
  for (int v=0; v<DIM; ++v) {
    for (int u=0; u<v; ++u) {
      if (std::abs(m(u,v)-m(v,u))>1e-10)
        return false;
    }
  }
  return true;
}


#endif
