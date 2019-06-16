#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Constructor with given data
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
   */
  KalmanFilter(int X_length, MatrixXd P_in, MatrixXd F_in, MatrixXd Q_in, 
               MatrixXd H_laser_in, MatrixXd R_laser_in, 
               MatrixXd H_radar_in, MatrixXd R_radar_in);

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Function to initialize vector x
   */
  void InitX(VectorXd x_init);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /**
   * Gets the x vector to be printed or used in other calculations
   */
  
  VectorXd getX();

  /**
   * Gets the n element of the x vector to be printed or used in other calculations
   */

  float getX_n(int n);

  /**
   * Gets the P matrix to be printed or used in other calculations
   */
  MatrixXd getP();

  /**
   * Prints all variables of Kalman Filter for debugging purposes
   */

  void printAllVariables();


  private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // measurement covariance matrix for laser
  Eigen::MatrixXd R_laser_;

  // measurement covariance matrix for radar
  Eigen::MatrixXd R_radar_;

  // observation matrix for laser
  Eigen::MatrixXd H_laser_;

  // observation matrix for radar (jacobian)
  Eigen::MatrixXd H_radar_;
};

#endif // KALMAN_FILTER_H_
