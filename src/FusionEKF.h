#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "ExtendedKF_rl.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Constructor.
   */
  FusionEKF();

  /**
   * Destructor.
   */
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  Eigen::VectorXd ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /*
  * Calculates the root mean squared error between the estimations of the Kalman filter
  * and the ground truth values.
  */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);
  

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  ExtendedKF_rl ekf_;

  /**
   * Tools instance for helper methods
   */

  Tools hm_;

 private:
  // check whether the tracking toolbox was initialized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long long previous_timestamp_;

  // Cycle number
  int cycle_number;

  // tool object used to compute Jacobian and RMSE
  Tools tools;
};

#endif // FusionEKF_H_
