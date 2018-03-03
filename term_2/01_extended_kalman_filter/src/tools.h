#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Tools {

  /**
  * A function to calculate RMSE.
  */
  VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

  /**
  * A function to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  /**
   * A function to normalize angles to the range -pi to pi
   */
  float NormalizeAngle(float angle); 

} // namespace Tools

#endif /* TOOLS_H_ */
