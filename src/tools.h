#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace Tools {

  // A helper function to calculate RMSE.
  VectorXd CalculateRMSE(const std::vector<VectorXd> &estimations, const std::vector<VectorXd> &ground_truth);

  // A helper function to normalize an angle
  double NormalizeAngle(double angle);

} // namespace Tools

#endif /* TOOLS_H_ */