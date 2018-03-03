#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                       const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse.fill(0.0);

  // check parameters
  if (estimations.empty() || estimations.size() != ground_truth.size()) {
    std::cerr << "CalculateRMSE() : invalid parameters" << std::endl;
    return rmse;
  }

  // accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    VectorXd delta = estimations[i] - ground_truth[i];
    rmse = rmse.array() + (delta.array() * delta.array());
  }

  // calculate the mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

double NormalizeAngle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;

  while (angle < -M_PI) 
    angle += 2.0 * M_PI;

  return angle;
}

} // namespace Tools