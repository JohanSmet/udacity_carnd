#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

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

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	// check division by zero
	if(fabs(c1) < 0.0001) {
		std::cerr << "CalculateJacobian () - Error - Division by Zero" << std::endl;
		Hj = MatrixXd::Zero(3,4);
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		   -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}

float NormalizeAngle(float angle) {
	float result = angle;
	
	while (result < - M_PI)
		result += 2 * M_PI;
	
	while (result > M_PI)
		result -= 2 * M_PI;
	
	return result;
}

} // namespace Tools
