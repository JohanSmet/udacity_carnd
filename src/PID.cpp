#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : p_error(0), i_error(0), d_error(0) {

}

PID::~PID() {

}

void PID::Init(double p_Kp, double p_Ki, double p_Kd) {
  Kp = p_Kp;
  Ki = p_Ki;
  Kd = p_Kd;
}

void PID::UpdateError(double p_cte) {
  d_error = p_cte - p_error;
  p_error = p_cte;
  i_error += p_cte;
}

double PID::ControlValue() const {
  return - (p_error * Kp) - (d_error * Kd) - (i_error * Ki);
}

double PID::TotalError() {
  return 0;
}

