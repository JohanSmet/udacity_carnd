#include "PID.h"

namespace {

enum COEFFICIENTS {
  C_P = 0,
  C_I = 1,
  C_D = 2
};

} // unnamed namespace

PID::PID() :  m_errors{0.0,0.0,0.0},
              m_coeffs{0.0,0.0,0.0} {
}

PID::~PID() {
}

void PID::Init(double p_Kp, double p_Ki, double p_Kd) {
  m_coeffs[C_P] = p_Kp;
  m_coeffs[C_I] = p_Ki;
  m_coeffs[C_D] = p_Kd;
}

void PID::UpdateError(double p_cte) {
  m_errors[C_D] = p_cte - m_errors[C_P];
  m_errors[C_P] = p_cte;
  m_errors[C_I] += p_cte;
}

double PID::ControlValue() const {
  return - m_errors[C_P] * m_coeffs[C_P]
         - m_errors[C_D] * m_coeffs[C_D]
         - m_errors[C_I] * m_coeffs[C_I];
}

double PID::TotalError() {
  return m_errors[C_P] + m_errors[C_I] + m_errors[C_D];
}

