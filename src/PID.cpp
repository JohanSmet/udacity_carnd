#include "PID.h"

#include <limits>
#include <cmath>
#include <iostream>

namespace {

enum COEFFICIENTS {
  C_P = 0,
  C_I = 1,
  C_D = 2
};

enum TWIDDLE_PHASE {
  TWP_ADD = 0,
  TWP_SUBTRACT,
  TWP_DONE
};

const int TWIDDLE_STEPS_SETTLE = 100;
const int TWIDDLE_STEPS_EVAL = 1900;
const int TWIDDLE_STEPS_TOTAL = TWIDDLE_STEPS_SETTLE + TWIDDLE_STEPS_EVAL;

} // unnamed namespace

PID::PID() :  m_errors{0.0,0.0,0.0},
              m_coeffs{0.0,0.0,0.0},
              m_twiddle(false),
              m_step(0),
              m_best_error(std::numeric_limits<double>::max()),
              m_total_error(0.0),
              m_tw_param(0),
              m_tw_phase(TWP_ADD),
              m_tw_delta{0.0,0.0,0.0} {
}

PID::~PID() {
}

void PID::Init(double p_Kp, double p_Ki, double p_Kd, bool p_twiddle) {
  m_coeffs[C_P] = p_Kp;
  m_coeffs[C_I] = p_Ki;
  m_coeffs[C_D] = p_Kd;

  m_twiddle = p_twiddle;
  m_tw_delta[C_P] = p_Kp * 0.1;
  m_tw_delta[C_I] = p_Ki * 0.1;
  m_tw_delta[C_D] = p_Kd * 0.1;
}

void PID::UpdateError(double p_cte) {
  m_errors[C_D] = p_cte - m_errors[C_P];
  m_errors[C_P] = p_cte;
  m_errors[C_I] += p_cte;

  if (m_twiddle) {
    PerformTwiddle();
  }
}

double PID::ControlValue() const {
  return - m_errors[C_P] * m_coeffs[C_P]
         - m_errors[C_D] * m_coeffs[C_D]
         - m_errors[C_I] * m_coeffs[C_I];
}

double PID::TotalError() {
  return m_errors[C_P] + m_errors[C_I] + m_errors[C_D];
}

void PID::PerformTwiddle() {
  ++m_step;

  // update total error only during the evaluation phase
  if (m_step % TWIDDLE_STEPS_TOTAL > TWIDDLE_STEPS_SETTLE) {
    m_total_error += std::fabs(m_errors[C_P]);
  }

  // change the value of a parameter at the first step of the twiddle phase
  if (m_step % TWIDDLE_STEPS_TOTAL == 1) {
    switch (m_tw_phase) {
      case TWP_ADD :
        m_coeffs[m_tw_param] += m_tw_delta[m_tw_param];
        ++m_tw_phase;
        break;

      case TWP_SUBTRACT :
        // (also undo the addition of the previous step)
        m_coeffs[m_tw_param] -= 2 * m_tw_delta[m_tw_param];
        ++m_tw_phase;
        break;
    }

    m_total_error = 0;
  }

  // evaluate the change at the end of the evaluation phase
  if (m_step % TWIDDLE_STEPS_TOTAL == 0) {
    std::cout << "TWIDDLE: step = " << m_step << " total error = " << m_total_error;

    if (m_total_error < m_best_error) {
      std::cout << " -- BETER !" << std::endl;
      std::cout << " (Kp=" << m_coeffs[C_P] 
                << ", Ki=" << m_coeffs[C_I] 
                << ", Kd=" << m_coeffs[C_D] 
                << ")" << std::endl;

      // go to the next parameter
      m_best_error = m_total_error;
      m_tw_delta[m_tw_param] *= 1.1;
      m_tw_param = (m_tw_param + 1) % 3;
      m_tw_phase = TWP_ADD;
    } else {
      std::cout << " -- WORSE !" << std::endl;

      // reset if both addition and subtraction have been tried for this param
      if (m_tw_phase == TWP_DONE) {
        m_coeffs[m_tw_param] += m_tw_delta[m_tw_param];   // undo subtraction
        m_tw_delta[m_tw_param] *= 0.9;
        m_tw_param = (m_tw_param + 1) % 3;
        m_tw_phase = TWP_ADD;
      }
    }
  }
}
