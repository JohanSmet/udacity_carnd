#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double p_Kp, double p_Ki, double p_Kd, bool p_twiddle = false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double p_cte);

  double ControlValue() const;

  /*
  * Calculate the total PID error.
  */
  double TotalError();

// helper functions
private:
  void PerformTwiddle();


// member variables
private:
  double m_errors[3];
  double m_coeffs[3];

  bool   m_twiddle;
  int    m_step;
  double m_best_error;
  double m_total_error;
  int    m_tw_param;
  int    m_tw_phase;
  double m_tw_delta[3];

};

#endif /* PID_H */
