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
  void Init(double p_Kp, double p_Ki, double p_Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double p_cte);

  double ControlValue() const;

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  double m_errors[3];
  double m_coeffs[3];

};

#endif /* PID_H */
