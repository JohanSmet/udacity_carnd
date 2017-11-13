#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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
};

#endif /* PID_H */
