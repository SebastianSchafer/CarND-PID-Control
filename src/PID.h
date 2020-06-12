#ifndef PID_H
#define PID_H

#include <vector>
#include <fstream>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  double SteeringAngle(double max_steer);
  void log_tune(std::ofstream &logfile, double &cte, double &speed, int &step);
  void log_summary(std::ofstream &summary);
  int tune_iter;
  bool tune;
  double tune_tolerance;
  double cum_err;
  double best_err;
  int twiddle_iter_p;
  bool twiddle_set_large_up;
  bool twiddle_set_large_down;
  bool twiddle_check_large_up;
  bool twiddle_check_large_down;
  std::vector<double> twiddle_p;
  std::vector<double> twiddle_dp;
  void twiddle(std::ofstream &summary);
  double target_speed;
  
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

};

#endif  // PID_H