#include "PID.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/range/numeric.hpp>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0;
  i_error = 0;
  d_error = 0;

  cum_err = 0;

  twiddle_p = {Kp, Ki, Kd};
  double fdp = 0.1;
  twiddle_dp = {fdp, fdp, fdp};
  twiddle_set_large_up = false;
  twiddle_set_large_down = false;
  twiddle_check_large_up = false;
  twiddle_check_large_down = false;

}

void PID::UpdateError(double cte) {
  d_error = cte - p_error; // diff to prev. cte
  i_error += cte;
  p_error = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return 0.0;  // TODO: Add your total error calc here!
}

double PID::SteeringAngle(double max_steer=0.25) {
  double steer = -Kp * p_error - Ki * i_error - Kd * d_error;
  return std::max(std::min(steer, max_steer), -max_steer);

}

void PID::log_tune(std::ofstream &logfile, double &cte, double &speed, int &step) {
  logfile << std::max(0,tune_iter) << "," << cum_err << ","
          << cte << "," << speed << ","
          << Kp << "," << Ki << "," << Kd << "\n";
}

void PID::log_summary(std::ofstream &summary) {
  summary << std::max(0,tune_iter) << "," << cum_err << "," << best_err << ","
          << boost::accumulate(twiddle_dp, 0.) << "," << target_speed << ","
          << Kp << "," << Ki << "," << Kd << "\n";
}

// Twiddle 'in-line', i.e. this is called after each single run, setting params for next run
void PID::twiddle(std::ofstream &summary) {
  if (tune_iter == -1) {
    tune_iter = 0;
    best_err = cum_err;
  } 
  if (twiddle_set_large_up == false) {
    twiddle_set_large_up = true;
    twiddle_p[twiddle_iter_p] += twiddle_p[twiddle_iter_p] * twiddle_dp[twiddle_iter_p];
  } else if (twiddle_check_large_up == false) {
      twiddle_check_large_up = true;
      if (cum_err < best_err) {
        twiddle_dp[twiddle_iter_p] *= 1.1;
      } else if (twiddle_set_large_down == false) {
          twiddle_set_large_down = true;
          twiddle_p[twiddle_iter_p] -= 2 * twiddle_p[twiddle_iter_p] *  twiddle_dp[twiddle_iter_p];
          }
    } else if (twiddle_check_large_down == false) {
        twiddle_check_large_down = true;
        if (cum_err < best_err) {
          twiddle_dp[twiddle_iter_p] *= 1.1;
        } else {
            twiddle_p[twiddle_iter_p] += twiddle_p[twiddle_iter_p] * twiddle_dp[twiddle_iter_p];
            twiddle_dp[twiddle_iter_p] *= 0.9;
          }
      }

  if (cum_err < best_err || (twiddle_check_large_down == true && twiddle_check_large_up == true)) {
    if (twiddle_iter_p < 2) {
      if (Ki == 0) { // skip I, for faster PD convergence
        twiddle_iter_p = 2;
      } else {
          twiddle_iter_p++;
        }
      } else {
        twiddle_iter_p = 0;
        tune_iter++;
      }
    if (cum_err < best_err) best_err = cum_err;
    twiddle_p[twiddle_iter_p] += twiddle_p[twiddle_iter_p] * twiddle_dp[twiddle_iter_p];
    twiddle_set_large_up = true;
    twiddle_set_large_down = false;
    twiddle_check_large_down = false;
    twiddle_check_large_up = false;
  }
  log_summary(summary);
  // stop updating PID params if twiddle is done
  if (boost::accumulate(twiddle_dp, 0.) > tune_tolerance) {
    Kp = twiddle_p[0];
    Ki = twiddle_p[1];
    Kd = twiddle_p[2];
  }

}