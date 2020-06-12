#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <fstream>
#include <boost/range/numeric.hpp>

// for convenience
using nlohmann::json;
using std::string;

// using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.o
   */

  double TARGET_SPEED = 40;
  pid.tune = false;

  if (TARGET_SPEED == 20) {
    pid.Init(0.795131, 0.000209, 3.089950);
  } else 
  if (TARGET_SPEED == 30) {
    pid.Init(0.187722,	0.001840,	1.005840);
  } else 
  if (TARGET_SPEED == 40) {
    pid.Init(0.096164, 0.005433, 0.938557);
  } else 
  if (TARGET_SPEED == 60) {
    pid.Init(0.032581, 0.006453,	0.625201);
  } else {
      TARGET_SPEED = 40; 
       pid.Init(0.096164, 0.005433, 0.938557);
    }
  // true if tuning pid
  std::cout << "Initial PID params: " << pid.Kp << "," << pid.Ki << "," << pid.Kd << "\n";
  pid.tune_iter = -1;
  pid.twiddle_iter_p = 0;
  pid.tune_tolerance = 0.1;
  int step = 0;
  double throttle = 0.75;
  pid.target_speed = TARGET_SPEED;
  std::ofstream logfile;
  std::ofstream summary;

  if (pid.tune == true) {
    logfile.open("../pid_log.csv", std::ios::trunc); // overwrite existing file
    // write header
    logfile << "iteration" << "," << "step" << ","
            << "cte" << "," << "speed" << ","
            << "Kp" << "," << "Ki" << "," << "Kd" << "\n";

    summary.open("../pid_summary.csv", std::ios::trunc); // overwrite existing file
    // write header
    summary << "iteration" << "," << "cum_err" << "," << "best_err" << ","
            << "sum(dp)" << ","<< "target speed" << ","
            << "Kp" << "," << "Ki" << "," << "Kd" << "\n";
  }

  h.onMessage([&pid, &step, &throttle, &logfile, &summary](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          const int TUNE_STEPS = 1000;
          const int SETTLE_STEPS = 100;

          pid.UpdateError(cte);
          steer_value = pid.SteeringAngle(1);

          // Log and twiddle below, if tuning:
          if (pid.tune == true) {
            pid.log_tune(logfile, cte, speed, step);
            if (step > SETTLE_STEPS) pid.cum_err += std::pow(cte, 2);

            if (step > TUNE_STEPS) {
              std::cout << "================================" << std::endl; 
              std::cout << "PID params: " << pid.Kp << "," << pid.Ki << "," << pid.Kd << "\n";
              step = 0;
              int tpi = pid.twiddle_iter_p;
              pid.twiddle(summary);
              std::cout << "iteration: " << pid.tune_iter << " | cumulative error: " << pid.cum_err << " | best error: " << pid.best_err << std::endl;
              std::cout << "sum(dp): " << boost::accumulate(pid.twiddle_dp, 0.) << " | par i: " << tpi << std::endl;
              pid.cum_err = 0;
              std::string msg("42[\"reset\", {}]");
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              if (boost::accumulate(pid.twiddle_dp, 0.) <= pid.tune_tolerance) pid.tune = false;

              logfile.flush();
              summary.flush();
            }
            step++;
          }

          // hold speed const for steer pid tune
          if ( speed > pid.target_speed) {
            throttle = 0.1;
          } else {
            throttle = 1; //0.9+ for 40+ mph
          }

          // =============================================================================
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
if (pid.tune == true) {
  logfile.close();
  summary.close();
}
}