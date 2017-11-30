#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
hyper_params params2;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double steering_angle = j[1]["steering_angle"];
          const double throttle = j[1]["throttle"];
          /*
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          /****************************
           Transformation
           ****************************/

          // from: map coordinate system
          // to: vehicle coordinate system
          const auto N = ptsx.size();
          Eigen::VectorXd waypoints_x(N), waypoints_y(N);
          for (int loop = 0; loop < N; loop += 1) {
            // x and y terms
            double dx = ptsx[loop] - px;
            double dy = ptsy[loop] - py;
            // x and y way points
            waypoints_x(loop) = dx * cos(-psi) - dy * sin(-psi);
            waypoints_y(loop) = dx * sin(-psi) + dy * cos(-psi);
          }

          // find 3rd order polynomial coefficients
          auto coeffs = polyfit(waypoints_x, waypoints_y, 3);

          // calculate the cross track error
          const double fx = polyeval(coeffs, px);
          const double cte = fx - py; // current_y - total_y

          // calculate the orientation error
          const double psi_desired = atan(coeffs[1]);
          const double epsi = psi - psi_desired;

          // using global kinematic model
          const double px_ = v * params2.dt,
                       py_ = 0.0, 
                       psi_ = v * (steering_angle / params2.Lf) * params2.dt,
                       v_ = v + throttle * params2.dt,
                       cte_ = cte + v * sin(epsi) * params2.dt,
                       epsi_ = epsi + v * (steering_angle / params2.Lf) * params2.dt;

          // vehicle state vector
          Eigen::VectorXd state(6);
          state << px_, py_, psi_, v_, cte_, epsi_;

          // call optimization solver and update state
          auto vars = mpc.Solve(state, coeffs);

          // steering and acceleration
          double steer_value = vars[0];
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = - steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // the points in the simulator are connected by a Green line
          // x and y are returned from the optimization solver
          for (int loop = 2; loop < vars.size(); loop += 1) {
            // x values
            if (loop % 2 == 0) { mpc_x_vals.push_back(vars[loop]); }
            // y values
            else { mpc_y_vals.push_back(vars[loop]); }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // the points in the simulator are connected by a Yellow line
          // x -> values from 0 to 100 with interval of 5
          // y -> f(x) using polyeval
          for (int car_heading = 0; car_heading < 100; car_heading += 5) {
            auto x = double(car_heading);
            auto y = polyeval(coeffs, x);
            next_x_vals.push_back(x);
            next_y_vals.push_back(y);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          // this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
}
