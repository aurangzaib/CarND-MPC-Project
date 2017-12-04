#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
helper params2;

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
      string s = params2.hasData(sdata);
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
          auto coeffs = params2.polyfit(waypoints_x, waypoints_y, 3);

          // calculate the cross track error
          const double fx = params2.polyeval(coeffs, px);
          // The polynomial is fitted to waypoints in vehicle coordinate
          // therefore your py should also be in vehicle coordinate which is 0
          const double cte = fx - 0; // current_y - total_y

          // calculate the orientation error
          double psi_desired = params2.polyeval(coeffs, px, true);
          // psi is from global map coordinate
          // it should be 0 in vehicle coordinate
          const double epsi = 0 - psi_desired;

          // using global kinematic model
          const double px_    = v * params2.dt,
                       py_    = 0.0,
                       psi_   = v * (-steering_angle / params2.Lf) * params2.dt,
                       v_     = v + throttle * params2.dt,
                       cte_   = cte + v * sin(epsi) * params2.dt,
                       epsi_  = epsi + v * (-steering_angle / params2.Lf) * params2.dt;

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
          msgJson["steering_angle"] = steer_value;
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
            auto y = params2.polyeval(coeffs, x);
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
          this_thread::sleep_for(chrono::milliseconds(100));
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
