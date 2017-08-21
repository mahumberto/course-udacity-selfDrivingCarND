#include <math.h>
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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Define global parameters
const float CONTROL_LATENCY = 0.1;
const float Lf = 2.67;

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

          // Unpack income telemetry data. Data structure detailed at DATA.md
          // DATA.md:
          // `ptsx` (Array<float>) - The global x positions of the waypoints.
          vector<double> ptsx = j[1]["ptsx"];
          // `ptsy` (Array<float>) - The global y positions of the waypoints.
          vector<double> ptsy = j[1]["ptsy"];
          // `x` (float) - The global x position of the vehicle.
          double px = j[1]["x"];
          // `y` (float) - The global y position of the vehicle.
          double py = j[1]["y"];
          // `psi` (float) - The orientation of the vehicle in **radians**
          // converted from the Unity format to the standard format expected
          // in most mathemetical functions
          double psi = j[1]["psi"];
          psi *= -1;
          // `speed` (float) - The current velocity in **mph**.
          double v = j[1]["speed"];
          // `steering_angle` (float) - The current steering angle in **radians**
          double steering_angle = j[1]["steering_angle"];
          steering_angle *= -1;
          // `throttle` (float) - The current throttle value [-1, 1].
          double throttle = j[1]["throttle"];

          // waypoints are provided in global based coordinates
          // translate waypoints to vehicle based coordinates
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());
          for (uint i=0; i<ptsx.size(); i++) {
            // move to vehicle 0,0 coordinate base
            double ptsx_car_base = ptsx[i] - px;
            double ptsy_car_base = ptsy[i] - py;
            // project to vehicle x,y orientation base
            ptsx_car(i) = (ptsx_car_base*cos(psi) - ptsy_car_base*sin(psi));
            ptsy_car(i) = (ptsx_car_base*sin(psi) + ptsy_car_base*cos(psi));
          }

          // find best polynomial order 3 to fit waypoints
          Eigen::VectorXd poly_coeff = polyfit(ptsx_car, ptsy_car, 3);

          // cross track error - evaluate polynomial for x=0
          double cte = polyeval(poly_coeff, 0);
          // orientation error
          double epsi = -atan(poly_coeff[1]);

          // correct control latency by using current state as predicted position
		  // after elapsed CONTROL_LATENCY time
          Eigen::VectorXd state(6);
          // px on t=CONTROL_LATENCY
          state(0) = 0.0 + v * cos (0.0) * CONTROL_LATENCY;
          // py on t=CONTROL_LATENCY
          state(1) = 0.0 + v * sin (0.0) * CONTROL_LATENCY;
          // psi on t=CONTROL_LATENCY
          state(2) = 0.0 + (v / Lf) * steering_angle * CONTROL_LATENCY;
          // v on t=CONTROL_LATENCY
          state(3) = v + throttle * CONTROL_LATENCY;
          // cte on t=CONTROL_LATENCY
          state(4) = cte + v * sin(epsi) * CONTROL_LATENCY;
          // epsi on t=CONTROL_LATENCY
          state(5) = epsi + (v / Lf) * steering_angle * CONTROL_LATENCY;

          /*
          * Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // use predicted state at t=CONTROL_LATENCY as input for the
          // model predictive controller
          vector<double> mpc_output = mpc.Solve(state, poly_coeff);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // invert controller output, to the psi orientation from the simulator
          msgJson["steering_angle"] = -(mpc_output[0]/deg2rad(25));
          msgJson["throttle"] = mpc_output[1];

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (uint i=2; i<mpc_output.size(); i++) {
            mpc_x_vals.push_back(mpc_output[i++]);
            mpc_y_vals.push_back(mpc_output[i]);
          }
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (uint i=0; i<50; i++) {
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(poly_coeff, i));
          }
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds((int)(CONTROL_LATENCY*1000)));
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
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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
