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

// latency and Lf
double latency_dt = 0.1;
const double Lf = 2.67;

// for converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// checks if the SocketIO event has JSON data.
// if there is data the JSON object in string format will be returned,
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

// evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// fit a polynomial.
// adapted from
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
    // the 4 signifies a websocket message
    // the 2 signifies a websocket event
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
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          // transform reference trajectory points to vehicle coordinate system
          for(int i=0;i<ptsx.size();i++){
        	  double shift_x = ptsx[i]-px;
        	  double shift_y = ptsy[i]-py;

        	  ptsx[i] = shift_x*cos(psi)+shift_y*sin(psi);
        	  ptsy[i] = -shift_x*sin(psi)+shift_y*cos(psi);
          }

          Eigen::VectorXd ptsx_(6);
          Eigen::VectorXd ptsy_(6);
          ptsx_ << ptsx[0], ptsx[1], ptsx[2], ptsx[3], ptsx[4], ptsx[5];
          ptsy_ << ptsy[0], ptsy[1], ptsy[2], ptsy[3], ptsy[4], ptsy[5];

          // calculate polynomial
          auto coeffs = polyfit(ptsx_, ptsy_, 3);
          // calculate the cross track error (y=0 because of transformation to vehicle coordinates)
          double cte = polyeval(coeffs, 0);
          // calculate the orientation error (x=0 because of transformation to vehicle coordinates)
          double epsi = psi - atan(coeffs[1]);

          // predict state 100ms later
          px = v * cos(psi) * latency_dt;
          py = v * sin(psi) * latency_dt;
          psi = v * steer_value / Lf * latency_dt;
          v = v + throttle_value * latency_dt;
          cte = cte + v * sin(epsi) * latency_dt;
          epsi = epsi + v * steer_value / Lf * latency_dt;

          // define state vector to be sent to MPC
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // predict trajectory and calculate actuator inputs
          auto vars = mpc.Solve(state, coeffs);

          // obtained actuator inputs
          steer_value = vars[4];
          throttle_value = vars[5];

          json msgJson;
          // actuator inputs sent to simulator (conversion to rad)
          msgJson["steering_angle"] = steer_value/deg2rad(25);
          msgJson["throttle"] = throttle_value;

          // display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for(int i=0;i<vars[6];i++){
             mpc_x_vals.push_back(vars[7 + i]);
             mpc_y_vals.push_back(vars[7 + vars[6] + i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i=0;i<vars[6];i++){
             next_x_vals.push_back(i*2);
             next_y_vals.push_back(coeffs[0] + coeffs[1] * i*2 + coeffs[2] * i*2 * i*2 + coeffs[3] * i*2 * i*2 * i*2);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

          // latency of 100ms
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {

        // manual driving
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
