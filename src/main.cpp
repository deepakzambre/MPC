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

// default tuned parmaters
size_t N = 10;
double dt = 0.1;
double Lf = 2.67;
double ref_v = 50;
int cte_wt = 10;
int epsi_wt = 10;
int v_wt = 1;
int delta_wt = 1;
int a_wt = 1;
int delta_diff_wt = 1;
int a_diff_wt = 1;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi()
{
  return M_PI;
}

double deg2rad(double x)
{
  return x * pi() / 180;
}

double rad2deg(double x)
{
  return x * 180 / pi();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }

  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;

  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }

  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(
  Eigen::VectorXd xvals,
  Eigen::VectorXd yvals,
  int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main(int argc, char* argv[])
{
  uWS::Hub h;

  if (argc > 1)
  {
    N = atoi(argv[1]);
    dt = atof(argv[2]);
    ref_v = atof(argv[3]);
    cte_wt = atoi(argv[4]);
    epsi_wt = atoi(argv[5]);
    v_wt = atoi(argv[6]);
    delta_wt = atoi(argv[7]);
    a_wt = atoi(argv[8]);
    delta_diff_wt = atoi(argv[9]);
    a_diff_wt = atoi(argv[10]);
  }
  
  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];

          Eigen::VectorXd local_ptsx(ptsx.size());
          Eigen::VectorXd local_ptsy(ptsx.size());
          for (uint i = 0; i < ptsx.size(); i++)
          {
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;
            local_ptsx[i] = dx * cos(-1 * psi) - dy * sin(-1 * psi);
            local_ptsy[i] = dx * sin(-1 * psi) + dy * cos(-1 * psi);
          }

          Eigen::VectorXd fit_curve = polyfit(local_ptsx, local_ptsy, 3);
          double cte = polyeval(fit_curve, 0);
          double epsi = -1 * atan(fit_curve[1]);

          Eigen::VectorXd next_state(6);
          next_state <<
            v * dt,
            0.0,
            -1 * v * steer / Lf * dt,
            v + throttle * dt,
            cte + v * sin(epsi) * dt,
            epsi - v * steer / Lf * dt;
          auto solution = mpc.Solve(next_state, fit_curve);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = solution[0] / (deg2rad(25) * Lf);
          msgJson["throttle"] = solution[1];

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          mpc_x_vals.push_back(next_state[0]);
          mpc_y_vals.push_back(next_state[1]);
          for (uint i = 2; i < solution.size(); i+=2)
          {
            mpc_x_vals.push_back(solution[i]);
            mpc_y_vals.push_back(solution[i+1]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          double x_step = 2;
          for (int i = 1; i < 20; i++)
          {
            next_x_vals.push_back(x_step * i);
            next_y_vals.push_back(polyeval(fit_curve, x_step * i));
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
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
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
