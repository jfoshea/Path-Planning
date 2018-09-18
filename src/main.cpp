#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "path_planner.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {

  uWS::Hub h;

  PathPlanner pp;

  pp.Init();

  h.onMessage([&pp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {
      auto s = hasData(data);
      if (s != "") 
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") 
        {
          // Main car's localization Data
          pp.car_x = j[1]["x"];
          pp.car_y = j[1]["y"];
          pp.car_s = j[1]["s"];
          pp.car_d = j[1]["d"];
          pp.car_yaw = j[1]["yaw"];
          pp.car_speed = j[1]["speed"];
          
          pp.set_previous_path_x(j[1]["previous_path_x"]);
          pp.set_previous_path_y(j[1]["previous_path_y"]);
          
          pp.end_path_s = j[1]["end_path_s"];
          pp.end_path_d = j[1]["end_path_d"];
          
          pp.set_sensor_fusion(j[1]["sensor_fusion"]);
          pp.prev_size = pp.previous_path_x.size();
          
          if( pp.prev_size > 0 ) 
          {
            pp.car_s = pp.end_path_s;
          }
          
          pp.Navigate( pp.current_lane );
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          pp.UpdatePath( next_x_vals, next_y_vals, pp.current_lane );
          
          json msgJson;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
          
          auto msg = "42[\"control\","+ msgJson.dump()+"]";
          
          //this_thread::sleep_for(chrono::milliseconds(1000));
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
