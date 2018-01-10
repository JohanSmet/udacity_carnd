#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "map.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

const int ANCHORPOINT_DELTA_S = 30;
const int NUM_PATH_POINTS = 50;
const double DELTA_T = 0.02;

double mph_to_mps(float vel_mph) {
	// conversion from miles/hour to meter/second
	//	1 mile per hour = 1.60934 km per hour = 0.447039 meter per second
	return vel_mph * 0.447039;
}

int desired_lane = 1;
double desired_speed = mph_to_mps(49);

void transform_to_car_coordinates(double ref_x, double ref_y, double ref_yaw,
                                  const vector<double> &in_x, const vector<double> &in_y,
                                  vector<double> &out_x, vector<double> &out_y) {
  auto cos_yaw = cos(-ref_yaw);
  auto sin_yaw = sin(-ref_yaw);

  for (size_t idx=0; idx < in_x.size(); ++idx) {
    auto dx = in_x[idx] - ref_x;
    auto dy = in_y[idx] - ref_y;

    out_x.push_back((dx * cos_yaw) - (dy * sin_yaw));
    out_y.push_back((dx * sin_yaw) + (dy * cos_yaw));
  }
}

void transform_to_map_coordinates(double ref_x, double ref_y, double ref_yaw,
                                  double in_x, double in_y,
                                  vector<double> &out_x, vector<double> &out_y) {
	auto cos_yaw = cos(ref_yaw);
	auto sin_yaw = sin(ref_yaw);

	out_x.push_back((in_x * cos_yaw) - (in_y * sin_yaw) + ref_x);
	out_y.push_back((in_x * sin_yaw) + (in_y * cos_yaw) + ref_y);
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  // Waypoint map to read from
  const char *map_file_ = "../data/highway_map.csv";

	Map map;
	map.load_from_file(map_file_);

  h.onMessage([&map](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

					// construct a list of 5 widely spaces waypoints as a rough path to follow
					vector<double> map_anchors_x, map_anchors_y;

					// use the position of the car as the reference frame for future calculations
					auto ref_x = car_x;
					auto ref_y = car_y;
					auto ref_yaw = deg2rad(car_yaw);
					auto ref_speed = mph_to_mps(car_speed);

					if (previous_path_x.size() < 2) {
						// not enough left of the previous path, start at the position of the car (at the angle of the car)
						map_anchors_x.push_back(ref_x - cos(ref_yaw));
						map_anchors_y.push_back(ref_y - sin(ref_yaw));

						map_anchors_x.push_back(ref_x);
						map_anchors_y.push_back(ref_y);
					} else {
						// start at the end of the previously computed path for smoothness
						int lst = previous_path_x.size() - 1;
						ref_x = previous_path_x[lst];
						ref_y = previous_path_y[lst];
						double prev_x = previous_path_x[lst- 1];
						double prev_y = previous_path_y[lst- 1];

						ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
						ref_speed = distance(prev_x, prev_y, ref_x, ref_y) / DELTA_T;

						map_anchors_x.push_back(prev_x);
						map_anchors_y.push_back(prev_y);

						map_anchors_x.push_back(previous_path_x[lst]);
						map_anchors_y.push_back(previous_path_y[lst]);
					}

					// add three more anchorpoints at even intervals (beyond the end of the last path)
					double d = Map::LANE_HALF_WIDTH + (desired_lane * Map::LANE_WIDTH);

					for (int idx = 0; idx < 3; ++idx) {
						auto anchor = map.getXY(car_s + ((idx+1) * ANCHORPOINT_DELTA_S), d);
						map_anchors_x.push_back(anchor[0]);
						map_anchors_y.push_back(anchor[1]);
					}

					// transform anchorpoints to the reference coordinate space
					vector<double> car_anchors_x, car_anchors_y;
					transform_to_car_coordinates(ref_x, ref_y, ref_yaw, map_anchors_x, map_anchors_y, car_anchors_x, car_anchors_y);

					// fit a spline to the anchorpoints
					tk::spline	path;
					path.set_points(car_anchors_x, car_anchors_y);

					// add the previous path points to the output path
					vector<double> next_x_vals = previous_path_x;
					vector<double> next_y_vals = previous_path_y;

					// fill out path
					double target_x = 30;
					double target_y = path(target_x);
					double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
					double new_x = 0, new_y = 0;

					for (int idx=1; idx < NUM_PATH_POINTS - previous_path_x.size(); ++idx) {
							
						// speed control
						if (ref_speed < desired_speed) {
							ref_speed += 3 * DELTA_T;
						}

						double N = target_dist / (DELTA_T * ref_speed);
						new_x = new_x + (target_x / N);
						new_y = path(new_x);

						// transform to map coordinates and add to output
						transform_to_map_coordinates(ref_x, ref_y, ref_yaw, new_x, new_y, next_x_vals, next_y_vals);
					}

					json msgJson;

					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\","+ msgJson.dump()+"]";

					//this_thread::sleep_for(chrono::milliseconds(1000));
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
