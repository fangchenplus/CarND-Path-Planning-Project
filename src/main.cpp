#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  int lane = 1; // three lanes labeled as 0, 1, and 2
  double ref_vel = 0.0; //mph
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // 4m lane width 
          // The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.
          
          
          // Step 1: Prediction          
          bool car_ahead = false; //if a car is in my way
          bool car_left = false;
          bool car_right = false;
          
          // find which lane I am at
          if (car_d <= 4) {
            lane = 0; //left lane
          }
          else if (car_d > 4 && car_d < 8 ) {
            lane = 1; //middle lane
          }
          else if (car_d >= 8) {
            lane = 2; //right lane
          }
          
          int my_lane = lane;
          
          int prev_size = previous_path_x.size();
          // cout << "prev_size" << prev_size << endl;
          // cout << "new_size" << 50-prev_size << endl;          
          //find ref_v to use
          double car_ahead_speed = 0.0; 
          
          for (int i = 0; i<sensor_fusion.size(); i++) {
            
            float d = sensor_fusion[i][6];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];  
            

            
            // check_car_s += (double)prev_size*0.02*check_speed; //predict the check_car future position if using previous points            
            //car is in my lane within 30 meters
            if (d<(2+4*lane+2) && d>(2+4*lane-2) && check_car_s > car_s && (check_car_s-car_s)<30) {
              //if check_car is in my waypoint
              car_ahead = true;
              car_ahead_speed = check_speed;
            }
            
            else if (d<(2+4*lane-2) && abs(check_car_s-car_s)<20) {
              car_left = true;
            }
            
            else if (d>(2+4*lane+2) && abs(check_car_s-car_s)<20) {
              car_right = true;
            }
            
            // car_ahead_speed = check_speed;
            
          }
          
          cout << "current lane: " << my_lane << "======target lane: " << lane << "======left: " << car_left << "======right: " << car_right  << "======ahead: " << car_ahead << endl;
          
          // Step 2: Behavior Planning
          if (car_ahead) {
            if (!car_left && lane > 0 && car_speed > 20) {
              lane -= 1;
            }
            else if (!car_right && lane < 2 && car_speed > 20) {
              lane += 1;
            }
            else if (car_ahead_speed < car_speed) {
              ref_vel -= 0.224/2; //0.224 = 5m/s
            } 
            else if (car_ahead_speed > car_speed + 0.224) {
              ref_vel += 0.224/2; //5m/s
            }
          }
          else if (ref_vel < 49.5) {
            ref_vel += 0.224*1.5;            
          }

          // Step 3: Trajecotry Generation
          vector<double> ptsx; //sparsed (x,y) points with 30m interval for interpolation
          vector<double> ptsy;
          
          //set the car coordicate reference point as where the car is or at the previous paths end point
          double ref_x = car_x; 
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
                                     
          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
            
          } else {
            //redefine reference state as previous path end point
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);   
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y); 
            
          }          
          
          
          vector<double> next_wp0 = getXY(car_s+45, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+90, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+135, 2+4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);   

          // change from map to vechile coordicates
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = shift_x*cos(0.0-ref_yaw) - shift_y*sin(0.0-ref_yaw);
            ptsy[i] = shift_x*sin(0.0-ref_yaw) + shift_y*cos(0.0-ref_yaw);
          }
          
          // break up with spline points to travel at desired car velocity          
          tk::spline s;
          s.set_points(ptsx,ptsy);
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0.0;
          
          for (int i = 0; i < prev_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          for (int i = 0; i < 50-prev_size; ++i) {    

            // pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
            // pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
            double N = target_dist/(0.02*ref_vel/2.24); //2.24 is from mile/hour to meter/second
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point; //x interval
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            // rotate back to map coordicates from vehicle coordinates
            x_point = ref_x + (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = ref_y + (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);            
          }

          // end

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
}