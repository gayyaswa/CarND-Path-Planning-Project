#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

#define ENABLE_LOGGING false

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::set;

enum LaneOccupancy { CLEAR, LEFT_LANE_OCCUPIED, RIGHT_LANE_OCCUPIED, CURR_LANE_OCCUPIED };

enum CarLaneStatus { CAR_ON_CURR_LANE, CAR_ON_LEFT_LANE, CAR_ON_RIGHT_LANE, CAR_ON_NON_ADJ_LANE };

const std::string getCarLaneStatusString(const CarLaneStatus& currLaneStatus){
    if(currLaneStatus == CAR_ON_CURR_LANE) {
      return "CAR_ON_CURR_LANE";
    } else if(currLaneStatus == CAR_ON_LEFT_LANE) {
      return "CAR_ON_LEFT_LANE";
    } else if(currLaneStatus == CAR_ON_RIGHT_LANE) {
      return "CAR_ON_RIGHT_LANE";
    }
    return "CAR_ON_NON_ADJ_LANE";
}

const std::string getLaneOccupancyString(const LaneOccupancy& laneOccupancy) {
    if(laneOccupancy == CLEAR) {
      return "CLEAR";
    } else if( laneOccupancy == LEFT_LANE_OCCUPIED) {
      return "LEFT_LANE_OCCUPIED";
    } else if( laneOccupancy == RIGHT_LANE_OCCUPIED) {
      return "RIGHT_LANE_OCCUPIED";
    } 
    return "CURR_LANE_OCCUPIED";
}

struct VehicleOccupancyState{
    double speed;
    LaneOccupancy occupiedLane;
    VehicleOccupancyState(){
        speed = 0.0;
        occupiedLane = CLEAR;
    }
    VehicleOccupancyState( LaneOccupancy laneOccupancy){
      occupiedLane = laneOccupancy;
    }
    bool operator < (const VehicleOccupancyState& other) const
    {
      return occupiedLane - other.occupiedLane; 
    }
};

struct FindVehicleStateByOccupiedLane{
    FindVehicleStateByOccupiedLane(const VehicleOccupancyState& occupancyState){
        thisOccupancyState = occupancyState;
    }
    bool operator()(const VehicleOccupancyState& other) const
    {
      bool isEqual = thisOccupancyState.occupiedLane == other.occupiedLane;   
      return isEqual;
    }
    private:
    VehicleOccupancyState thisOccupancyState;
};

CarLaneStatus getCarLaneStatus(const int& curr_vehicle_lane, const int& other_vehicle_lane){
  int lane_diff = curr_vehicle_lane - other_vehicle_lane;
  if(lane_diff == -1){
    return CAR_ON_RIGHT_LANE;
  } else if( lane_diff == 1){
    return CAR_ON_LEFT_LANE;  
  } else if( lane_diff == 0){
    return CAR_ON_CURR_LANE; 
  } else {
    return CAR_ON_NON_ADJ_LANE;
  }
}

void getOtherVehicleProximityState( const vector<vector<double>>& sensor_fusion, const int& prev_size, const double& car_s, const int& curr_vehicle_lane
                                            , set<VehicleOccupancyState>& otherVehicleStates){
  //Iterate the sensor fusion list in order to determine any other
  //vehicle in our current lane
  for(int i = 0; i < sensor_fusion.size(); ++i) {
    //Get the other vehicle lane distance
    float d = sensor_fusion[i][6];
    //Check if this falls within our current vehicle distance
    //with the assumption the road has only 3 lanes on each travelling side
    int other_vehicle_lane = getLane(d);
    //Get the other vehicle speed from its vectors
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    
    CarLaneStatus otherCarLaneStatus = getCarLaneStatus(curr_vehicle_lane, other_vehicle_lane);

    if(otherCarLaneStatus != CAR_ON_NON_ADJ_LANE) {
      //Get the magnitude of the velocity from the vectors
      double other_vehicle_speed = sqrt(vx * vx + vy * vy);
      double other_vehicle_s = sensor_fusion[i][5];
      
      //Lets project the other vehicle distance s by using the previous point and other car speed converted to m/s.
      //This will help us determine if our vehicle would be closer to other car then some evasive action
      //needs to be taken.
      other_vehicle_s += ((double)prev_size * .02 * other_vehicle_speed);
      //If the project distance of the ahead vehicle is within 30m s then take evasive action.
      bool is_vehicle_ahead = other_vehicle_s > car_s;
      double proximity_dist = 0.0;
      if(is_vehicle_ahead) {
        proximity_dist = other_vehicle_s - car_s;
      } else {
        proximity_dist = car_s - other_vehicle_s;
      }
      
      VehicleOccupancyState otherVehicleState;
      otherVehicleState.speed = other_vehicle_speed;
      if(is_vehicle_ahead){
        if( otherCarLaneStatus == CAR_ON_CURR_LANE && proximity_dist < 30 ){
          otherVehicleState.occupiedLane = CURR_LANE_OCCUPIED;
          otherVehicleStates.insert(otherVehicleState);
        } else if( otherCarLaneStatus == CAR_ON_LEFT_LANE && proximity_dist < 15 ){
          otherVehicleState.occupiedLane = LEFT_LANE_OCCUPIED;
          otherVehicleStates.insert(otherVehicleState);
        } else if( otherCarLaneStatus == CAR_ON_RIGHT_LANE && proximity_dist < 15 ){
          otherVehicleState.occupiedLane = RIGHT_LANE_OCCUPIED;
          otherVehicleStates.insert(otherVehicleState);
        }
      } else if(otherCarLaneStatus != CAR_ON_NON_ADJ_LANE && otherCarLaneStatus != CAR_ON_CURR_LANE && proximity_dist < 15){
        if(otherCarLaneStatus == CAR_ON_LEFT_LANE){
          otherVehicleState.occupiedLane = LEFT_LANE_OCCUPIED;
          otherVehicleStates.insert(otherVehicleState);
        } else {
          otherVehicleState.occupiedLane = RIGHT_LANE_OCCUPIED;
          otherVehicleStates.insert(otherVehicleState);
        }
      }
    }
  }
}


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

  int curr_vehicle_lane = 1;
  double ref_velocity = 0.0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&curr_vehicle_lane,&ref_velocity]
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

          //Create points that are far apart say 30m and create a spline for those points
          //based on the previous vehicle path. Using the spline then we can generate next
          //sequential waypoints that vehicle can travel through.
          int prev_size = previous_path_x.size();
          
          //Lets avoid the vehicle collision by using sensor fusion data
          //If we have previous path let use that  for car_s
          if(prev_size > 0){
              car_s = end_path_s;
          }
          
          set<VehicleOccupancyState> vehicleOccupancyStates;
          getOtherVehicleProximityState(sensor_fusion, prev_size, car_s, curr_vehicle_lane, vehicleOccupancyStates); 
          
#if ENABLE_LOGGING
          std::cout<<"After Vehicle Proximity State: "<< curr_vehicle_lane<<std::endl;
          std::cout<<"Printing Vehicle Occupancy States"<<std::endl;
          for(VehicleOccupancyState currState: vehicleOccupancyStates){
              std::cout<<"Occupancy State:"<<currState.speed<<","<<getLaneOccupancyString(currState.occupiedLane)<<std::endl;
          }           
#endif
          std::set<VehicleOccupancyState>::iterator currLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(), vehicleOccupancyStates.end(),
                                                                          FindVehicleStateByOccupiedLane(VehicleOccupancyState(CURR_LANE_OCCUPIED)));
          if( currLaneVehOccuIter != vehicleOccupancyStates.end()){
              std::set<VehicleOccupancyState>::iterator leftLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(), vehicleOccupancyStates.end(),
                                                                          FindVehicleStateByOccupiedLane(VehicleOccupancyState(LEFT_LANE_OCCUPIED)));
              std::set<VehicleOccupancyState>::iterator rightLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(), vehicleOccupancyStates.end(),
                                                                          FindVehicleStateByOccupiedLane(VehicleOccupancyState(RIGHT_LANE_OCCUPIED)));
              if( curr_vehicle_lane > 0 && leftLaneVehOccuIter == vehicleOccupancyStates.end()){
                  curr_vehicle_lane -= 1;
              } else if( curr_vehicle_lane < 2 && rightLaneVehOccuIter == vehicleOccupancyStates.end()) {
                  curr_vehicle_lane += 1;
              } else {
                  ref_velocity -= .112;
#if ENABLE_LOGGING
                  std::cout<<"Velocity Decremented: "<< ref_velocity;
#endif
              }
          } else if (ref_velocity < 49.5) {
              ref_velocity += .224;
#if ENABLE_LOGGING			  
              std::cout<<"Velocity Incremented: "<< ref_velocity;
#endif
          }
#if ENABLE_LOGGING    
          std::cout<<"Current Velocity: "<< ref_velocity;
#endif		  
          
          vector<double> ptsx;
          vector<double> ptsy;

          //Need to identify reference points x, y, yaw
          //use the car current position as reference point if there is no previous path
          //from simulator.
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          //Initial state use the car position as reference
          if(prev_size < 2) {
            //draw a tangent from car reference to find the previous x and y
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }else {//find the prev x and y points from the previous vehicle path
          
            //Lets use previous vehicle position as new reference points
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            
            double prev_ref_x = previous_path_x[prev_size - 2];
            double prev_ref_y = previous_path_y[prev_size - 2];
            
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x );
            
            //Lets add this tangential points to x and y list
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);
          }
#if ENABLE_LOGGING          
          std::cout<<"Before trajectory generation : "<<curr_vehicle_lane<<std::endl;
#endif

          //Compute the next waypoints  30 m apart using frenet transform and get the x y coordinates
          //that correspond to the lane the vehicle would be traveling.
          vector<double> next_wp0 = getXY(car_s+30,getLaneCenterDist(curr_vehicle_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,getLaneCenterDist(curr_vehicle_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,getLaneCenterDist(curr_vehicle_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); i++ ){
            //shift car reference angle to 0 degrees
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }


          tk::spline s;
          s.set_points(ptsx,ptsy);

          //Variable that would hold the actual points for path planner
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //Lets add the points from previous which is still not travelled by the vehicle
          for(int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          //Lets define 30m horizon for the vehicle.
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
            
          double x_add_on = 0;

          for (int i = 1; i <= 50-previous_path_x.size(); i++) {
            
           //Let's compute the points in order for the vehicle to travel at the desired speed
           // Car visits each of those points .02s and car desire speed to m/s by 2.24  
           // Distance over speed would give us the desired N points           
           double N = (target_dist/(.02*ref_velocity/2.24));
           double x_point = x_add_on+(target_x)/N;
           double y_point = s(x_point);

           x_add_on = x_point;

           double x_ref = x_point;
           double y_ref = y_point;

           //rotate back to normal coordinates  
           x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
           y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

           x_point += ref_x;
           y_point += ref_y;

           next_x_vals.push_back(x_point);
           next_y_vals.push_back(y_point);
          }


          json msgJson;

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
