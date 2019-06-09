# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

#### Reflection
There are 2 main sections implemented in this project:
1. Vehicle and Lane Occupancy state
2. Trajectory generetation

#### Vehicle and Lane Occupancey State
The simulator provides the state of each vehicle through sensor fusion data. These data from sensor fusion lane center distance **d**, **vehicle_speed**, **vehicle_distance(s)** are used to determine **vehicle_lane**, **proximity to ego vehicle**, **is vehicle ahead or behind** of other vehiclle respectively. These following code identifies the **Vehicle Proximity State** and also **Lane Occupancy State**:

```cpp
enum LaneOccupancy { CLEAR, LEFT_LANE_OCCUPIED, RIGHT_LANE_OCCUPIED, CURR_LANE_OCCUPIED };

enum CarLaneStatus { CAR_ON_CURR_LANE, CAR_ON_LEFT_LANE, CAR_ON_RIGHT_LANE, CAR_ON_NON_ADJ_LANE };

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
```
The above states `set<VehicleOccupancyState>` are populated by the `getOtherVehicleProximityStateconst vector<vector<double>>& sensor_fusion,...)` method refer in `src/main.cpp`. The state of the vehicle and lane occupancy would determine the vehicle next lane position. The following highlights some of the important logic in decision making:
* The other vehicle is on the current lane `CURR_LANE_OCCUPIED` within 30m proximity and adjacent lanes are occupied as well then vehicle would start slowing down at **0.224 MPH** rate as discussed in Q & A video to prevent collision
* Either of the adjacent lanes are not occupied then vehicle would navigate towards that lane by considering the farther left and right lanes
* Finally the vehicle would accelerate in increment of **0.224 MPH** once the lane is avaiable and would reach the desired *49.5 MPH *

Below is the code for decision making logic described above:

```cpp
std::set<VehicleOccupancyState>::iterator currLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(), vehicleOccupancyStates.end(),
                                                                          FindVehicleStateByOccupiedLane(VehicleOccupancyState(CURR_LANE_OCCUPIED)));
if( currLaneVehOccuIter != vehicleOccupancyStates.end()){
  std::set<VehicleOccupancyState>::iterator leftLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(), vehicleOccupancyStates.end(),                                                                          
                                            FindVehicleStateByOccupiedLane(VehicleOccupancyState(LEFT_LANE_OCCUPIED)));
  std::set<VehicleOccupancyState>::iterator rightLaneVehOccuIter = std::find_if(vehicleOccupancyStates.begin(),   
  vehicleOccupancyStates.end(),                                                                                                                                                      
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
```


#### Trajectory generation
The trajectory path was generated using spline as discussed in the QA video. The general idea is to pick the non-travelled points from the previously generated path use them as 3 anchor points 30m apart based on the desired lane to be travelled. Also the vehicle curret point and the point before is used then using the spline library Y points correspond to the X points are generated as well. The following code provide details of the trajectory generation:
 ```cpp
 //Initial state use the car position as reference
 if(prev_size < 2) {
   //draw a tangent from car reference to find the previous x and y
   double prev_car_x = car_x - cos(car_yaw);
   double prev_car_y = car_y - sin(car_yaw);

   ptsx.push_back(prev_car_x);
   ptsx.push_back(car_x);
            
   ptsy.push_back(prev_car_y);
   ptsy.push_back(car_y);
 } else {//find the prev x and y points from the previous vehicle path   
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
 ```
 The following code computes 30 m apartu x, y points using frenet and spline:
 ```cpp
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
 ```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

