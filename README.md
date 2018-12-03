# Path Planner

## Overview 
This project implements a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic. The goal for the path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data

## Goals

In this project, our goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The path planner code will use the provided car's localization and sensor fusion data, and there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt

Each waypoint in the list contains [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Code Implementation
The code consists of main.cpp and a class implementation in path_planner.cpp.  Also included is spline.h which is a cubic spline interpolation implementation which can use splines instead of polynomials. The path planner class has an init function which loads the highway_map.csv file for the simulator. All relevant helper functions are made into class functions: `CalculateLane()`, `ChangeLane()`, `FrenetToCartesian()`, `ClosestWaypoint()`,  `NextWaypoint()`, plus other helper functions. The class functions `Navigate()`, and `UpdatePath()` are the main high level path_planner.cpp class functions called from main.cpp.

#### Lane Enumerated Types.
In order to make the code more readable I created two enumerated types for lanes and lane changing actions. These enumerated types are used throughout the code to easily calculate lanes, implement collision warning detectors, and lane changing functions.

```C++
typedef enum
{ 
  LeftLane = 0, 
  MiddleLane, 
  RightLane, 
  UnknownLane 
} LANES_T;
```
```C++
typedef enum
{ 
  SHIFT_LEFT = 0, 
  SHIFT_RIGHT, 
  UnknownAction
} LANE_ACTION_T;
```

As mentioned previously the two main class functions in path_planner.cpp are `Navigate()` and `UpdatePath()`. The prototypes for each class function are shown below.

```C++
void PathPlanner::Navigate( LANES_T &current_lane )
```
```C++
void PathPlanner::UpdatePath( vector<double> &next_x_vals,
                              vector<double> &next_y_vals,
                              LANES_T lane )
```

#### Navigate()

The `Navigate()` class function takes in the current lane and using the current sensor fusion data snapshot calculates a lane, and generates collision detectors for all surrounding lanes. This information is then used to navigate safe lane transitions while maintaining a speed close to 50Mph when safe to do so. The following detectors are generated from sensor fusion data and current lane to help in navigation:

* `collision_warning`: Used to moderate safe distance to a car in my lane.
* `car_detected_on_left`: Used to make safe left lane change decisions.
* `car_detected_on_right`: Used to make safe right lane change decisions.

Using the above detectors the `Navigate()` function determines safe lane transitions using the following collision avoidance criteria:

Collision Avoidance Mechanisms: 
1. Shift one lane to the right if the car is in the left or middle lanes and no cars are detected on the right lane.
2. Shift one lane to the left if the car is in the right or middle lanes and no cars are detected on the left lane.
3. If a lane change can not be made at this moment in time, the car needs to slow down to avoid a collision. The deacceleration rate is higher than the normal acceleration rate to help prevent collisions by sudden lane shifts of other cars in close proximity in the new lane.

Here are the prototypes for `CalculateLane()`, and `ChangeLane()`. The code can be found in path_planner.cpp.

```C++
LANES_T PathPlanner::CalculateLane( const double d )
```
```C++
void PathPlanner::ChangeLane( const LANE_ACTION_T action, LANES_T &lane )
```
Path planning decisions will monitor the following state changes. The state changes have a particular priority order for safe navigation:

1. If there is no car detected on the right side and the current_lane is either the left lane or middle lane, the `ChangeLane()` function is called to shift one lane to the right and avoid colliding with the car in front.
2. If there is no car detected on the left side and the current lane is either the middle or right lanes, the `ChangeLane()` function is called to shift one lane to the left and avoid colliding with the car in front.
3. If a lane change is not possible in this instance of time, the only alternative is to slow down. The deaccerleration rate is higher than the acceleration rate to avoid sudden lane changes by cars in other lanes.
4. The default lane is middle lane. When it safe to do so the car will return to the middle lane. The car will increase its speed until target speed is reached.



#### UpdatePath()
The `UpdatePath()` class function does the work of generating new waypoints which are generated from the current position of the car using the frenet coordinates. The current frenet coordinates of the car are then used and extrapolate the path for next 90 meters as illustrated below. The coordinates are then converted to global x,y coordinates and the to the local coordinate of the car. The cublic spline is then used to fit a spline on the waypoints. The points are merged to the waypoints on previous_path_x, previous_path_y by pushing them on the next_x_vals, next_y_vals vectors. A Jerk Minimizing Trajectory (JMT) is used generate smooth trajectories during lane changes, where the jerk is minimized by solving a quintic polynomial.

```C++
vector<double> next_wp0 = FrenetToCartesian( (car_s + 30), (2 + 4 * lane) );
vector<double> next_wp1 = FrenetToCartesian( (car_s + 60), (2 + 4 * lane) );
vector<double> next_wp2 = FrenetToCartesian( (car_s + 90), (2 + 4 * lane) );
```

```C++
// create a spline
tk::spline spline_interpolation;

// set (x, y) points to the spline
spline_interpolation.set_points( ptsx, ptsy );
```

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros

## Build Instructions 
1. Clone the Path Planner git repository
    ```  
    $ git clone https://github.com/jfoshea/Path-Planning.git
    ```
2. This project involves the Term 3 Simulator which can be downloaded here [link](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)

3. Build the project using cmake or using the scipts below 
    ```  
    $ ./clean.sh 
    $ ./build.sh 
    ```

4. Run the Path Planner
    1. Launch the simulator and select Path Planner 
    2. Use the run script to connect to the server and run the simulation 
    ```  
    $ ./run.sh
    ```
