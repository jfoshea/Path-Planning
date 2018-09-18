#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <fstream>
#include <algorithm>
#include "json.hpp"

using namespace std;

typedef enum
{ 
  LeftLane = 0, 
  MiddleLane, 
  RightLane, 
  UnknownLane 
} LANES_T;

typedef enum
{ 
  SHIFT_LEFT = 0, 
  SHIFT_RIGHT, 
  UnknownAction
} LANE_ACTION_T;

class PathPlanner {
public:

  PathPlanner();
  virtual ~PathPlanner();
 
  // Constant Defines
  const double ACCELERATION_RATE = 0.224;
  const double DEACCELERATION_RATE = ( ACCELERATION_RATE * 8 );
  const double SPEED_LIMIT = 49.5;
  const double ALLOWED_DISTANCE = 30.0;
  
  int prev_size;
  double speed;
  LANES_T current_lane;

  // Waypoint Vectors
  vector<double> wp_x;
  vector<double> wp_y;
  vector<double> wp_s;
  vector<double> wp_dx;
  vector<double> wp_dy;

  // Main car's localization Data
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  double end_path_s;
  double end_path_d;
  vector<vector<double>> sensor_fusion;

  // Initialize the Path Planner.
  void Init();

  // Calculate Lane based on Sensor Fusion Data.
  LANES_T CalculateLane( const double d );

  // Change Lane based on current lane and action (left or right).
  void ChangeLane( const LANE_ACTION_T action, LANES_T &lane );

  // Helper function to update previous path x vector.
  void set_previous_path_x ( vector<double> v ) 
  {
    previous_path_x = v;
  }

  // Helper function to update previous path y vector.
  void set_previous_path_y ( vector<double> v ) 
  {
    previous_path_y = v;
  }

  // Helper function to update sensor fusion vector.
  void set_sensor_fusion ( vector<vector<double>> v ) 
  {
    sensor_fusion = v;
  }

  // Helper function for converting angle in degrees to radians.
  double DegToRad( const double x );

  // Helper function for converting angle in radians to degrees.
  double RadToDeg( const double x );

  // Computes the distance between a set of points
  double ComputeDistance( const double x1, const double y1, 
                          const double x2, const double y2 );
  

  // Transform from Frenet s,d coordinates to Cartesian x,y
  vector<double> FrenetToCartesian( const double s, const double d );

  // Calculate the closest waypoint 
  int ClosestWaypoint( const double x, const double y );
  
  // Determine the next waypoint 
  int NextWaypoint( const double x, const double y, const double theta );

  // Navigate lanes on the highway
  void Navigate( LANES_T &current_lane );

  // Update the trajectory path 
  void UpdatePath( vector<double> &next_x_vals,
                   vector<double> &next_y_vals,
                   LANES_T lane );
  
};

#endif /* PATH_PLANNER */
