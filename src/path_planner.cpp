#include <iostream>
#include <fstream>
#include "Eigen-3.3/Eigen/Dense"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "path_planner.h"
#include "spline.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

PathPlanner::PathPlanner() {}

PathPlanner::~PathPlanner() {}

//=============================================================================
//  @brief  Init()
//          Initialize Path Planner waypoints by loading highway map csv file 
//
//  @param  void
//
//  @return void 
//=============================================================================
void PathPlanner::Init( ) 
{
  prev_size = 0;
  speed = 0.0;
  current_lane = MiddleLane;

  // Load Highway Map
  string line;
  
  string map_file_ = "../data/highway_map.csv";
  ifstream in_map_( map_file_.c_str(), ifstream::in );
  
  while( getline( in_map_, line ) ) 
  {
  	istringstream iss( line );
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
  	this->wp_x.push_back( x );
  	this->wp_y.push_back( y );
  	this->wp_s.push_back( s );
  	this->wp_dx.push_back( d_x );
  	this->wp_dy.push_back( d_y );
  }
}

//=============================================================================
//  @brief  JMT()
//          Calculate Jerk Minimized Trajectory 
//
//  @param  start and end vectors 
//
//  @return jmt coefficients 
//=============================================================================
vector<double> PathPlanner::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    //return {1,2,3,4,5,6};

  MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

  return result;
}

//=============================================================================
//  @brief  CalculateLane()
//          Calculate lane from sensor fusion data
//
//  @param   sensor fusion d parameter 
//
//  @return calculated lane 
//=============================================================================
LANES_T PathPlanner::CalculateLane( const double d )
{
  LANES_T calculated_lane;

  if( d >= 0 && d < 4 ) 
  {
    calculated_lane = LeftLane;
  } 
  else if( d >= 4 && d < 8 ) 
  {
    calculated_lane = MiddleLane;
  } 
  else if( d >= 8 && d <= 12 ) 
  {
    calculated_lane = RightLane;
  } 
  else 
  {
    calculated_lane = UnknownLane;
  }

  return calculated_lane;
}

//=============================================================================
//  @brief  CalculateLaneOffset()
//          Calculate d offset for lane
//
//  @param   LANES_T lane 
//
//  @return d offset for lane 
//=============================================================================
double PathPlanner::CalculateLaneOffset( const LANES_T lane )
{
  double offset = 0.0;

  switch( lane )
  {
    case LeftLane:
    case MiddleLane:
    case RightLane:
        offset = ( lane * 4.0 ) + 2.0;
    default:
        offset = -1.0;
  }

  return offset;
}

//=============================================================================
//  @brief  ChangeLane()
//          Change lane left or right
//
//  @param  action: Shift left or right 
//  @param  lane: car lane 
//
//  @return void 
//=============================================================================
void PathPlanner::ChangeLane( const LANE_ACTION_T action, LANES_T &lane , queue<double> &lane_changes )
{
  double current_lane_offset, changed_lane_offset;

  switch( action )
  {
    case SHIFT_LEFT:
      if( lane == MiddleLane )
      {
        current_lane_offset = CalculateLaneOffset( lane );
        lane = LeftLane;
        changed_lane_offset = CalculateLaneOffset( lane );
        cout << "Collision Warning: Left Lane <-- Middle Lane " << endl; 
      }
      else if( lane == RightLane )
      {
        current_lane_offset = CalculateLaneOffset( lane );
        lane = MiddleLane;
        changed_lane_offset = CalculateLaneOffset( lane );
        cout << "Collision Warning: Middle Lane <-- Right Lane " << endl; 
      }
      break;

    case SHIFT_RIGHT:
      if( lane == LeftLane )
      {
        current_lane_offset = CalculateLaneOffset( lane );
        lane = MiddleLane;
        changed_lane_offset = CalculateLaneOffset( lane );
        cout << "Collision Warning: Left Lane --> Middle Lane " << endl; 
      }
      else if( lane == MiddleLane )
      {
        current_lane_offset = CalculateLaneOffset( lane );
        lane = RightLane;
        changed_lane_offset = CalculateLaneOffset( lane );
        cout << "Collision Warning: Middle Lane --> Right Lane " << endl; 
      }
      break;

    default:
      break;
  }

  vector<double> start_offset = { current_lane_offset, 0.0, 0.0 };
  vector<double> end_offset = { changed_lane_offset, 0.0, 0.0 };

  vector<double> jmt = JMT( start_offset, end_offset, T );

  for( double t = TIME_INCREMENT; t <= T; t += TIME_INCREMENT )
  {
    double offset = 0.0;
    for( int i = 0; i < jmt.size(); i++) 
    {
      offset += jmt[i] * pow( t, i );
    }

    lane_changes.push(offset);
  }
}

//=============================================================================
//  @brief  DegToRad()
//          Helper function to convert degrees to radians
//
//  @param  x:  Angle in Degrees      
//
//  @return  Angle in Radians
//=============================================================================
double PathPlanner::DegToRad( const double x ) 
{ 
  return x * M_PI / 180; 
}

//=============================================================================
//  @brief  RadToDeg()
//          Helper function to convert radians to degrees
//
//  @param  x:  Angle in Radians      
//
//  @return  Angle in Degrees 
//=============================================================================
double PathPlanner::RadToDeg( const double x ) 
{ 
  return x * 180 / M_PI; 
}

//=============================================================================
//  @brief  ComputeDistance()
//          Computes the distance between a set of points
//
//  @param  x1:     origin point x value
//  @param  y1:     origin point y value
//  @param  x2:     destination point x value
//  @param  y2:     destination point y value
//
//  @return         distance result
//=============================================================================
double PathPlanner::ComputeDistance( const double x1, const double y1,
                                     const double x2, const double y2)
{
  return sqrt( (x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1) );
}

//=============================================================================
//  @brief  FrenetToCartesian()
//          Transform from Frenet s,d coordinates to Cartesian x,y
//
//  @param  s:      Frenet s coordinate
//  @param  d:      Frenet d coordinate
//
//  @return x,y:    Cartesian x,y coordinates
//=============================================================================
vector<double> PathPlanner::FrenetToCartesian( double s, double d )
{
  int prev_wp = -1;

  while( s > wp_s[prev_wp+1] && ( prev_wp < (int)( wp_s.size()-1 ) ) )
  {
    prev_wp++;
  }
  
  int wp2 = (prev_wp + 1) % wp_x.size();
  
  double heading = atan2( ( wp_y[wp2]-wp_y[prev_wp] ), ( wp_x[wp2]-wp_x[prev_wp] ) );
  double seg_s = ( s - wp_s[ prev_wp ] );
  
  double seg_x = wp_x[ prev_wp ] + seg_s*cos( heading );
  double seg_y = wp_y[ prev_wp ] + seg_s*sin( heading );
  
  double perp_heading = heading-M_PI/2;
  
  double x = seg_x + d*cos( perp_heading );
  double y = seg_y + d*sin( perp_heading );
  
  return { x, y };
}

//=============================================================================
//  @brief  ClosestWaypoint()
//          Calculate the closest waypoint to current map vector
//
//  @param  x:      x coordinate
//  @param  y:      y coordinate
//
//  @return closest_waypoint
//=============================================================================
int PathPlanner::ClosestWaypoint( const double x, const double y )
{
  double closest_len = 100000;
  int closest_waypoint = 0;

  for( int i = 0; i < wp_x.size(); i++ )
  {
    double maps_x = wp_x[i];
    double maps_y = wp_y[i];
    double dist = ComputeDistance( x, y, maps_x, maps_y );

    if( dist < closest_len )
    {
      closest_len = dist;
      closest_waypoint = i;
    }
  }

  return closest_waypoint;
}

//=============================================================================
//  @brief  NextWaypoint()
//          Return the closest waypoint as next waypoint 
//
//  @param  x:      x coordinate
//  @param  y:      y coordinate
//  @param  y:      theta
//
//  @return closest_waypoint
//=============================================================================
int PathPlanner::NextWaypoint( const double x, const double y, const double theta)
{
  int closest_waypoint = ClosestWaypoint( x, y );

  double maps_x = wp_x[ closest_waypoint ];
  double maps_y = wp_y[ closest_waypoint ];

  double heading = atan2( ( maps_y-y ),( maps_x-x ) );

  double angle = abs( theta - heading );

  if( angle > M_PI/4 )
  {
    closest_waypoint++;
  }

  return closest_waypoint;
}

//=============================================================================
//  @brief  Navigate()
//          Navigate safe lane transistions given current_lane and a 
//          snapshot of the current sensor fusion data
//
//  @param  current_lane: Current car lane 
//
//  @return void 
//=============================================================================
void PathPlanner::Navigate( LANES_T &current_lane )
{
  bool collision_warning = false;
  bool car_detected_on_left = false;
  bool car_detected_on_right = false;
  
  for( int i = 0; i < sensor_fusion.size(); i++ ) 
  {
    float d = sensor_fusion[i][6];

    LANES_T calculated_lane = CalculateLane( d );

    if( calculated_lane == UnknownLane )
    {
      continue;
    }
  
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double check_speed = sqrt( vx*vx + vy*vy );
    double check_car_s = sensor_fusion[i][5];
    
    check_car_s += ( (double)prev_size * 0.02 * check_speed );
  
    ////////////////////////////////////////////////////////////////////////////
    //
  	// Identify the following detectors to avoid collisions:
    //
    // (1) collision_warning: 
    //      Used to moderate safe distance to a car in my lane
    //
    // (2) car_detected_on_left: 
    //      Used to make safe left lane change decisions
    //
    // (3) car_detected_on_right: 
    //      Used to make safe right lane change decisions
    //
    ////////////////////////////////////////////////////////////////////////////
  
    if( calculated_lane == current_lane )
    {
      collision_warning |= ( check_car_s > car_s ) && ( ( check_car_s - car_s ) < ALLOWED_DISTANCE );
    } 
    else if( ( calculated_lane == RightLane && current_lane == MiddleLane ) || ( calculated_lane == MiddleLane && current_lane == LeftLane ) )
    {
      car_detected_on_right |= ( ( car_s - ALLOWED_DISTANCE ) < check_car_s ) && ( ( car_s + ALLOWED_DISTANCE ) > check_car_s );
    } 
    else if( ( current_lane == MiddleLane && calculated_lane == LeftLane ) || ( current_lane == RightLane && calculated_lane == MiddleLane ) ) 
    {
      car_detected_on_left |= ( ( car_s - ALLOWED_DISTANCE ) < check_car_s ) && ( ( car_s + ALLOWED_DISTANCE)  > check_car_s );
    }
  }
  
  if( collision_warning ) 
  {
    ////////////////////////////////////////////////////////////////////////////
    //
    // Collision Avoidance Mechanisms: 
    //
    // (1) shift one lane to the right if in left or middle lanes and no cars 
    // are detected on the right lane.
    //
    // (2) shift one lane to the left if in right or middle lanes and no cars 
    // are detected on the left lane.
    //
    // (3) if a lane change is impossible in this moment of time the slow
    // down to avoid a collision. The deacceleration rate is higher than the
    // normal acceleration rate to help prevent sudden lane shifts with a car
    // close proximity in the new lane
    //
    ////////////////////////////////////////////////////////////////////////////
    
    if( ( car_detected_on_right == false ) && ( ( current_lane == LeftLane ) || ( current_lane == MiddleLane ) ) ) 
    {
      ChangeLane( SHIFT_RIGHT, current_lane, lane_changes );
    } 
    else if( ( car_detected_on_left == false ) && ( ( current_lane == MiddleLane ) || ( current_lane == RightLane ) ) )
    {
      ChangeLane( SHIFT_LEFT, current_lane, lane_changes );
    } 
    else 
    {
      speed -= DEACCELERATION_RATE;
      cout << "Collision Warning: Reducing speed " << speed << endl;
    }
  } 
  else 
  {
    if( speed < SPEED_LIMIT )
    {
      speed += ACCELERATION_RATE;
    }
  }
}

//=============================================================================
//  @brief  UpdatePath()
//          Update path Trajectory 
//
//  @param  next_x_vals: vector of x coords for trajectory 
//  @param  next_y_vals: vector of y coords for trajectory 
//
//  @return void 
//=============================================================================
void PathPlanner::UpdatePath( vector<double> &next_x_vals,
                              vector<double> &next_y_vals,
                              LANES_T lane )
{
  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x, ref_x_prev, ref_y, ref_y_prev, ref_yaw, prev_car_x, prev_car_y;

  if( prev_size < 2 )
  {
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = DegToRad( car_yaw );

    prev_car_x = car_x - cos( car_yaw );
    prev_car_y = car_y - sin( car_yaw );

    ptsx.push_back( prev_car_x );
    ptsx.push_back( car_x );
    ptsy.push_back( prev_car_y );
    ptsy.push_back( car_y );
  }
  else
  {
    ref_x = previous_path_x[ prev_size - 1 ];
    ref_x_prev = previous_path_x[ prev_size - 2 ];

    ref_y = previous_path_y[ prev_size - 1 ];
    ref_y_prev = previous_path_y[ prev_size - 2 ];

    ref_yaw = atan2( ref_y - ref_y_prev, ref_x - ref_x_prev );

    ptsx.push_back( ref_x_prev );
    ptsx.push_back( ref_x );
    ptsy.push_back( ref_y_prev );
    ptsy.push_back( ref_y );
  }
 
  // In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = FrenetToCartesian( (car_s + 30), (2 + 4 * lane) );
  vector<double> next_wp1 = FrenetToCartesian( (car_s + 60), (2 + 4 * lane) );
  vector<double> next_wp2 = FrenetToCartesian( (car_s + 90), (2 + 4 * lane) );

  ptsx.push_back( next_wp0[0] );
  ptsx.push_back( next_wp1[0] );
  ptsx.push_back( next_wp2[0] );

  ptsy.push_back( next_wp0[1] );
  ptsy.push_back( next_wp1[1] );
  ptsy.push_back( next_wp2[1] );

  for( int i = 0; i < ptsx.size(); i++ )
  {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = ( shift_x * cos( 0 - ref_yaw ) - shift_y * sin( 0 - ref_yaw ) );
    ptsy[i] = ( shift_x * sin( 0 - ref_yaw ) + shift_y * cos( 0 - ref_yaw ) );
  }

  // create a spline
  tk::spline spline_interpolation;

  // set (x, y) points to the spline
  spline_interpolation.set_points( ptsx, ptsy );

  // Start with all of the previous path points from last time
  for( int i = 0; i < prev_size; i++ )
  {
    next_x_vals.push_back( previous_path_x[i] );
    next_y_vals.push_back( previous_path_y[i] );
  }

  // Calculate spline points to travel at desired reference velocity
  double target_x = 30.0; // 30 meters ahead
  double target_y = spline_interpolation( target_x );
  double target_dist = sqrt( target_x*target_x + target_y*target_y );
  double x_add_on = 0;

  double offset1 = current_lane * 4 + 2;
  double offset2 = current_lane * 4 + 2;
	if (!lane_changes.empty()) 
  {
    offset1 = lane_changes.front();
    lane_changes.pop();
    if (!lane_changes.empty()) 
    {
      offset2 = lane_changes.front();
    }
  }

  double offset_velocity = abs( offset1 - offset2 );
  double ref_velocity;
  if (offset_velocity > speed) 
  {
    ref_velocity = 0;
  }
  else
  {
    ref_velocity = sqrt(speed*speed - offset_velocity*offset_velocity);
  }

  double N = ( target_dist / ( .02 * ref_velocity / 2.24 ) );
  double x_step = target_x / N;

  // Fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 ponts
  
  for( int i = 1; i <= 50 - prev_size; i++ ) 
  {
    double x_point = x_add_on + x_step;
    double y_point = spline_interpolation( x_point );
    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = ( x_ref * cos( ref_yaw ) - y_ref * sin( ref_yaw ) );
    y_point = ( x_ref * sin( ref_yaw ) + y_ref * cos( ref_yaw ) );
    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back( x_point );
    next_y_vals.push_back( y_point );
  }
}
