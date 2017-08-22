#include "path.h"
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Path::Path(){}
Path::~Path(){}

void Path::init(vector<double> map_x,vector<double> map_y,vector<double> map_s,
  vector<double> map_dx,vector<double> map_dy,double max_track_s) {

  map_waypoints_x = map_x;
  map_waypoints_y = map_y;
  map_waypoints_s = map_s;
  map_waypoints_dx = map_dx;
  map_waypoints_dy = map_dy;

  max_s = max_track_s;
  my_target_speed = 0.0;

  for(int ln=0; ln<3; ln++){
    lanescore[ln]=0.0;
    obs_ahead_dist[ln] = 9999.9;
  	obs_ahead_speed[ln] = -1.0;
  	obs_behind_dist[ln] = 9999.9;
  	obs_behind_speed[ln] = -1.0;
  }

  behavior_state = "KeepLane";

  start_time = chrono::high_resolution_clock::now();
}

void Path::updateLocalData(double x,double y,double s,double d,double yaw,double speed){
  car_x = x;
	car_y = y;
	car_s = s;
	car_d = d;
  car_lane = lane_from_d(d);
	car_yaw = yaw;
	car_speed = speed;
  // if(car_speed<1.0){
  //   my_target_speed = 0.0;
  // }
}

void Path::updateLaneScore() {
  cout << " update lane scores: ";

  for(int ln=0; ln<3; ln++){

    //available space in front
    lanescore[ln] = obs_ahead_dist[ln];

    //predict future 1 sec
    lanescore[ln] += (lanescore[ln]<9999.9)? obs_ahead_speed[ln]*1.0 : 0.0;

    // watch out for fast cars behind
    if (obs_behind_dist[ln]/obs_behind_speed[ln]<1.0) { // dist/speed = time
      lanescore[ln] = -1.0; // behind car catchs up within 1 sec.
    }

  }

  for(int ln=0; ln<3; ln++){
    cout << lanescore[ln] << ", ";
  }
  cout << endl;

}

void Path::prediction(vector< vector<double>> sensor_fusion) {
  double d, vx, vy, check_speed, check_s_now, check_s_future;
  vector<double> obs_future;
  double time_horizon_sec = 1.0;
  int lane;


  traffic_now.clear();
  traffic_future.clear();

  traffic_now = sensor_fusion;

  double closest_infront_s[3],closest_behind_s[3];
  for(int ln=0; ln<3; ln++){
    obs_ahead_dist[ln] = 9999.9;
  	obs_ahead_speed[ln] = -1.0;
  	obs_behind_dist[ln] = 9999.9;
  	obs_behind_speed[ln] = -1.0;
    closest_infront_s[ln] = 9999.9;
    closest_behind_s[ln] = -9999.9;
  }

  for (int i=0; i<traffic_now.size(); i++){
    d = traffic_now[i][6];
    lane = lane_from_d(d);

    vx = traffic_now[i][3];
    vy = traffic_now[i][4];
    check_speed = sqrt(vx*vx+vy*vy);
    check_s_now = traffic_now[i][5];

    // future - 1sec
    check_s_future = check_s_now + 1.0*check_speed; // 1 sec in future
    obs_future.clear();
    obs_future.push_back(lane);
    obs_future.push_back(check_s_future);
    traffic_future.push_back(obs_future);

    // figure out closest obstacle cars in front and behind
    for(int ln=0; ln<3; ln++){
      if (lane == ln){

        // obs ahead
        if ( (check_s_now>car_s) && (check_s_now<closest_infront_s[ln]) ) {
          closest_infront_s[ln] = check_s_now;
          obs_ahead_dist[ln] = check_s_now - car_s;
          obs_ahead_speed[ln] = check_speed;
        }

        // obs behind
        else if ( (check_s_now<car_s) && (check_s_now>closest_behind_s[ln]) ) {
          closest_behind_s[ln] = check_s_now;
          obs_behind_dist[ln] = car_s - check_s_now;
          obs_behind_speed[ln] = check_speed;
        }

      } // if (lane == car_lane)
    } // for(int ln=0; lan<3; ln++)
  } // for traffic_now.size()

  updateLaneScore();
}

void Path::plan_target_sd(int targetlane, double target_s, double speed_mph) {
  my_target_s = target_s;
  my_target_lane = targetlane;
  my_target_speed = speed_mph;
  cout << "current speed=" << car_speed << ". Set lane=" << my_target_lane << ", set speed=" << my_target_speed << ", s=" << my_target_s << endl;
}

void Path::behavior() {
  // purpose: decide target (s,d)

  int targetlane; // {0,1,2}

  if (behavior_state == "KeepLane"){
    targetlane = car_lane;
  }

  double max_speed = 49.9;
  double target_speed = 0.0;
  double speed_mph = my_target_speed;
  double target_s = 50.0; //my_target_s;

  double check_speed;
  double onecarlength = 4.0;
  bool too_close = false;
  double closest_car_s;

  if (behavior_state == "KeepLane"){

    if (obs_ahead_dist[car_lane]<30.0) {
      // cout << "closest car = " << obs_ahead_dist[car_lane] << endl;
      too_close = true;

      speed_mph = obs_ahead_speed[car_lane] * mps2Mph; // match speed of car ahead
      if (obs_ahead_dist[car_lane]<10.0) { // keep a safe following distance
        speed_mph -= (car_speed>10.0)? 5.0 : 3.0;
      }

      // Shall we switch lane?
      switch (car_lane) {
        case 0:
          targetlane = (lanescore[1]>lanescore[car_lane])? 1 : car_lane;
          break;
        case 1:
          if((lanescore[0]<0.0)&&(lanescore[2]<0.0))
            targetlane = car_lane;
          else {
            if(lanescore[0]>=lanescore[2])
              targetlane = (lanescore[0]>lanescore[car_lane])? 0 : car_lane;
            else
              targetlane = (lanescore[2]>lanescore[car_lane])? 2 : car_lane;
            }
          break;
        case 2:
          targetlane = (lanescore[1]>lanescore[car_lane])? 1 : car_lane;
          break;

      }

    }
    else
    {
      speed_mph += (car_speed<5.0)? 4.0 : 5.0;
    }
    // cout << "1) Target speed=" << speed_mph << ". " << endl;

    // limit acceleration or deceleration
    if(car_speed>0.0) {
      speed_mph = ( (speed_mph-car_speed) > 5.0 )? car_speed+3.0 : speed_mph;
      speed_mph = ( (speed_mph-car_speed) < -5.0 )? car_speed-3.0 : speed_mph;
    }
    // cout << "2) Target speed=" << speed_mph << ". " << endl;

  }

  speed_mph = (speed_mph<=0.0)? 1.0 : speed_mph; // speed cannot be zero
  speed_mph = (speed_mph>max_speed)? max_speed : speed_mph;

  // cout << "3) Target speed=" << speed_mph << endl;

  plan_target_sd(targetlane, target_s, speed_mph);
}

// Jerk Minimizing Trajectory
vector<double> Path::JMT(vector< double> start, vector <double> end, double T){
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

void Path::trajectory(vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s, double end_path_d){
  double pos_x;
  double pos_y;
  double pos_s;
  double angle;
  double prev_x;
  double prev_y;
  vector<double> ptsx;
  vector<double> ptsy;

  int path_size = previous_path_x.size();

  double ref_vel = my_target_speed;
  double ref_lane = my_target_lane;

  if( path_size<3 )
  {
      pos_x = car_x;
      pos_y = car_y;
      angle = deg2rad(car_yaw);
      prev_x = car_x - cos(angle);
      prev_y = car_y - sin(angle);

      pos_s = car_s;
  }
  else
  {
      pos_x = previous_path_x[path_size-1];
      pos_y = previous_path_y[path_size-1];
      prev_x = previous_path_x[path_size-2];
      prev_y = previous_path_y[path_size-2];
      angle = atan2(pos_y-prev_y, pos_x-prev_x);

      pos_s = end_path_s;
  }

  //cout << "traj: pos_x " << pos_x << ", pos_y " << pos_y << ", pos_s " << pos_s << ", angle " << angle << endl;
  ptsx.push_back(prev_x);
  ptsx.push_back(pos_x);
  ptsy.push_back(prev_y);
  ptsy.push_back(pos_y);

  vector<double> next_wp0 = getXY(pos_s+30,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(pos_s+60,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(pos_s+90,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i=0; i<ptsx.size(); i++){
    //shift ref to zero
    double shift_x = ptsx[i] - pos_x;
    double shift_y = ptsy[i] - pos_y;

    ptsx[i] = shift_x*cos(0-angle) - shift_y*sin(0-angle);
    ptsy[i] = shift_x*sin(0-angle) + shift_y*cos(0-angle);
  }

  // create spline
  tk::spline s;

  //set (x,y) points to spline
  s.set_points(ptsx,ptsy);

  planned_path.x.clear();
  planned_path.y.clear();

  for(int i = 0; i < path_size; i++)
  {
    planned_path.x.push_back(previous_path_x[i]);
    planned_path.y.push_back(previous_path_y[i]);
  }

  // break up spline points to match velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt( (target_x*target_x) + (target_y*target_y));

  double x_add_on = 0.0;

  for (int i=1; i<=50-path_size; i++)
  {
    double N = (target_dist/(.02*ref_vel/2.24));
    double x_point = x_add_on + (target_x/N);
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal
    x_point = x_ref*cos(angle) - y_ref*sin(angle);
    y_point = x_ref*sin(angle) + y_ref*cos(angle);

    x_point += pos_x;
    y_point += pos_y;

    planned_path.x.push_back(x_point);
    planned_path.y.push_back(y_point);
  }
}

void Path::gen_trajectory(vector<double> previous_path_x, vector<double> previous_path_y){
  double ref_vel = my_target_speed;
  double ref_lane = my_target_lane;

  double pos_x;
  double pos_y;
  double pos_s;
  double angle;
  double prev_x;
  double prev_y;
  vector<double> ptss;
  vector<double> ptsx;
  vector<double> ptsy;

  // cout << "traj: " << car_x << ", " << car_y << endl;
  pos_x = car_x;
  pos_y = car_y;
  // angle = deg2rad(car_yaw);
  // prev_x = car_x - cos(angle);
  // prev_y = car_y - sin(angle);

  pos_s = car_s;
  ptss.push_back(pos_s);

  // ptsx.push_back(prev_x);
  ptsx.push_back(pos_x);
  // ptsy.push_back(prev_y);
  ptsy.push_back(pos_y);

  vector<double> next_wp0 = getXY(pos_s+20,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp1 = getXY(pos_s+40,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  vector<double> next_wp2 = getXY(pos_s+60,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptss.push_back(pos_s+20);
  ptss.push_back(pos_s+40);
  ptss.push_back(pos_s+60);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  // create spline
  tk::spline spx;
  tk::spline spy;

  //set (x,y) points to spline
  spx.set_points(ptss,ptsx);
  spy.set_points(ptss,ptsy);

  planned_path.x.clear();
  planned_path.y.clear();

  //TODO: better way to change velocity Minimizing jerk
  // double ds = my_target_speed/120;
  double speed_mps = my_target_speed * Mph2mps; // 50 nodes in 1 sec
  double ds = speed_mps * 1.0 / 50;

  for(int si = 0; si < 50; si++)
  {
    planned_path.x.push_back(spx(pos_s+(ds*si)));
    planned_path.y.push_back(spy(pos_s+(ds*si)));
    //cout << si << ": " << planned_path.x[si] << ", " << planned_path.y[si] << endl;
  }

}

void Path::generate_trajectory(vector<double> previous_path_x, vector<double> previous_path_y){

  // 1. trajectory s
  JMT_PARAMS jmt_s;
  jmt_s.start.push_back(car_s);
  jmt_s.start.push_back(car_speed*0.44704);
  jmt_s.start.push_back(0.0);
  jmt_s.end.push_back(my_target_s);
  jmt_s.end.push_back(my_target_speed*0.44704);
  jmt_s.end.push_back(0.0);
  jmt_s.T = 1.0;

  //vector<double> targetxy = getXY(target_SD.s, target_SD.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> evaltimes;
  for (int i=0; i<50; i++){
    evaltimes.push_back(0.02*i); // each point is 20ms apart
  }

  vector<double> jmt_coeffs_s = JMT(jmt_s.start, jmt_s.end, jmt_s.T);
  vector<double> path_s = polyvals(jmt_coeffs_s, evaltimes);

  // 2. trajectory d
  JMT_PARAMS jmt_d;
  jmt_d.start.push_back(car_d);
  jmt_d.start.push_back(0.0);
  jmt_d.start.push_back(0.0);
  jmt_d.end.push_back((my_target_lane*4.0)+2.0);
  jmt_d.end.push_back(0.0);
  jmt_d.end.push_back(0.0);
  jmt_d.T = 1.0;

  vector<double> jmt_coeffs_d = JMT(jmt_d.start, jmt_d.end, jmt_d.T);
  vector<double> path_d = polyvals(jmt_coeffs_d, evaltimes);

  planned_path.x.clear();
  planned_path.y.clear();

  if (1) //(previous_path_x.size()>0)
  {
    for (int i=0; i<50; i++){
      vector<double> pathxy = getXY(path_s[i],path_d[i],map_waypoints_s, map_waypoints_x, map_waypoints_y);
      planned_path.x.push_back(pathxy[0]);
      planned_path.y.push_back(pathxy[1]);
    }
  }
  else // initial state
  {
    // for (int i=0; i<50; i++){
    //   planned_path.x.push_back(waypoints_upsampled.x[i]);
    //   planned_path.y.push_back(waypoints_upsampled.y[i]);
    // }
  }
}
