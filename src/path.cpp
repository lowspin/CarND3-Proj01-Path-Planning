#include "path.h"
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

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
  my_next_lane = 1;
  my_target_lane = 1;
  speed_limit = 49.7;

  start_time = chrono::high_resolution_clock::now();
}

void Path::updateLocalData(double x,double y,double s,double d,double yaw,double speed){
  car_x = x;
	car_y = y;
	car_s = s;
	car_d = d;
  car_lane = lane_from_d(d);
	car_yaw = yaw;
	car_speed = speed; // Miles per hour
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

  // Update Lane Scores
  for(int ln=0; ln<3; ln++){
    //available space in front
    lanescore[ln] = obs_ahead_dist[ln];

    //predict future 1 sec
    lanescore[ln] += (lanescore[ln]<9999.9)? obs_ahead_speed[ln]*1.0 : 0.0;

    // watch out for fast cars behind
    if (obs_behind_dist[ln]<9999.9) {
      if (obs_behind_dist[ln]/obs_behind_speed[ln]<0.5) { // dist/speed = time
        lanescore[ln] = -1.0; // behind car catches up within x secs.
      }
    }
  }

  cout << behavior_state << " " << car_lane << "->" << my_target_lane << ", scores: | ";
  for(int ln=0; ln<3; ln++){
    cout << lanescore[ln] << " | ";
  }
  cout << endl;

}

int Path::whichLane() {
  int targetlane = car_lane;

  switch (car_lane) {
    case 0:
      targetlane = (lanescore[1]>lanescore[car_lane])? 1 : car_lane;
      break;
    case 1:
      if((lanescore[0]<0.0)&&(lanescore[2]<0.0))
        targetlane = car_lane;
      else {
        double test0, test2;
        if (abs(lanescore[0]-lanescore[2])>2.0) { // space difference > 10m
          cout << "MORE then 2.0m, ";
          test0 = lanescore[0]-lanescore[2]; // use available space
        } else {
          cout << "less then 2.0m, ";
          test0 = obs_ahead_speed[0]-obs_ahead_speed[2]; // use speed difference
        }
        if(test0>=0.0) {
          cout << "lane 0 better than lane 2" << endl;
          targetlane = (lanescore[0]>lanescore[car_lane])? 0 : car_lane;
        }
        else {
          cout << "lane 2 better than lane 0" << endl;
          targetlane = (lanescore[2]>lanescore[car_lane])? 2 : car_lane;
        }
      }
      break;
    case 2:
      targetlane = (lanescore[1]>lanescore[car_lane])? 1 : car_lane;
      break;
  }
  return targetlane;
}

void Path::behavior() {
  // purpose: decide target (s,d)

  int targetlane = car_lane; // {0,1,2}
  double target_speed = 0.0;
  double speed_mph = my_target_speed;
  double target_s = 50.0; //my_target_s;

  double check_speed;
  double onecarlength = 4.0;
  bool too_close = false;
  double closest_car_s;

  if (true) {//behavior_state == "KeepLane"){

    if (obs_ahead_dist[car_lane]<30.0) {
      // cout << "closest car = " << obs_ahead_dist[car_lane] << endl;
      too_close = true;

      speed_mph = obs_ahead_speed[car_lane] * mps2Mph; // match speed of car ahead
      if (obs_ahead_dist[car_lane]<10.0) { // keep a safe following distance
        speed_mph -= (car_speed>10.0)? 5.0 : 3.0;
      }

      // Shall we switch lane?
      targetlane = whichLane();
    }
    else
    {
      speed_mph += (car_speed<5.0)? 4.0 : 5.0;
    }

    // limit acceleration or deceleration
    if(car_speed>0.0) {
      speed_mph = ( (speed_mph-car_speed) > 5.0 )? car_speed+3.0 : speed_mph;
      speed_mph = ( (speed_mph-car_speed) < -5.0 )? car_speed-3.0 : speed_mph;
    }

  } // if (behavior_state == "KeepLane")

  speed_mph = (speed_mph<=0.0)? 1.0 : speed_mph; // speed cannot be zero
  speed_mph = (speed_mph>speed_limit)? speed_limit : speed_mph;

  my_target_s = car_s + speed_mph*Mph2mps*1.0; // 1 sec later
  my_target_lane = targetlane;
  my_target_speed = speed_mph;

  if (behavior_state == "KeepLane"){
    if (targetlane != car_lane) {
      behavior_state = "LaneChange";
    }
  }
  else if (behavior_state == "LaneChange"){
    my_next_lane = my_target_lane;
    double lanecenter = (4.0*my_next_lane) + 2.0;
    if(abs(car_d-lanecenter)<0.6){ // within +/- 1.0m of target lane center
      behavior_state = "KeepLane";
    }
  }

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
  vector<double> vec_sd;

  int path_size = previous_path_x.size();

  double ref_vel = my_target_speed;
  double ref_lane = my_target_lane;
  // double ref_lane = my_next_lane;

  int prevpts; //path_size;
  if ((behavior_state == "LaneChange") || (my_target_speed<car_speed)){
    prevpts = 25;
  }else{
    prevpts = 40; //path_size;
  }
  prevpts = (path_size<prevpts)?path_size:prevpts;

  // if( path_size<2 )
  if( prevpts<2 )
  {
    pos_x = car_x;
    pos_y = car_y;
    angle = deg2rad(car_yaw);
    prev_x = car_x - cos(angle);
    prev_y = car_y - sin(angle);

    //angle = atan2(pos_y-prev_y, pos_x-prev_x);

    // vec_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
    // pos_s = vec_sd[0];
    pos_s = car_s;
  }
  else
  {
    // pos_x = previous_path_x[path_size-endindex];
    // pos_y = previous_path_y[path_size-endindex];
    // prev_x = previous_path_x[path_size-endindex-1];
    // prev_y = previous_path_y[path_size-endindex-1];
    pos_x = previous_path_x[prevpts-1];
    pos_y = previous_path_y[prevpts-1];
    prev_x = previous_path_x[prevpts-2];
    prev_y = previous_path_y[prevpts-2];

    angle = atan2(pos_y-prev_y, pos_x-prev_x);

    vec_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
    pos_s = vec_sd[0];
    //pos_s = end_path_s;
  }

  //cout << "traj: pos_x " << pos_x << ", pos_y " << pos_y << ", pos_s " << pos_s << ", angle " << angle << endl;
  ptsx.push_back(prev_x);
  ptsx.push_back(pos_x);
  ptsy.push_back(prev_y);
  ptsy.push_back(pos_y);

  vector<double> next_wp0;
  vector<double> next_wp1;
  vector<double> next_wp2;
  //if (car_speed > 30.0) {
  if (behavior_state == "LaneChange"){
    next_wp0 = getXY(pos_s+30,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
    next_wp1 = getXY(pos_s+60,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
    next_wp2 = getXY(pos_s+90,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  } else {
    next_wp0 = getXY(pos_s+30,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
    next_wp1 = getXY(pos_s+75,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
    next_wp2 = getXY(pos_s+90,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  }

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

  // for(int i = 0; i < path_size; i++)
  for(int i = 0; i < prevpts; i++)
  {
    planned_path.x.push_back(previous_path_x[i]);
    planned_path.y.push_back(previous_path_y[i]);
  }

  // break up spline points to match velocity
  double target_x = 30.0; //30.0; - 0.05*pos_y
  double target_y = s(target_x);
  double target_dist = sqrt( (target_x*target_x) + (target_y*target_y));

  double x_add_on = 0.0;
  double x_point = 0.0;
  double y_point = 0.0;

  // prev_x = (path_size>0) ? previous_path_x[path_size-1] : 0.0;
  // prev_y = (path_size>0) ? previous_path_y[path_size-1] : 0.0;
  prev_x = (path_size>0) ? previous_path_x[prevpts-1] : 0.0;
  prev_y = (path_size>0) ? previous_path_y[prevpts-1] : 0.0;
  double prev_dist = -1.0;
  double curr_dist = -1.0;

  // for (int i=1; i<=50-path_size; i++)
  for (int i=1; i<=50-prevpts; i++)
  {
    double N = (target_dist/(.02*ref_vel/mps2Mph));

    x_point = x_add_on + (target_x/N);
    y_point = s(x_point);

    x_add_on = x_point;

    // rotate back to normal
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(angle) - y_ref*sin(angle);
    y_point = x_ref*sin(angle) + y_ref*cos(angle);

    x_point += pos_x;
    y_point += pos_y;

    //--- adjust downwards if exceed speed limit - each intervale should be max 50*0.02*mps2Mph = 0.447 meters
    // // cout << i << ": (" << prev_x << " " << prev_y << ") -> (" << x_point << " " << y_point << "): " << distance(prev_x,prev_y,x_point,y_point)*mps2Mph/0.02 << endl;
    // if (distance(prev_x,prev_y,x_point,y_point)<10.) { //((prev_x!=0.0) && (prev_y!=0.0)){
    //   //cout << distance(prev_x,prev_y,x_point,y_point)*mps2Mph/0.02 << endl;
    //   while (distance(prev_x,prev_y,x_point,y_point)>49.9*0.02*Mph2mps) {
    //     // cout << "adjust " << i << ": (" << prev_x << " " << prev_y << ") -> (" << x_point << " " << y_point << "): " << distance(prev_x,prev_y,x_point,y_point)*mps2Mph/0.02;
    //     x_point -= 0.01*cos(angle); // 50*0.02*mps2Mph = 0.447 meters
    //     y_point -= 0.01*sin(angle);
    //     // cout << " -> " << distance(prev_x,prev_y,x_point,y_point)*mps2Mph/0.02 << endl;
    //   }
    // }

    prev_x = x_point;
    prev_y = y_point;
    //--- end adjustment

    planned_path.x.push_back(x_point);
    planned_path.y.push_back(y_point);

  }

}
