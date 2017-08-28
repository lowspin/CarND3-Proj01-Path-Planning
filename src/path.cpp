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

#define _DEBUG_PRINT 1

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

  behavior_state = KEEP_LANE;
  my_next_lane = 1;
  my_target_lane = 1;
  speed_limit = 49.8;

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

  // re-initialize
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
    check_speed = sqrt((vx*vx)+(vy*vy));

    // if near track end-start loop boundary, adjust s
    double range_s = 200.0;
    check_s_now = traffic_now[i][5];
    if (car_s > max_s-range_s) { // near track end
      check_s_now += (check_s_now<range_s)? max_s : 0.0;
    } else if (car_s < range_s){
      check_s_now -= (check_s_now>max_s-range_s)? max_s : 0.0; // this will make some negative
    }

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

    //predict future gap in 1 sec
    lanescore[ln] += (lanescore[ln]<9999.9)? (obs_ahead_speed[ln]-car_speed)*1.0 : 0.0;

    // if(ln!=car_lane)
    // {
    //   // watch out for fast cars behind!
    //   if (obs_behind_dist[ln]<9999.9) {
    //     double time2close = (obs_behind_dist[ln]-car_speed)/obs_behind_speed[ln];
    //     if ((time2close>0.0)&&(time2close<0.5)) { // dist/speed = time
    //       lanescore[ln] = -9999.9; // behind car catches up within x secs.
    //     }
    //   }
    // }
  }

  if (_DEBUG_PRINT) { // _DEBUG_PRINT
    cout << "state [" << behavior_state << "] " << car_lane << "->" << my_target_lane << ", scores: | ";
    for(int ln=0; ln<3; ln++){
      cout << lanescore[ln] << " | ";
    }
    cout << " speed = " << car_speed << " / " << my_target_speed << endl;
  }

}

int Path::whichLane() {
  int targetlane = car_lane;
  double bestscore = -1;
  for (int ln=0; ln<3; ln++)
  {
    if ((lanescore[ln]>0)&&(lanescore[ln]>bestscore))
    {
      bestscore = lanescore[ln];
      targetlane = ln;
    }
  }

  // only change if difference is significant (prevent changing lanes back and forth)
  if ((bestscore<0)||(bestscore-lanescore[car_lane]<2.0))
    return car_lane;
  else
    return targetlane;
}

double Path::adjustSpeed(double speed) {
  double speed_mph = speed;

  // limit acceleration or deceleration
  if(car_speed>0.0) {
    if (speed_mph>car_speed)
      speed_mph = ( (speed_mph-car_speed) > 3.0 )? car_speed+3.0 : speed_mph;
    else
      speed_mph = ( (speed_mph-car_speed) < -3.0 )? car_speed-3.0 : speed_mph;
  }

  // else {
  //   if (speed_mph>car_speed)
  //     speed_mph = ( (speed_mph-car_speed) > 3.0 )? car_speed+2.0 : speed_mph;
  //   else
  //     speed_mph = ( (speed_mph-car_speed) < -3.0 )? car_speed-2.0 : speed_mph;
  // }

  // Final -  Hard limits
  speed_mph = (speed_mph<=0.0)? 0.0 : speed_mph; // speed cannot be zero
  speed_mph = (speed_mph>speed_limit)? speed_limit : speed_mph;

  return speed_mph;
}

void Path::behavior() {
  int targetlane = car_lane; // {0,1,2}
  double target_speed = 0.0;
  double speed_mph = my_target_speed;
  double check_speed;

  bool too_close = false;
  double closest_car_s;

  double avail_gap, safe_gap;
  double relspeed;
  bool SAFE2PASS, UNSAFE1_ownfront, UNSAFE2_targetfront, UNSAFE3_targetback;

  switch (behavior_state) {

    /* ------------------ */
    /* KEEP LANE
    /* ------------------ */
    case KEEP_LANE:

      if (obs_ahead_dist[car_lane]<30.0) {

        too_close = true;

        speed_mph = obs_ahead_speed[car_lane] * MPS2MPH; // match speed of car ahead
        if (obs_ahead_dist[car_lane]<3.0*ONECARLENGTH) { // keep a safe following distance
          speed_mph -= (car_speed>10.0)? 3.0 : 2.0;
        }

        // Shall we switch lane?
        targetlane = whichLane();
      }
      else
      {
        speed_mph += (car_speed<5.0)? 3.0 : 4.0;
      }

      // set speed
      my_target_speed = adjustSpeed(speed_mph);

      // set lane
      if (targetlane>car_lane) {
        my_target_lane = min(car_lane+1,2);
      }
      else if (targetlane<car_lane) {
        my_target_lane = max(car_lane-1,0);
      }
      else {
        my_target_lane = car_lane;
      }

      if (my_target_lane != car_lane) {
        behavior_state = PREP_LANE_CHANGE;
      }

      break;
      /* -------------------------------------------------------------*/

    /* ------------------ */
    /* PREPARE LANE_CHANGE
    /* ------------------ */
    case 	PREP_LANE_CHANGE:

      if(_DEBUG_PRINT) {
        cout << "\n"
        << "target_behind: " << obs_behind_dist[my_target_lane] << "m, " << obs_behind_speed[my_target_lane] * MPS2MPH << "Mph. \n"
        << "target_ahead : " << obs_ahead_dist[my_target_lane] << "m, " << obs_ahead_speed[my_target_lane] * MPS2MPH  << "Mph. \n"
        << "own_ahead    : " << obs_ahead_dist[car_lane] << "m, " << obs_ahead_speed[car_lane] * MPS2MPH  << "Mph. \n"
        << "My speed: " << car_speed << " / " << my_target_speed << " Mph.\n" << endl;
      }

      SAFE2PASS = true;
      UNSAFE1_ownfront = false;
      UNSAFE2_targetfront = false;
      UNSAFE3_targetback = false;

      // target lane - behind
      if (obs_behind_speed[my_target_lane]>0) {
        relspeed = obs_behind_speed[my_target_lane]-(car_speed*MPH2MPS);
        // avail_gap = (relspeed>0)?obs_behind_dist[my_target_lane]-(relspeed*0.5):obs_behind_dist[my_target_lane];
        avail_gap = obs_behind_dist[my_target_lane]-(relspeed*0.5);
        safe_gap = (car_speed<30)? 2.5*ONECARLENGTH : 4.0*ONECARLENGTH;
        if ((avail_gap<0) || (avail_gap < safe_gap)) {
          SAFE2PASS = false;
          UNSAFE3_targetback = true;
          cout << "Watch out - target lane behind. avail_gap = " << avail_gap << endl;
        }
      }

      // target lane - ahead
      if (obs_ahead_speed[my_target_lane]>0) {
        relspeed = (car_speed*MPH2MPS) - obs_ahead_speed[my_target_lane];
        // avail_gap = (relspeed>0)? obs_ahead_dist[my_target_lane]-(relspeed*0.5) : obs_ahead_dist[my_target_lane];
        avail_gap = obs_ahead_dist[my_target_lane]-(relspeed*0.5);
        safe_gap = (car_speed<30)? 2.0*ONECARLENGTH : 3.0*ONECARLENGTH;
        if ((avail_gap<0) || (avail_gap < safe_gap)) {
          SAFE2PASS = false;
          UNSAFE2_targetfront = true;
          cout << "Watch out - target lane ahead. avail_gap = " << avail_gap << endl;
        }
      }

      // own lane - ahead
      if (obs_ahead_speed[car_lane]>0) {
        relspeed = (car_speed*MPH2MPS) - obs_ahead_speed[car_lane];
        // avail_gap = (relspeed>0)? obs_ahead_dist[car_lane]-(relspeed*1.0) : obs_ahead_dist[car_lane];
        avail_gap = obs_ahead_dist[car_lane]-(relspeed*1.0);
        safe_gap = (car_speed<30)? 2.0*ONECARLENGTH : 3.0*ONECARLENGTH;
        if ((avail_gap<0) || (avail_gap < safe_gap)) {
          SAFE2PASS = false;
          UNSAFE1_ownfront = true;
          cout << "Watch out - own lane ahead. avail_gap = " << avail_gap << endl;
        }
      }

      // beware of other side cutting into same lane when changing to center lane
      if(my_target_lane == 1) {
        int otherlane = (car_lane==2)? 2 : 0;
        // cross traffic - behind
        if (obs_behind_speed[otherlane]>0) {
          relspeed = obs_behind_speed[otherlane]-(car_speed*MPH2MPS);
          if ( (relspeed>1.0) && (obs_behind_dist[otherlane]<1.0*ONECARLENGTH) ) {
            SAFE2PASS = false;
            cout << "WATCH OUT: CROSS FROM BEHIND!" << endl;
          }
        }
        // cross traffic - ahead
        if (obs_ahead_speed[otherlane]>0) {
          relspeed = obs_ahead_speed[otherlane]-(car_speed*MPH2MPS);
          if ( (abs(relspeed)<1.0) && (obs_ahead_dist[otherlane]<1.0*ONECARLENGTH) ) {
            SAFE2PASS = false;
            cout << "WATCH OUT: CROSS FROM AHEAD!" << endl;
          }
        }
      }

      if ( SAFE2PASS )
      {
        behavior_state =  LANE_CHANGE;
      }
      else
      {
        // adjust speed to match car ahead in my lane
        // if (obs_ahead_dist[car_lane]<3.0*ONECARLENGTH)
        //   // target_speed = 0.9*car_speed; //target_speed = obs_ahead_speed[car_lane];
        //   target_speed = my_target_speed - 3.0;

        // keep a safe following distance
        // if (obs_ahead_dist[car_lane]<30.0) {
        //   target_speed = obs_ahead_speed[car_lane]*MPS2MPH;
        //   if (obs_ahead_dist[car_lane]<3.0*ONECARLENGTH) { // too close
        //     target_speed -= 2.0; //(car_speed>10.0)? 3.0 : 2.0;
        //   }
        // }

        if (UNSAFE1_ownfront==true) {
          target_speed = car_speed - 3.0;
          if (obs_ahead_dist[car_lane]<3.0*ONECARLENGTH) { // too close
            target_speed -= 2.0; //(car_speed>10.0)? 3.0 : 2.0;
          }
        }
        else if (UNSAFE3_targetback==true && UNSAFE2_targetfront==false && UNSAFE1_ownfront==false) { // speed up and pass
          if ( (car_speed+5.0<speed_limit) &&
                (obs_behind_dist[my_target_lane]>1.5*ONECARLENGTH) &&
                (obs_behind_speed[my_target_lane]<=car_speed)) {
            my_target_speed = adjustSpeed(car_speed + 5.0);
            behavior_state = LANE_CHANGE;
            cout << "BEEP! BEEP!" << endl;
            break;
          }
        }
        else {
          // target_speed = (car_speed<5.0)? my_target_speed+3.0 : my_target_speed+4.0;
          target_speed = my_target_speed+5.0;
          // target_speed = my_target_speed;
        }

        // set speed
        my_target_speed = adjustSpeed(target_speed);
        cout << target_speed << " -> " << my_target_speed << endl;

        // Still wanna switch lane?
        targetlane = whichLane();
        if (targetlane>car_lane) {
          my_target_lane = min(car_lane+1,2);
        }
        else if (targetlane<car_lane) {
          my_target_lane = max(car_lane-1,0);
        }
        else {
          my_target_lane = car_lane;
        }
        behavior_state = (my_target_lane!=car_lane)? PREP_LANE_CHANGE : KEEP_LANE;

      }
      break;
      /* -------------------------------------------------------------*/

    /* ------------------ */
    /* LANE_CHANGE
    /* ------------------ */
    case LANE_CHANGE:

      my_next_lane = my_target_lane; // this is used in trajectory

      // if (car_speed+2.0 < speed_limit) // speed up to pass
        // target_speed = (car_speed>40)? car_speed+4.0 : car_speed+2.0;

      target_speed = car_speed + 3.0;
      my_target_speed = adjustSpeed(target_speed);

      if (lane_from_d(car_d)==my_target_lane) {
        behavior_state = KEEP_LANE;
      }

      break;
      /* -------------------------------------------------------------*/

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
  double ref_lane = my_next_lane;

  int prevpts; //path_size;
  if ((behavior_state == LANE_CHANGE)) { //}|| (abs(my_target_speed-car_speed)>3.0)){ //} || (my_target_speed<car_speed)){
    prevpts = min(40,path_size); //(car_speed>40)? 30 : 25;
  }else{
    prevpts = min(20,path_size);
  }

  // prevpts = (car_speed>40)? 30 : 25;
  // prevpts = (path_size<prevpts)?path_size:prevpts;

  if( prevpts<2 )
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
    pos_x = previous_path_x[prevpts-1];
    pos_y = previous_path_y[prevpts-1];
    prev_x = previous_path_x[prevpts-2];
    prev_y = previous_path_y[prevpts-2];

    angle = atan2(pos_y-prev_y, pos_x-prev_x);

    vec_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y);
    pos_s = vec_sd[0];
  }

  ptsx.push_back(prev_x);
  ptsx.push_back(pos_x);
  ptsy.push_back(prev_y);
  ptsy.push_back(pos_y);

  double check_interval = distance(prev_x,prev_y,pos_x,pos_y);
  cout << check_interval << " - ";

  vector<double> next_wp0;
  vector<double> next_wp1;
  vector<double> next_wp2;
  vector<double> next_wp3;
  vector<double> next_wp4;

  // double temp_s;
  //
  // if (behavior_state == LANE_CHANGE) {
  //   // take care of end/start boundary adjustments
  //   temp_s = (car_speed<30)? pos_s+30 : pos_s+30; //30;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp0 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  //
  //   temp_s = (car_speed<30)? pos_s+50 : pos_s+70; //pos_s+70;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp1 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  //
  //   temp_s = (car_speed<30)? pos_s+80 : pos_s+90; //pos_s+90;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp2 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  // } else {
  //   // take care of end/start boundary adjustments
  //   temp_s = (car_speed<30)? pos_s+30 : pos_s+40; //pos_s+30;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp0 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  //
  //   temp_s = (car_speed<30)? pos_s+50 : pos_s+70; //pos_s+75;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp1 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  //
  //   temp_s = (car_speed<30)? pos_s+80 : pos_s+90; //pos_s+90;
  //   temp_s -= (temp_s>max_s)? max_s : 0.0;
  //   next_wp2 = getXY(temp_s,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  // }

  next_wp0 = getXY(pos_s+30,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  next_wp1 = getXY(pos_s+50,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  next_wp2 = getXY(pos_s+70,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  next_wp3 = getXY(pos_s+90,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);
  next_wp4 = getXY(pos_s+110,(ref_lane*4)+2,map_waypoints_s,map_waypoints_x,map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);
  ptsx.push_back(next_wp3[0]);
  ptsx.push_back(next_wp4[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  ptsy.push_back(next_wp3[1]);
  ptsy.push_back(next_wp4[1]);

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
    // assume end/start boundary already taken care of
    planned_path.x.push_back(previous_path_x[i]);
    planned_path.y.push_back(previous_path_y[i]);
  }

  // break up spline points to match velocity
  // double target_x = 30.0;
  // double target_x = (car_speed<40)? 30 : 60; //50.0; //30.0;
  // double target_x = (behavior_state == LANE_CHANGE)? 40 : 30;
  double target_x = 30; //(behavior_state != KEEP_LANE)? 60 : 30;
  double target_y = s(target_x);
  double target_dist = sqrt( (target_x*target_x) + (target_y*target_y));

  double x_add_on = 0.0;
  double x_point = 0.0;
  double y_point = 0.0;

  // prev_x = (path_size>0) ? previous_path_x[prevpts-1] : 0.0;
  // prev_y = (path_size>0) ? previous_path_y[prevpts-1] : 0.0;
  // double prev_dist = -1.0;
  // double curr_dist = -1.0;
  double N = (target_dist/(.02*ref_vel*MPH2MPS));
  double req_delta_x = target_x/N;
  // cout << " (" << req_delta_x << ") ";
  //
  // double max_change_interval = 9.0 * 0.02 * 0.02;
  // int num_fillers = 0;
  // double delta_x;
  // if ( req_delta_x-check_interval > max_change_interval) { // acceleration
  //   delta_x = check_interval + max_change_interval;
  //   num_fillers = floor(req_delta_x / check_interval);
  // } else if ( check_interval-req_delta_x > max_change_interval) { // deceleration
  //   num_fillers = floor(check_interval / req_delta_x);
  //   delta_x = check_interval - max_change_interval;
  // } else {
  //   num_fillers = 0;
  //   delta_x = req_delta_x;
  // }

  for (int i=1; i<=50-prevpts; i++)
  {
    // x_point = x_add_on + (target_x/N);
    x_point = x_add_on + req_delta_x;
    // change interval gradually until target
    // if (num_fillers>0) {
    //   cout << "<[" << num_fillers << "]> ";
    //   // cout << x_point << " = " << x_add_on << " + " <<delta_x << endl;
    //   delta_x += max_change_interval;
    //   if (delta_x > req_delta_x) // overshot
    //     delta_x = req_delta_x;
    //   num_fillers--;
    // }

    y_point = s(x_point);

    x_add_on = x_point;

    // rotate back to normal
    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref*cos(angle) - y_ref*sin(angle);
    y_point = x_ref*sin(angle) + y_ref*cos(angle);

    x_point += pos_x;
    y_point += pos_y;

    planned_path.x.push_back(x_point);
    planned_path.y.push_back(y_point);
  }

}
