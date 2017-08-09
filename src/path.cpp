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

void Path::init() {
  start_time = chrono::high_resolution_clock::now();
}

void Path::updateLocalData(double x,double y,double s,double d,double yaw,double speed){
  car_x = x;
	car_y = y;
	car_s = s;
	car_d = d;
	car_yaw = yaw;
	car_speed = speed;
}

void Path::Upsample_Waypoints(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, double max_s) {
  // tk::spline spline_x, spline_y;
  // spline_x.set_points(map_waypoints_s, map_waypoints_x);
  // spline_y.set_points(map_waypoints_s, map_waypoints_y);
  //
  // // upsample map points with spline.
  // int spline_samples = 12000; // in meters
  // for (size_t i = 0; i < spline_samples; ++i) {
  //   // waypoints_upsampled.x.push_back(spline_x((i/spline_samples)*max_s));
  //   // waypoints_upsampled.y.push_back(spline_y((i/spline_samples)*max_s));
  //   // waypoints_upsampled.s.push_back((i/spline_samples)*max_s);
  //   waypoints_upsampled.x.push_back(spline_x(i));
  //   waypoints_upsampled.y.push_back(spline_y(i));
  //   waypoints_upsampled.s.push_back(i);
  // }
}

void Path::plan_target_sd(vector<double> map_waypoints_x, vector<double> map_waypoints_y,
   vector<double> map_waypoints_s, vector<double> map_waypoints_dx, vector<double> map_waypoints_dy) {

  int targetlane = 2;

  double speed_mph = 50.0;
  target_SD.s = car_s + (1.0*speed_mph*1.6/1000/3600); // 1.0 sec ahead
  target_SD.d = (targetlane-1)*4.0+2.0;

  // vector<double> targetxy = getXY(target_s, (targetlane-1)*4.0+2.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  //
  // // double targetx = map_waypoints_x[index%181]+((targetlane-1)*4.0+2.0)*map_waypoints_dx[index%181];
  // // double targety = map_waypoints_y[index%181]+((targetlane-1)*4.0+2.0)*map_waypoints_dy[index%181];
  // double targetx = targetxy[0];
  // double targety = targetxy[1];
  // double theta =
  //
  // vector<double> targetsd = getFrenet( targetx, targety, theta, map_waypoints_x, map_waypoints_x);
  //
  // target_SD.s = targetsd[0]; //map_waypoints_s[closewaypoint];
  // target_SD.d = targetsd[1]; //map_waypoints_s[closewaypoint];
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

void Path::generate_trajectory(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s,
  vector<double> previous_path_x, vector<double> previous_path_y){

  //vector<double> targetxy = getXY(target_SD.s, target_SD.d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  double targetspeed_mps = 50.0 *1.6*1000/3600;

  vector<double> evaltimes;
  for (int i=0; i<50; i++){
    evaltimes.push_back(0.02*i); // each point is 20ms apart
  }

  // 1. trajectory s
  JMT_PARAMS jmt_s;
  jmt_s.start.push_back(car_s);
  jmt_s.start.push_back(car_speed);
  jmt_s.start.push_back(0.0);
  jmt_s.end.push_back(target_SD.s);
  jmt_s.end.push_back(targetspeed_mps);
  jmt_s.end.push_back(0.0);
  jmt_s.T = 1.0;

  vector<double> jmt_coeffs_s = JMT(jmt_s.start, jmt_s.end, jmt_s.T);
  vector<double> path_s = polyvals(jmt_coeffs_s, evaltimes);

  // 2. trajectory d
  JMT_PARAMS jmt_d;
  jmt_d.start.push_back(car_d);
  jmt_d.start.push_back(0.0);
  jmt_d.start.push_back(0.0);
  jmt_d.end.push_back(target_SD.d);
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
