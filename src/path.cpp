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
  //cout << pi() << endl;
}

void Path::Upsample_Waypoints(vector<double> map_waypoints_x, vector<double> map_waypoints_y, vector<double> map_waypoints_s, double max_s) {
  tk::spline spline_x, spline_y;
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);

  // upsample map points with spline.
  int spline_samples = 12000; // in meters
  for (size_t i = 0; i < spline_samples; ++i) {
    // waypoints_upsampled.x.push_back(spline_x((i/spline_samples)*max_s));
    // waypoints_upsampled.y.push_back(spline_y((i/spline_samples)*max_s));
    // waypoints_upsampled.s.push_back((i/spline_samples)*max_s);
    waypoints_upsampled.x.push_back(spline_x(i));
    waypoints_upsampled.y.push_back(spline_y(i));
    waypoints_upsampled.s.push_back(i);
  }
}

void Path::plan_target_sd() {
  target_SD.s = 0;
  target_SD.d = 0;
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

void Path::generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y){
  if (previous_path_x.size()>0)
  {
    planned_path.x = previous_path_x;
    planned_path.y = previous_path_y;
  }
  else
  {
    for (int i=0; i<50; i++){
      planned_path.x.push_back(waypoints_upsampled.x[i]);
      planned_path.y.push_back(waypoints_upsampled.y[i]);
    }
  }
}
