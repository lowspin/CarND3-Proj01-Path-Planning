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
    waypoints_upsampled.x.push_back(spline_x((i/spline_samples)*max_s));
    waypoints_upsampled.y.push_back(spline_y((i/spline_samples)*max_s));
    waypoints_upsampled.s.push_back((i/spline_samples)*max_s);
  }
}

void Path::plan_target_sd() {
  target_SD.s = 0;
  target_SD.d = 0;
}
