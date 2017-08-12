#ifndef PATH_H
#define PATH_H

#include <vector>
#include <chrono>

// struct MAP {
// 	std::vector<double> waypoints_s_upsampled = {};
// 	std::vector<double> waypoints_x_upsampled = {};
// 	std::vector<double> waypoints_y_upsampled = {};
// };

// struct Path_SXY {
// 	std::vector<double> s;
// 	std::vector<double> x;
// 	std::vector<double> y;
// };

struct Path_XY {
	std::vector<double> x;
	std::vector<double> y;
};

// struct Path_SD {
// 	std::vector<double> s;
// 	std::vector<double> d;
// };

struct Point_SD {
	double s;
	double d;
};

// struct PreviousPath {
// 	Path_XY XY;
// 	Point_SD SD;
// };

// for boundary conditions and time for JMT
struct JMT_PARAMS {
	std::vector<double> start;
	std::vector<double> end;
	double T;
};


class Path {
public:
	Path();
	virtual ~Path();

	std::chrono::high_resolution_clock::time_point start_time, current_time;

	// track and  waypoints
	std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;
	double max_s;

	//Path_SXY waypoints_upsampled;
	Path_XY planned_path;
	Point_SD target_SD;

	// Main car's localization Data
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;

	void init(std::vector<double>map_x, std::vector<double>map_y,
		std::vector<double>map_s, std::vector<double>map_dx, std::vector<double>map_dy, double max_track_s);

	void updateLocalData(double x,double y,double s,double d,double yaw,double speed);

	void plan_target_sd(int targetlane, double speed_mph);
	void behavior();

	// void Upsample_Waypoints(
	// 	std::vector<double> map_waypoints_x,std::vector<double> map_waypoints_y,
	// 	std::vector<double> map_waypoints_s, double max_s);

	// Jerk Minimizing Trajectory
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);

	void trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_path_s, double end_path_d);
	void generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y);

};

#endif // path_h
