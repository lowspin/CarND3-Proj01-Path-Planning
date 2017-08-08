#ifndef PATH_H
#define PATH_H

#include <vector>
#include <chrono>

// for boundary conditions and time for JMT
struct jmt_params {
		std::vector<double> start;
		std::vector<double> end;
		double T;
};

class Path {
public:
	Path();
	virtual ~Path();

	std::chrono::high_resolution_clock::time_point start_time, current_time;

	// struct MAP {
	// 	std::vector<double> waypoints_s_upsampled = {};
	// 	std::vector<double> waypoints_x_upsampled = {};
	// 	std::vector<double> waypoints_y_upsampled = {};
	// };

	struct Path_SXY {
		std::vector<double> s;
		std::vector<double> x;
		std::vector<double> y;
	} waypoints_upsampled;

  struct Path_XY {
		std::vector<double> x;
		std::vector<double> y;
	} planned_path;

	struct Path_SD {
		std::vector<double> s;
		std::vector<double> d;
	};

	struct Point_SD {
		double s;
		double d;
	} target_SD;

	struct PreviousPath {
		Path_XY XY;
		Point_SD SD;
	};

	void init();

	void plan_target_sd();
	void Upsample_Waypoints(
		std::vector<double> map_waypoints_x,std::vector<double> map_waypoints_y,
		std::vector<double> map_waypoints_s, double max_s);

	// Jerk Minimizing Trajectory
	std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T);

	void generate_trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y);

};

#endif // path_h
