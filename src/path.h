#ifndef PATH_H
#define PATH_H

#include <vector>
#include <chrono>
#include <string>

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

// struct Point_SD {
// 	double s;
// 	double d;
// };

// struct PreviousPath {
// 	Path_XY XY;
// 	Point_SD SD;
// };

// for boundary conditions and time for JMT
// struct JMT_PARAMS {
// 	std::vector<double> start;
// 	std::vector<double> end;
// 	double T;
// };

enum BehaviorState {
	KEEP_LANE,
	PREP_LANE_CHANGE,
	LANE_CHANGE
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

	// Lane score
	double lanescore[3];

	//Planned Path;
	Path_XY planned_path;

	// Speed Limit
	double speed_limit;

	// Main car's localization Data
	double car_x;
	double car_y;
	double car_s;
	double car_d;
	double car_yaw;
	double car_speed;
	int car_lane;

	// Other cars location and speed
	std::vector< std::vector<double>> traffic_future;
	std::vector< std::vector<double>> traffic_now;
	double obs_ahead_dist[3]; // one for each lane
	double obs_ahead_speed[3]; // one for each lane
	double obs_ahead_yaw[3]; // one for each lane
	double obs_behind_dist[3]; // one for each lane
	double obs_behind_speed[3]; // one for each lane
	double obs_behind_yaw[3]; // one for each lane

	// Main car's target location and speed
	double my_target_s;
	int my_target_lane;
	int my_next_lane;
	double my_target_speed;

	// Behabior planning state
	// std::string behavior_state;
	BehaviorState behavior_state;

	void init(std::vector<double>map_x, std::vector<double>map_y,
		std::vector<double>map_s, std::vector<double>map_dx, std::vector<double>map_dy, double max_track_s);

	void updateLocalData(double x,double y,double s,double d,double yaw,double speed);

	int whichLane();
	void checkTrajectory(double startspeed);

	void prediction(std::vector< std::vector<double>> sensor_fusion);
	void behavior();
	void trajectory(std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_path_s, double end_path_d);

};

#endif // path_h
