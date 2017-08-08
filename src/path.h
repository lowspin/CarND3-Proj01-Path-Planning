#ifndef path_h
#define path_h

#include <vector>
#include <chrono>

using namespace std;

class Path {
public:
	Path();
	virtual ~Path();

	chrono::high_resolution_clock::time_point start_time, current_time;

	struct MAP {
		vector<double> waypoints_s_upsampled = {};
		vector<double> waypoints_x_upsampled = {};
		vector<double> waypoints_y_upsampled = {};
	};

	void init();

};

#endif // path_h
