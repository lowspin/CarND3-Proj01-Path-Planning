#ifndef path_h
#define path_h

#include <chrono>

using namespace std;

class Path {
public:
	Path();
	virtual ~Path();

	chrono::high_resolution_clock::time_point start_time, current_time;
	void init();

};

#endif // path_h
