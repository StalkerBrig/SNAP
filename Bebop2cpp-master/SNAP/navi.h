#pragma once

#include <vector>
#include "Drone.h"
#include <math.h>
#include <boost/asio/io_service.hpp>
#include <Fullnavdata.h>
#include <gnuplot_iostream.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace std;

/*This information is used for the Parrot Bebop 2*/
#define DRONE_IP                    "192.168.42.1"
#define DRONE_MAX_ALTITUDE          1.0
#define DRONE_MAX_HORIZONTAL_SPEED  0.3
#define DRONE_MAX_VERTICAL_SPEED    0.3
#define LAND_AFTER_LAST_WAYPOINT    true
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true
#define LOOK_FOR_CHESSBOARD         false


/*This class is used to keep track of distance and direction of a Lidar Scan point*/
class DistAndDirec
{
public:
	double distance;
	char direction;	
};

class Navi{
private:
	DistAndDirec FindMaxDistance(double bd, double fd, double ld, double rd);
	DistAndDirec CheckForIntersections(double bd, double fd, double ld, double rd, char cd);
	void handleIntersection();

	bool IsValueInDpos(double *value);
	void ReadyToReadScan();
	double ** ReadScanFile(string fn);
    double * CheckMaxCoord(double angle, double dist, char check_axis, double current_min[]);
    double GetDistance(char cd);
	char ReverseDirection(char cd);
	void ChangeDistance(char cd, double cv, double * dist);

	static const double INTERSECTION_THRESHOLD;
	static const double WALL_THRESHOLD;
	static double INTERSECTION_INDICATOR[2]; //if you try and make this const, the compiler will throw some ill-formed conversion errors...

	//drone position (should only be one copy); keeps track of what X Y coordinates the drone has been at
	vector <double*> dpos;

	//which way the drone was moving (again, only ever one copy)
	vector <char> ddirec;

	bool getting_info_loop;
	//node info array
	double ** nia;
	double *nia_position;
	
	//max backwards distance
	double mbd;
	//max forward distance
	double mfd;
	//max left distance
	double mld;
	//max right distance
	double mrd; 
	
	
	char current_direction;
	char initial_direction;	
	DistAndDirec drone_dd_vals;
	DistAndDirec intersection;
	double current_distance;

	bool currently_reversing;
	bool did_reverse;
    
public:
	Navi();
	void mainLoop( string file_name );

};

