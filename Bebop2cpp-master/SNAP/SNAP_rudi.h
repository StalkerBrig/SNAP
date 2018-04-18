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

#define DRONE_IP                    "192.168.42.1"
#define DRONE_MAX_ALTITUDE          1.0
#define DRONE_MAX_HORIZONTAL_SPEED  0.3
#define DRONE_MAX_VERTICAL_SPEED    0.3
#define LAND_AFTER_LAST_WAYPOINT    true
#define CALIBRATION_FILE            "res/calib_bd2.xml"
#define HULLPROTECTIONON            true
#define LOOK_FOR_CHESSBOARD         false


class DistAndDirec
{
public:
	static const double INTERSECTION_THRESHOLD;
	static const double WALL_THRESHOLD;
	static double INTERSECTION_INDICATOR[2]; //if you try and make this const, the compiler will throw some ill-formed conversion errors...

	//drone position (should only be one copy)
	static vector <double*> dpos;

	//which way the drone was moving (again, only ever one copy)
	static vector <char> ddirec;

	double distance;
	char direction;	
};

DistAndDirec FindMaxDistance(double mbd, double mfd, double mld, double mrd);
DistAndDirec CheckForIntersections(double mbd, double mfd, double mld, double mrd, char cd);
void handleIntersection( DistAndDirec& intersection, char& current_direction, double* nia_position, double& mbd, double& mfd, double& mld, double& mrd );

bool IsValueInDpos(double *value);
void ReadyToReadScan();
double ** ReadScanFile(string fn);
double * CheckMaxCoord(double x, double y, int check_x_or_y, double current_min[]);
double GetDistance(double mbd, double mfd, double mld, double mrd, char cd);
char ReverseDirection(char cd);
//void ChangeDistance(char cd, double cv, double *mbd, double *mfd, double *mld, double *mrd);
void ChangeDistance(char cd, double cv, double * dist);


