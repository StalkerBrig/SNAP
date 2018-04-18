/*
	ALL THE DISTANCE UNITS HERE ARE IN METERS!
	Speed would be meters/second
	For x axis:
		positive = forwards
		negative = backwards
	For y axis:
		postive = left
		negative = right
*/

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
#include "SNAP_rudi.h"

using namespace std;

const double DistAndDirec::INTERSECTION_THRESHOLD = 4.0;
const double DistAndDirec::WALL_THRESHOLD = 1.0;
//arbitrary value... probably something better could be used
double DistAndDirec::INTERSECTION_INDICATOR[2] = { 100000000000, -100000000000 };

//drone position (should only be one copy)
vector <double*> DistAndDirec::dpos; 
//which way the drone was moving (again, only ever one copy)
vector <char> DistAndDirec::ddirec;

void handleIntersection( DistAndDirec& intersection, char& current_direction, double* nia_position, double& mbd, double& mfd, double& mld, double& mrd ){
	cout << "INTERSECTION!!!!!" << endl;
			
	bool prev_intersection = false;

	char prev_direction = current_direction; 
			
	current_direction = intersection.direction;
			
	//looking to see if we are at a new intersection!
	if ( !IsValueInDpos(nia_position) )
	{
		cout << "????" << endl;
		DistAndDirec::ddirec.push_back('x');
		//If we reach an intersection, don't want to go down the way we came
		// when we come back to it
		DistAndDirec::ddirec.push_back(ReverseDirection(prev_direction));
	}
	else //we are at a previous intersection
	{
		prev_intersection = true;
	}
			
			
	//This means we need to go back through the other paths we did not take yet
	if( prev_intersection == true )
	{
		cout << "AHKKKKKKKK" << endl;
		//making temp ints for intersection maxes
		double imbd = mbd;
		double imfd = mfd;
		double imld = mld;
		double imrd = mrd;
		
		double idist[4] = {imbd, imfd, imld, imrd};
		
		//the idea behind this is that we go through all the previous route we
		// went to at this intersection and make their max 0 (temporarily)
		// when we do this, we know which new direction to go
		vector <char> :: reverse_iterator ddrit;
		for (ddrit = DistAndDirec::ddirec.rbegin(); ddrit != DistAndDirec::ddirec.rend() && (*ddrit) != 'x'; ddrit++)
		{
			cout << (*ddrit) << endl;
			ChangeDistance(*ddrit, 0, idist);
		}
				
		imbd = idist[0];
		imfd = idist[1];
		imld = idist[2];
		imrd = idist[3];
			
		/*
			Checking l and f here, since it will basically check all directions at the
			 intersection. left first since we are prioritizing: left, forward, right
		*/	
		current_direction = 'l';
		intersection = CheckForIntersections(imbd, imfd, imld, imrd, current_direction);
		if(intersection.direction == 'n')
		{
			current_direction = 'f';	
			intersection = CheckForIntersections(imbd, imfd, imld, imrd, current_direction);
		}

		current_direction = intersection.direction;
				
		if (current_direction == 'n')
		{
			cout << "THE DRONE HAS NOWHERE ELSE TO GO! NAVIGATION COMPLETE!" << endl;
			exit(0);
		}			

	}
			
	DistAndDirec::dpos.push_back(DistAndDirec::INTERSECTION_INDICATOR);
			
	//don't want to go back down this way when we travel back to the
	// intersection
	DistAndDirec::ddirec.push_back(current_direction);
}

/*
	This just makes sure the user is ready
*/
void ReadyToReadScan()
{
	char scan_ready;
	cout << "Ready for next scan (y for yes; e for exit)" << endl;
	bool scan_ready_loop = true;
	while(scan_ready_loop)
	{
		cin >> scan_ready;
		switch(scan_ready)
		{
			case 'y':
				scan_ready_loop = false;
				break;
			
			case 'e':
				exit(0);
				break;
			
			default:
				cout << "FOOL, DO NOT TEST ME!" << endl;
		}
	}
}

double **  ReadScanFile(string fn)
{
	ifstream fin;
	double x_info;
	double y_info;
	double z_info;
	
	double *node_info = new double[2];
	double *max_forward_x = NULL;
	double *max_backward_x = NULL;
	double *max_left_y = NULL;
	double *max_right_y = NULL;

	string info_line;

	string junk;
	double x_node_info;
	double y_node_info;

	fin.open(fn);
	
	
	//the first line is the "NODE" line, which should be discarded here	
	getline(fin, info_line);
	
	istringstream iss(info_line);
	
	iss >> junk;
	iss >> x_node_info;
	iss >> y_node_info;
	node_info[0] = x_node_info;
	node_info[1] = y_node_info;

	//getting each x and y pair of data
	while( getline(fin, info_line) )
	{
		//cout << info_line << endl;
		
		istringstream iss(info_line);

		iss >> x_info;
		iss >> y_info;
		iss >> z_info;
		
		
		if( x_info < 0)
		{
			max_backward_x = CheckMaxCoord(x_info, y_info, 1, max_backward_x);
		}
		if( x_info >= 0 )
		{
			max_forward_x = CheckMaxCoord(x_info, y_info, 1, max_forward_x);
		}	
		//TODO: THIS MIGHT BE BACKWARDS, need to check!!!!!!!!!!!!!!!!!!!!
		if( y_info >= 0 )
		{
			max_left_y = CheckMaxCoord(x_info, y_info, 0, max_left_y);
		}
		if( y_info < 0 )
		{
			max_right_y = CheckMaxCoord(x_info, y_info, 0, max_right_y);
		}
	} 

	/*
	cout << "MBX: " << max_backward_x[0] << " " << max_backward_x[1] << endl;
	cout << "MFX: " << max_forward_x[0] << " " << max_forward_x[1] << endl;
	cout << "MLY: " << max_left_y[0] << " " << max_left_y[1] << endl;
	cout << "MRY: " << max_right_y[0] << " " << max_right_y[1] << endl;
	*/
	fin.close();

	double ** max_arrays = new double*[5];
	
	max_arrays[0] = node_info;
	max_arrays[1] = max_backward_x;
	max_arrays[2] = max_forward_x;
	max_arrays[3] = max_left_y;
	max_arrays[4] = max_right_y;

	return max_arrays;	
	
}

/*
	check_x_or_y means which index it should be checking in the array:
		0 for x, 1 for y
*/
double * CheckMaxCoord(double x, double y, int check_x_or_y, double current_min[])
{
	double * new_min_array = new double[2];
	
	//if it is the first loop, need to set the min to current point
	if( current_min == NULL )
	{
		new_min_array[0] = x;
		new_min_array[1] = y;
		return new_min_array;
	}
	
	double check_min = current_min[check_x_or_y];

	new_min_array[0] = current_min[0];
	new_min_array[1] = current_min[1];

	//checking y
	if( check_x_or_y == 0 ) 
	{
		if( abs(check_min) > abs(x) )
		{
			new_min_array[0] = x;
			new_min_array[1] = y;
		}
	}
	//checking x
	if( check_x_or_y == 1 ) 
	{
		if( abs(check_min) > abs(y) )
		{
			new_min_array[0] = x;
			new_min_array[1] = y;
		}
	}
	delete[] current_min;
	
	return new_min_array;
}

DistAndDirec FindMaxDistance(double mbd, double mfd, double mld, double mrd)
{	
	DistAndDirec drone_dd_vals;
	
	/*
		I am putting mfd first because this would be the direction to move
		if there is a tie between the distances.
	*/
	double distance_array[4] = {mfd, mbd, mld, mrd};

	double cur_max = 0;
	int best_index = -1;
	
	char direction = 'n';

	for(int i = 0; i < 4; i++)
	{
		if (abs(distance_array[i]) > cur_max)
		{
			cur_max = abs(distance_array[i]);
			best_index = i;
		}
	}
	
	switch(best_index)
	{
		case 0:
			direction = 'f';
			break;
		case 1:
			direction = 'b';
			break;
		case 2:
			direction = 'l';
			break;
		case 3:
			direction = 'r';
			break;
		default:
			cout << "This should be an intersection." << endl;
			break;

	}
	
	drone_dd_vals.distance = cur_max;
	drone_dd_vals.direction = direction;

	return drone_dd_vals;
} 

/*
	cd = current direction
*/
DistAndDirec CheckForIntersections(double mbd, double mfd, double mld, double mrd, char cd)
{
	DistAndDirec drone_dd_vals;	
	DistAndDirec intersection_vals;	
	

	intersection_vals.distance = 0;
	intersection_vals.direction = 'n';
	
	//We check for f and b here, because we don't want to go in the opposite direction
	// and loop between the two largest values.
	if( cd == 'f' || cd == 'b' )
	{
		//0, 0 is because we only want to look for mld and mrd
		drone_dd_vals = FindMaxDistance(0, 0, mld, mrd);
		if (drone_dd_vals.distance >= DistAndDirec::INTERSECTION_THRESHOLD)
		{
			intersection_vals.distance = drone_dd_vals.distance;
			intersection_vals.direction = drone_dd_vals.direction;
		}
	}

	if( cd == 'l' || cd == 'r' )
	{
		//0, 0 is because we only want to look for mld and mrd
		drone_dd_vals = FindMaxDistance(mbd, mfd, 0, 0);
		if (drone_dd_vals.distance >= DistAndDirec::INTERSECTION_THRESHOLD)
		{
			intersection_vals.distance = drone_dd_vals.distance;
			intersection_vals.direction = drone_dd_vals.direction;
		}
	}

	return intersection_vals;
}

double GetDistance(double mbd, double mfd, double mld, double mrd, char cd)
{
	double current_distance;

	switch(cd)
	{
		case 'b':
			current_distance = abs(mbd);
			break;
		case 'f':
			current_distance = abs(mfd);
			break;
		case 'l':
			current_distance = abs(mld);
			break;
		case 'r':
			current_distance = abs(mrd);
			break;
		default:
			cout << "how" << endl;
			break;
	}	
	
	return current_distance;
}


/*
	cd = current_direction
*/
char ReverseDirection(char cd)
{
	//new direction
	char nd;

	switch(cd)
	{
		case 'f':
			nd = 'b';
			break;
		case 'b':
			nd = 'f';
			break;
		case 'l':
			nd = 'r';
			break;
		case 'r':
			nd = 'l';
			break;
		default:
			cout << "STOP MESSING THINGS UP!" << endl;
			break;
	}
	
	return nd;
}


/*
	cd = current_direction
	cv = changed value
*/
//void ChangeDistance(char cd, double cv, double *mbd, double *mfd, double *mld, double *mrd)
void ChangeDistance(char cd, double cv, double * dist)
{
	//new value
	double nv;

	switch(cd)
	{
		case 'f':
			dist[1] = cv;
			break;
		case 'b':
			dist[0] = cv;
			break;
		case 'l':
			dist[2] = cv;
			break;
		case 'r':
			dist[3] = cv;
			break;
		default:
			cout << "STOP MESSING THINGS UP!" << endl;
			break;
	}
	
	//cout << cv << (*mfd) << (*mbd) << (*mld) << (*mrd) << endl;	
}

bool IsValueInDpos(double *value)
{
	vector <double*> :: iterator dpit;
	for( dpit = DistAndDirec::dpos.begin(); dpit != DistAndDirec::dpos.end(); dpit++)
	{
		cout << (*dpit)[0] << " " << (*dpit)[1] << " " << value[0] << " " << value[1] << endl;
		if( (*dpit)[0] == value[0] && (*dpit)[1] == value[1] )
		{
			return true;
		}
	}
	return false;
}
