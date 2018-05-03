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
#include "navi.h"

using namespace std;

//If the distances on the perpendicular axis of the current direction the drone is going is greater than 4 meters,
// we assume that we are at an intersection
const double Navi::INTERSECTION_THRESHOLD = 4.0;
//If the LIDAR scan shows we are .8 meters away from the direction we are going, we assume we are at a wall
const double Navi::WALL_THRESHOLD = .8;
//arbitrary value... probably something better could be used; indicates there is an intersection
double Navi::INTERSECTION_INDICATOR[2] = { 100000000000, -100000000000 };

Navi::Navi(){
	getting_info_loop = true;
	//node info array
	nia = NULL;
	current_direction = 'n';
	initial_direction = 'n';
	
	currently_reversing = false;
	did_reverse = false;
}

/* mainLoop: This loop is used as the full navigation module;
              it will tell the user which direction to go,
              determine intersections, and walls.
              
    Parameters:
        file_name: the filename where the LIDAR data is scanned to.
                    The format is based on the scan.cpp module
*/
void Navi::mainLoop( string file_name ){
	while(getting_info_loop)
	{	
        //Want to wait til the LIDAR scan is in the file and ready to go
        ReadyToReadScan();
		
		//nia[0] = node coordinates
		//nia[1] = max backward distance vector
		//nia[2] = max forward distance vector
		//nia[3] = max left distance vector
		//nia[4] = max right distance vector
		nia = ReadScanFile(file_name);
		
		//the X Y coordinates on the plane. Z wasn't used here, so isn't included
		nia_position = nia[0];
		//max back distance
        mbd = nia[1][1];
        //max forward distance
		mfd = nia[2][1];
        //max left distance
		mld = nia[3][1];
        //max right distance
		mrd = nia[4][1];
		
        //We are seeing which way to start the drone; the farthest distance is the way it goes.
		if( initial_direction == 'n' ) //n indicates that there is no direcetion
		{
			drone_dd_vals = FindMaxDistance(mbd, mfd, mld, mrd);
			initial_direction = drone_dd_vals.direction;
			current_direction = initial_direction;
			cout << "Go: " << initial_direction << endl;	
			continue;
		}
		
		intersection = CheckForIntersections(mbd, mfd, mld, mrd, current_direction);		
		
		// n indicates no value was set, thus it isn't an intersection
		if( intersection.direction != 'n')
		{	
			handleIntersection();
		}
		//need to insert the position after checking the intersection, since
		// the intersection check looks for it inside. Not a huge deal,
		// but will be easier since we don't need it til now anyways...
		// That was a lot of comments for no real good reason.
		dpos.push_back(nia_position);
        
		current_distance = GetDistance(current_direction);

        //If we are in range of a wall, we start to reverse the drone

		if( current_distance <= Navi::WALL_THRESHOLD )
		{
			cout << "WALLLLLLL!" << endl;
			current_direction = ReverseDirection(current_direction);
			currently_reversing = true;
			dpos.pop_back();
		}
		
        // NOTE: This is very rudimentary. It just says to go the opposite direction
        // the number of times you went forward. It doesn't actually scan each time!
        // Just waits for drone/user to get back to the point in the interesection.
		if( currently_reversing == true )
		{
			cout << "Begin reversing" << endl;
			vector <double*> :: reverse_iterator dprit;
			for (dprit = dpos.rbegin(); *dprit != Navi::INTERSECTION_INDICATOR; ++dprit)
			{
				ReadyToReadScan();
				cout << "Go: " << current_direction << endl;
				//dpos.pop_back();
			}
			
			cout << "End reversing" << endl;	
			currently_reversing = false;
			did_reverse = true;
		}
	
		//just don't want to print out what direction to go twice	
		if(did_reverse != true)
		{
			cout << "Go: " << current_direction << endl;	
		}
		else //we did just reverse
		{
			did_reverse = false;
		}
	
	}


}

/*  handleIntersection: We know an intersection happened, and we are handeling the process of the interesection.
    
    Post-Conditions:
        dpos is updated with INTERSECTION_INDICATOR
        ddirec updated with intersection information
*/
void Navi::handleIntersection(){
	cout << "INTERSECTION!!!!!" << endl;
			
	bool prev_intersection = false;

	char prev_direction = current_direction; 
			
	current_direction = intersection.direction;
			
	//looking to see if we are at a new intersection!
    // Doing this by seeing what positions we have been at; i.e. the X Y coordinates
    // if this X Y coordinate is not in the dpos, then we say it is a new intersection
    //  NOTE: This will probably cause potential problems. Will need some love I am sure.
	if ( !IsValueInDpos(nia_position) )
	{
        //indicator of an intersection
		ddirec.push_back('x');
		//If we reach an intersection, don't want to go down the way we came
		// when we come back to it
		ddirec.push_back(ReverseDirection(prev_direction));
	}
	else //we are at a previous intersection
	{
		prev_intersection = true;
	}
			
			
	//This means we need to go back through the other paths we did not take yet
	if( prev_intersection == true )
	{
		//making temp ints for intersection maxes
        //imbd = intersection max back distance; mbd = max back distance
		double imbd = mbd;
        //imfd = intersection max forward distance; mfd = max forward distance
		double imfd = mfd;
        //imld = intersection max left distance; mld = max left distance
		double imld = mld;
        //imrd = intersection max right distance; mrd = max right distance
		double imrd = mrd;
		
        //used to send into the ChangeDistance function
		double idist[4] = {imbd, imfd, imld, imrd};
		
		//the idea behind this is that we go through all the previous routes we
		// went to at this intersection and make their max 0 (temporarily)
		// when we do this, we know which new direction to go
		vector <char> :: reverse_iterator ddrit;
		for (ddrit = ddirec.rbegin(); ddrit != ddirec.rend() && (*ddrit) != 'x'; ddrit++)
		{   
            //idist values can/should change after this call!
			ChangeDistance(*ddrit, 0, idist);
		}
				
        //update to the new max distance values
		imbd = idist[0];
		imfd = idist[1];
		imld = idist[2];
		imrd = idist[3];
			
        //Checking l and f here, since it will basically check all directions at the
		// intersection. This is mostly arbitrary.
		current_direction = 'l';
		intersection = CheckForIntersections(imbd, imfd, imld, imrd, current_direction);
        //If there wasn't an interesection indcated, then check the perpendicular axis (i.e. check for forward)
		if(intersection.direction == 'n')
		{
			current_direction = 'f';	
			intersection = CheckForIntersections(imbd, imfd, imld, imrd, current_direction);
		}
        
		current_direction = intersection.direction;
		
        //If both left and front did not indicate an interesection, that means there was no path to go down, and is thus
        // no other paths to take. This will make the drone end at this interesection though.
		if (current_direction == 'n')
		{
			cout << "THE DRONE HAS NOWHERE ELSE TO GO! NAVIGATION COMPLETE!" << endl;
			exit(0);
		}			

	}
    
    //inditaces there is an intersection at this position after the last X Y coordinate
	dpos.push_back(Navi::INTERSECTION_INDICATOR);
			
	//don't want to go back down this way when we travel back to the
	// intersection from another direction, so make sure to put on ddirec
	ddirec.push_back(current_direction);
}

/* ReadyToReadScan: This just waits until a command is given ('y') to indicate that
                     the drone/user is ready for the navigation to read the new scan file.
*/
void Navi::ReadyToReadScan()
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
				cout << "Please enter either 'y' or 'e'." << endl;
		}
	}
}

/* ReadScanFile: Reads the LIDAR Data.
                 Format of the lidar data should be:
                 NODE <X-coordinate> <Y-coordinate> <Z-coordinate> 0 0 0
                 <angle_in_degrees> <distance_in_meters>
                 ...
                 
                 Note: The scan file should NOT append; it should be a new scan each time,
                        using the same file name
    
    Returns: 
        array[0] = node coordinates; [X-coordinate][Y-coordinate]
		array[1] = max backward distance vector; [angle][distance]
		array[2] = max forward distance vector; [angle][distance]
		array[3] = max left distance vector; [angle][distance]
		array[4] = max right distance vector; [angle][distance]
*/
double ** Navi::ReadScanFile(string fn)
{
	ifstream fin;
	double scan_angle;
	double scan_dist;
	
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
    //junk is the "NODE" string
	iss >> junk;
	iss >> x_node_info;
	iss >> y_node_info;
    //don't need the z-coordinate or the 0 0 0; the 0 0 0 is for point clouds, which we didn't use
	node_info[0] = x_node_info;
	node_info[1] = y_node_info;

	//getting each x and y pair of data
	while( getline(fin, info_line) )
	{
		istringstream iss(info_line);

		iss >> scan_angle;
		iss >> scan_dist;
        
        //checking to see how this scan does when compared to all directions;
        // need to do this since it might be closer to one of angles than another
        // i.e. if the angle is 1 degree, it will be very close to the x forward axis, which is 0/360.
        //      if there isn't anything closer, it will be the max distance for the forward.
        //      But, 1 degree is not indictive to backwards, which is 180 degrees. 
		max_backward_x = CheckMaxCoord(scan_angle, scan_dist, 'b', max_backward_x);
		max_forward_x = CheckMaxCoord(scan_angle, scan_dist, 'f', max_forward_x);
		max_left_y = CheckMaxCoord(scan_angle, scan_dist, 'l', max_left_y);
		max_right_y = CheckMaxCoord(scan_angle, scan_dist, 'r', max_right_y);
	}
	fin.close();
    
    //This is just so we can return everything at once
	double ** max_arrays = new double*[5];
	max_arrays[0] = node_info;
	max_arrays[1] = max_backward_x;
	max_arrays[2] = max_forward_x;
	max_arrays[3] = max_left_y;
	max_arrays[4] = max_right_y;

	return max_arrays;	
}



/*  CheckMaxCoord: Returns the max distance, based on the angles closest to the x and y axis;
                    i.e. 0/360, 90, 180, and 270 degrees
    
    Parameters:
        angle: the angle of the current scan information
        dist: the distance of the current scan information
        check_axis: indicates if we are checking 'f'orward (0/360), 'b'ackward(180), 'l'eft(90), or 'r'ight(270).
        
    Return: a double array for the current angle closest to the respective check_axis, with its respective distance.
*/
double * Navi::CheckMaxCoord(double angle, double dist, char check_axis, double current_min[])
{
    //keeps track of the min array
	double * new_min_array = new double[2];
	
	//if it is the first loop, need to set the min to current point as initialization
	if( current_min == NULL )
	{
		new_min_array[0] = angle;
		new_min_array[1] = dist;
		return new_min_array;
	}
    
    //check_angle is used to compare against the current angle
    double check_angle = current_min[0];
    //check_dist is used to compare against the current distance
	double check_dist = current_min[1];
    
    //need large values since we are checking for min
    double angle_calc = 100000;
    double calc_check_angle = 100000;

    
	new_min_array[0] = current_min[0];
	new_min_array[1] = current_min[1];

    switch(check_axis)
    {
        //backwards
        case 'b':
            //check to see how close angle is to 180 degrees
            angle_calc = abs(angle - 180);
            calc_check_angle = abs(check_angle - 180);
            //if this angle is closer to 180, then change the min array
            if (angle_calc < calc_check_angle)
            {
                new_min_array[0] = angle;
                new_min_array[1] = dist;
            }
            break;
            
        //forward
        case 'f':
            if (angle >= 180) //check for numbers closer to 360
            {
                if( check_angle >= 180 ) //closer to 360
                {
                    calc_check_angle = abs(check_angle - 360);
                }
                else //is closer to 0
                {
                    calc_check_angle = abs(check_angle - 0);
                }
                
                angle_calc = abs(angle - 360);
                
                //if this angle is closer to 360, then change the min array
                if (angle_calc < calc_check_angle)
                {
                    new_min_array[0] = angle;
                    new_min_array[1] = dist;
                }
            }
            else //check for numbers closer to 0
            {
                if( check_angle >= 180 ) //closer to 360
                {
                    calc_check_angle = abs(check_angle - 360);
                }
                else //is closer to 0
                {
                    calc_check_angle = abs(check_angle - 0);
                }
                angle_calc = abs(angle - 0);
                
                //if this angle is closer to 0, then change the min array
                if (angle_calc < calc_check_angle)
                {
                    new_min_array[0] = angle;
                    new_min_array[1] = dist;
                }
            }
            break;
            
        //left
        case 'l':
            //check to see how close angle is to 90
            angle_calc = abs(angle - 90);
            calc_check_angle = abs(check_angle - 90);

            //if this angle is closer to 90, then change the min array
            if (angle_calc < calc_check_angle)
            {
                new_min_array[0] = angle;
                new_min_array[1] = dist;
            }
            break;
        
        //right
        case 'r':
            //check to see how close angle is to 270
            angle_calc = abs(angle - 270);
            calc_check_angle = abs(check_angle - 270);
            //if this angle is closer to 270, then change the min array
            if (angle_calc < calc_check_angle)
            {
                new_min_array[0] = angle;
                new_min_array[1] = dist;
            }
            break;
        
        default:
            cout << "CheckMaxCoor check_axis is not valid; or you are at the end of the navigation(?)." << endl;
            break;

    }	
    delete[] current_min;
	
	return new_min_array;
}

/* FindMaxDistance: Goes through the distances of the 4 directions and finds the max distance among them.
    Parameters:
        bd: back distance
        fd: forward distance
        ld: left distance
        rd: right distance
    
    Returns:
        The distance and direction of the direction that had the maximum distance
*/
DistAndDirec Navi::FindMaxDistance(double bd, double fd, double ld, double rd)
{	
    //I am putting fd first because this would be the direction to move
    //if there is a tie between the distances.
	double distance_array[4] = {fd, bd, ld, rd};

    //need to initialize to 0 since looking for max
	double cur_max = 0;
	int best_index = -1;
	
	char direction = 'n';
    
    //goes through each distance to find the max
	for(int i = 0; i < 4; i++)
	{
		if (abs(distance_array[i]) > cur_max)
		{
			cur_max = abs(distance_array[i]);
			best_index = i;
		}
	}
	
    // index indicated by above loop to find max distance
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

/*  CheckForIntersections: Checks to see if there is an interesection
    
    Parameters:
        bd = backwards distance
        fd = forward distance
        ld = left distance
        rf = right distance
        cd = current direction
    
    Returns:
        The direction and distance for which way to go down the interesection, based on max distance.
*/
DistAndDirec Navi::CheckForIntersections(double bd, double fd, double ld, double rd, char cd)
{
	intersection.distance = 0;
    //n is used for error checking/knowing there is nothing left to check at the intersection
	intersection.direction = 'n';
	
	//We check for f and b here, because we don't want to go in the opposite direction
	// and loop between the two largest values.
	if( cd == 'f' || cd == 'b' )
	{
		//0, 0 is because we only want to look for mld and mrd
		drone_dd_vals = FindMaxDistance(0, 0, ld, rd);
		if (drone_dd_vals.distance >= Navi::INTERSECTION_THRESHOLD)
		{
			intersection.distance = drone_dd_vals.distance;
			intersection.direction = drone_dd_vals.direction;
		}
	}
    
    //We check for l and r here, because we don't want to go in the opposite direction
	// and loop between the two largest values.
	if( cd == 'l' || cd == 'r' )
	{
		//0, 0 is because we only want to look for mld and mrd
		drone_dd_vals = FindMaxDistance(bd, fd, 0, 0);
		if (drone_dd_vals.distance >= Navi::INTERSECTION_THRESHOLD)
		{
			intersection.distance = drone_dd_vals.distance;
			intersection.direction = drone_dd_vals.direction;
		}
	}

	return intersection;
}

/*  GetDistance: Gets the distance of the current direction
    
    Parameters:
        cd = current direction

    Returns:
        Distance of the current direction
*/
double Navi::GetDistance(char cd)
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
			cout << "Error in GetDistance fuction; invalid direcetion input." << endl;
			break;
	}	
	
	return current_distance;
}


/*  ReverseDirection: Gives the direction opposite of the current direction moving.
    
    Parameters:
        cd = current_direction
    
    Returns:
        The direction opposite of the current moving direction.
        i.e. if going forward, return 'b'ackwards, and etc.
*/
char Navi::ReverseDirection(char cd)
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


/*  ChangeDistance: Changes the value of one value in the dist array, based on the cd.
    
    Parameters:
        cd = current_direction; this indicates which value to change in the dist array
        dist = array of [backwards dist, foward dist, left dist, right dist]
        cv = The value to change to
    
    Post-Conditions:
        The value in the dist array will change to the cv value, based on the cd index.
*/
void Navi::ChangeDistance(char cd, double cv, double * dist)
{
	//new value
	double nv;

	switch(cd)
	{
		case 'f': //forward
			dist[1] = cv;
			break;
		case 'b': //backward
			dist[0] = cv;
			break;
		case 'l': //left
			dist[2] = cv;
			break;
		case 'r': //right
			dist[3] = cv;
			break;
		default:
			cout << "Invalid input of direction in ChangeDistance function." << endl;
			break;
	}
}

/* IsValueInDpos: Checks to see if a X Y node coordinate is in dpos

    Parameters:
        value: the X Y coordinate pair to check for in dpos
        
    Returns:
        True if value is in the dpos array; false otherwise.
*/
bool Navi::IsValueInDpos(double *value)
{
	vector <double*> :: iterator dpit;
    //goes through the dpos array
	for( dpit = dpos.begin(); dpit != dpos.end(); dpit++)
	{
		//if X and Y are same values, then is in dpos vector
        if( (*dpit)[0] == value[0] && (*dpit)[1] == value[1] )
		{
			return true;
		}
	}
	return false;
}
