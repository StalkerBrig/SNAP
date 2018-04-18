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

/* TODO: Need to uncomment drone code if trying to use drone;
          currently just trying to make a droneless code   */
//Drone bebop;

/* Signal catches; if a process dies, we need drone to emergency stop */
/*
void kill_handler (int junk)
{
	signal(SIGINT, kill_handler);
	cout << endl << "AHHHHHHHHHHHH, WHAT ARE YOU DOING" << endl;
	bebop.emergency();
	exit(1);
}
*/

int main(int argc, char *argv[]) {
	
	cout << argc << endl;	
	if( argc != 2 )
	{
		//need to put the file name of where the scans will go;
		// this assumes all scans will go to the same file
		cout << "PUT IN A FILE NAME, FOOL!" << endl;
 		exit(0);
	}
	

		
	/* SETUP FOR DONE */	
/*	signal(SIGINT, kill_handler);
	bebop.connect();

	while(!bebop.isRunning()){ sleep(1); }

	bebop.setMaxAltitude(DRONE_MAX_ALTITUDE);
   bebop.setMaxHorizontalSpeed(DRONE_MAX_HORIZONTAL_SPEED);
   bebop.setMaxVerticalSpeed(DRONE_MAX_VERTICAL_SPEED);

	if(bebop.blockingFlatTrim()) 
	{
		std::cout << "FLAT TRIM IS OK" << std::endl;
	}
	else
	{
		std::cerr << "FLAT TRIM NOT OK" << std::endl;
		return 1;
	}
*/	/*END SETUP*/
	
	/*
	   This is very rudimentary; the drone cannot currently fly with
		everything on it.
	*/
	
	string file_name = argv[1];
	
	bool getting_info_loop = true;
	//node info array
	double ** nia = NULL;
	
	//max backwards distance
	double mbd;
	//max forward distance
	double mfd;
	//max left distance
	double mld;
	//max right distance
	double mrd; 
	
	//n meaning not determined
	char current_direction = 'n';
	char initial_direction = 'n';	
	DistAndDirec drone_dd_vals;
	DistAndDirec intersection;
	double current_distance;
	
	bool currently_reversing = false;
	bool did_reverse = false;	

	while(getting_info_loop)
	{	
		ReadyToReadScan();
		
		//nia[0] = node coordinates
		//nia[1] = max backward distance vector
		//nia[2] = max forward distance vector
		//nia[3] = max left distance vector
		//nia[4] = max right distance vector
		nia = ReadScanFile(file_name);
		
		//helps readability, hopefully..
		
		double *nia_position = nia[0];
		mbd = nia[1][0];
		mfd = nia[2][0];
		mld = nia[3][1];
		mrd = nia[4][1];
		
		if( initial_direction == 'n' )
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
			handleIntersection( intersection, current_direction, nia_position, mbd, mfd, mld, mrd );
		}
		//need to insert the position after checking the intersection, since
		// the intersection check looks for it inside. Not a huge deal,
		// but will be easier since we don't need it til now anyways...
		// That was a lot of comments for no real good reason.
		DistAndDirec::dpos.push_back(nia_position);
		
		current_distance = GetDistance(mbd, mfd, mld, mrd, current_direction);

		if( current_distance <= DistAndDirec::WALL_THRESHOLD )
		{
			cout << "WALLLLLLL!" << endl;
			current_direction = ReverseDirection(current_direction);
			currently_reversing = true;
			DistAndDirec::dpos.pop_back();
			//want to remove last position, since we need to reverse back
			//DistAndDirec::dpos.pop_back();
		}
		
		if( currently_reversing == true )
		{
			cout << "Begin reversing" << endl;
			vector <double*> :: reverse_iterator dprit;
			for (dprit = DistAndDirec::dpos.rbegin(); *dprit != DistAndDirec::INTERSECTION_INDICATOR; ++dprit)
			{
				ReadyToReadScan();
				cout << "Go: " << current_direction << endl;
				//DistAndDirec::dpos.pop_back();
			}
			
			cout << "End reversing" << endl;	
			currently_reversing = false;
			did_reverse = true;
		}
	
		/*
		vector <double*> :: iterator dpit;	
		for(dpit = DistAndDirec::dpos.begin(); dpit != DistAndDirec::dpos.end(); dpit++)
		{	
			cout << *dpit[0] << endl;
		}	


		vector <char> :: iterator ddit;
		for( ddit = ddirec.begin(); ddit != ddirec.end(); ddit++)
		{
			cout << *ddit << endl;
		}
		*/
	
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

	exit(0);
}

