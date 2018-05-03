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
    
	Navi navi;
	
    //the mainLoop is the navigation loop for going through hallways
    navi.mainLoop( file_name );
	
	exit(0);
}

