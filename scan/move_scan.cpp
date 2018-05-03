#include "scan.h"
#include "scan.cpp"
#include <fstream>
#include <iostream>
#include <stdio.h>

int main (int argc, char** argv){

    
    remove("movedata.log");    
    remove("navidata.dat");    
    Scan *S = new Scan();
    std::ofstream outfile;
    std::ofstream navfile;

    outfile.open("movedata.log", std::ofstream::out | std::ofstream::app);
    navfile.open("navidata.dat", std::ofstream::out);

    outfile << "NODE 0 0 0 0 0 0" << std::endl;
    navfile << "NODE 0 0 0 0 0 0" << std::endl;

    S->perform_scan(outfile, navfile);
    navfile.close();
    int x,y,z;
    
    std::cout << "Input Distance: ";
    while (std::cin >> x >> y >> z){

        if (x == -1000) break;

        navfile.open("navidata.dat", std::ofstream::out);
        outfile << "NODE " << x << " " << y << " " << z << " 0 0 0" << std::endl;
        navfile << "NODE " << x << " " << y << " " << z << " 0 0 0" << std::endl;
        S->perform_scan(outfile, navfile);
        navfile.close();
        std::cout << "Input Distance: ";
    }

    outfile.close();

    return 0;    
}
