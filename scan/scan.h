#pragma once

#include <string>
#include <vector>
#include <cstdlib>
#include <cstdio>
#include <sweep/sweep.hpp>
#include <fstream>

class Scan{
    
    public:
        Scan();
        int perform_scan(std::ofstream& outfile, std::ofstream& navfile);
        void set_device_path(std::string input_path);
        double DegreestoRadians(const double& degrees);
		void coord_polar_to_rect(const double& radius, const double& angle_degrees, double& x, double& y);
        std::string get_device_path();
        
    private:
       std::string devicepath;
       std::string sweeppath;
       std::vector <int> angles;
       std::vector <int> distance;
       int num_scans_per;
       int motor_speed;
       int sample_rate;
};

