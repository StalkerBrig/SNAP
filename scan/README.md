Scanning Library for Scanse Sweep Application
=============================================

*Work in Progress*
------------------

Requirements
-----------------

First and foremost, follow these instruction to get Scanse Sweep working with the Raspberry Pi:
https://github.com/scanse/sweep-3d-scanner/wiki/Setup#setting-up-raspberry-pi-from-scratch

You don't need to follow the whole thing, just get the C++ libraries installed on your Pi.
This is what the -lsweep link in the compilation is for

The scanning code was adapted from the examples listed in the above instructions


Currently compiled (to object file) with:

    g++ -std=c++11 -c scan.cpp -lsweep

See Config folder for configuration instructions
