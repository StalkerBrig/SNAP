SNAP Navigation
=============================================

*Work in Progress*
------------------

testnavi.cpp is th code to actually run the navigation

navi.cpp contains the code for navigation classes

navi.h is the header file for the navigation classes


To build the navigation module using the Bebop SDK:
    • Go to the previous folder (Bebop2cpp-master)
    • Follow the instructions of the Bebop2cpp-master to build a project
        - For these specific files, make sure that the lines are in the "CMakeList.txt" file:
            add_executable(test_Navi SNAP/navi.cpp SNAP/test_navi.cpp)
            target_link_libraries(test_Navi bebop2cpp)
          Note: This should already be there. But, if you add other files, make sure to follow this example.
    • Once you build the project, the "build" folder should have an executable called "test_Navi", or whatever you named it.
    
Quick guide to build the Bebop2cpp-master:
    1) mkdir build
    2) cd build
    3) cmake ..
    4) make
    
This navigation module is not fully tested and will need some love. But, it should give a decent start.   
