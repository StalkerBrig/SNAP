/*
    call the hciscan shell script
    get the return value from the file
    it redirects it to. Take that number and
    put it into an integer
*/
#include <stdlib.h>
#include <stdio.h>
using namespace std;

int rssi()
{
    FILE *file;
    char string[56];
    int rssi;

    system("sh hciscan.sh");

    file = fopen("test.txt", "r");

    fgets(string, 56, file);
    rssi = atoi(string);

    //get absolute value
    if(rssi < 0) rssi = -rssi;

    return rssi;
}
