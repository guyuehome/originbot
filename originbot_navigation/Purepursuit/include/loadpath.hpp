#ifndef LOAD_PATH_H
#define LOAD_PATH_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
using namespace std;

class WaypointLoader
{
    public:
        WaypointLoader(const string name);
        ~WaypointLoader(){};
        bool load_waypoints();
        vector<vector<double>> get_waypoints();

    private:
        string path_name;
        vector<vector<double>> waypoint;
};

#endif //LOAD_PATH_H
