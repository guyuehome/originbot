/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

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
        virtual ~WaypointLoader(){};
        bool load_waypoints();
        vector<vector<double>> get_waypoints();

    private:
        string path_name;
        vector<vector<double>> waypoint;
};

#endif //LOAD_PATH_H
