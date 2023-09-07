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

#include "loadPath.h"

WaypointLoader::WaypointLoader(const string name)
{
    path_name = name;
}

//加载路径点
bool WaypointLoader::loadWayPoints() 
{
    bool fileLoadFlag;
    ifstream ifs;
    ifs.open(path_name,ios::in);
    if(!ifs.is_open())
        fileLoadFlag = false;
    char ch;
    ifs >> ch;
    if(ifs.eof())
        fileLoadFlag = false;
    ifs.putback(ch);

    string line;
    string field;
    while (getline(ifs,line)) {
        vector<double> v;
        istringstream sin(line);
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));
        getline(sin,field,',');
        v.push_back(atof(field.c_str()));
        wayPoint.push_back(v);  
    }
    fileLoadFlag = true;
    ifs.close();
    return fileLoadFlag;
}

vector<vector<double>> WaypointLoader::getWayPoints() 
{
    return wayPoint;
}
