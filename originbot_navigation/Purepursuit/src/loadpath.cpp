#include "loadpath.h"

WaypointLoader::WaypointLoader(const string name) {
    path_name = name;
}

//加载路径点
bool WaypointLoader::load_waypoints() {
    bool fileLoadFlag;
    ifstream ifs;
    ifs.open(path_name,ios::in);
    if(!ifs.is_open()) {
        fileLoadFlag = false;
    }
    char ch;
    ifs >> ch;
    if(ifs.eof())
    {
        fileLoadFlag = false;
    }
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
        waypoint.push_back(v);  
    }
    fileLoadFlag = true;
    ifs.close();
    return fileLoadFlag;
}

vector<vector<double>> WaypointLoader::get_waypoints() {
    return waypoint;
}