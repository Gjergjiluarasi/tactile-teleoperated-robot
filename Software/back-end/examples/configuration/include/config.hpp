#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <iostream>
#include <map>
#include <string>
#include <string_view>
#include <list>
#include <iterator>
#include <algorithm>
#include "pugixml.hpp"

using namespace std;
using namespace pugi;

typedef struct
{
    string robotName;
    string robotDescription;
    string robotID;
    string robotIP;
    string pcUdpIP;
    int pcUdpPort;
} robot_t;

map<string, robot_t> getRobotMap(const char *path);

void printRobotMap(string comment, const map<string, robot_t> &m);

bool configExample();

#endif