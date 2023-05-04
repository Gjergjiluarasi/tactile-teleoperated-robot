/**
 * @file configUtils.hh
 * @author Gjergji Luarasi (gjergji.luarasi@tum.de)
 * @brief
 * @version 0.1
 * @date 2023-03-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONFIGUTILS_HH
#define CONFIGUTILS_HH

// Includes
#include <iostream>
#include <map>
#include <string>
#include <string_view>
#include <list>
#include <iterator>
#include <algorithm>
#include "pugixml.hpp"

// Defines
#define NUM_SENSORS 1

// Namespaces
using namespace std;
using namespace pugi;

// Types
typedef struct
{
    string Name;
    string Description;
    uint ID;
} tool_t;

typedef struct
{
    string Name;
    string Description;
    uint ID;
} sensor_t;
typedef struct
{
    string Name;
    string Description;
    uint ID;
    string FrankaIP;
    string PcUdpIP;
    int PcUdpPort;
    tool_t Tool;
    sensor_t Sensors[NUM_SENSORS];
    bool PlotFlag;
} robot_t;

map<uint, robot_t> getRobotMap(const char *path);

void printRobotMap(string comment, const map<uint, robot_t> &m);

#endif