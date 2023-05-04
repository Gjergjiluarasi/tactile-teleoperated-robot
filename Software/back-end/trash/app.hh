#ifndef APP_HH
#define APP_HH

// System libraries
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include "matplotlibcpp.h"
#include <string>
#include <thread>

// Franka Emika libraries
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "examples_common.h"
#include "rcmKinematics.hh"

// Custom libraries
#include "configUtils.hh"
#include "endoWrist.hh"
#include "frankaEmika.hh"
#include "robot.hh"
#include "udpUtils.hh"

// Default paramters
#define MAX_SIZE 255
#define ROBOT_TO_CONSOLE_RATIO 1.5
#define CONFIG_PATH "config/robot_map.xml"
#define DEFAULT_APP_NAME "6G-life"
#define DEFAULT_APP_DESCRIPTION "6G-life research drives cutting-edge research for future 6G communication networks with a \
                                focus on human-machine collaboration. 6G-life provides new approaches for sustainability, \
                                security, resilience and latency and will sustainably strengthen the economy and thus \
                                digital sovereignty in Germany"

// Namespaces
namespace plt = matplotlibcpp;
using namespace std;
using namespace literals::chrono_literals;

// Custom types
typedef enum
{
    ERROR,
    INIT,
    RUN,
    EXIT
} AppState;

typedef enum
{
    SUCCESS,
    ERR_INIT,
    ERR_RUN,
    ERR_EXIT
} AppResultState;

// App class
namespace App
{
    class App
    {
    private:
        // Private attributes
        AppState state;

    public:
        // Public attributes
        string name;
        string description;

        // Constructors
        App();
        App(string a_name, string a_description);

        // Member functions
        AppResultState RCM(void);
    };
}

#endif