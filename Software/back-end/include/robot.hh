#ifndef ROBOT_HH
#define ROBOT_HH

// System libraries
#include <string>

// Custom headers

// Default paramters
#define DEFAULT_ROBOT_ID 0
#define DEFAULT_ROBOT_NAME "untitledRobot"
#define DEFAULT_ROBOT_DESCRIPTION "Robot object is used to serve as common interface for several robots"

using namespace std;

// Robot class
class Robot
{
private:
    // Private attributes
    uint _ID;

public:
    // Public attributes
    string name;
    string description;

    // Constructors
    Robot();
    Robot(uint a_ID, string a_name, string a_description);

    // Member functions
    void setID(uint a_ID);
    uint getID();
};

#endif