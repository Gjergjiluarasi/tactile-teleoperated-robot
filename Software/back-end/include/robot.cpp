#include "robot.hh"

// Default Robot constructor
Robot::Robot()
{
    _ID = DEFAULT_ROBOT_ID;
    name = DEFAULT_ROBOT_NAME;
    description = DEFAULT_ROBOT_DESCRIPTION;
}

// Custom Robot constructor
Robot::Robot(uint a_ID, string a_name, string a_description)
{
    _ID = a_ID;
    name = a_name;
    description = a_description;
}

// Member functions
void Robot::setID(uint a_ID)
{
    _ID = a_ID;
}

uint Robot::getID()
{
    return _ID;
}
