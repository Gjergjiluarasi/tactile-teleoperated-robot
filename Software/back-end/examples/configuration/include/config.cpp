#include "config.hpp"

bool configExample()
{
    const char *path = "/home/gjergji/Master-Thesis/mirmi6Glife/6g-life/examples/configuration/robot_map.xml";
    map<string, robot_t> robotMap = getRobotMap(path);
    printRobotMap("Robot map: \n", robotMap);
    return true;
}

map<string, robot_t> getRobotMap(const char *path)
{
    xml_document doc;
    xml_parse_result result = doc.load_file(path);
    cout << "Load result: " << result.description() << endl
         << "Document type name: " << doc.first_child().name() << endl
         << endl;
    xml_node robots = doc.child("RobotMap").child("Robots");
    map<string, robot_t> robotMap;
    string tempRobotIndex;
    robot_t temp_robot;
    for (xml_node robot_node = robots.child("Robot"); robot_node; robot_node = robot_node.next_sibling("Robot"))
    {
        tempRobotIndex = robot_node.child("ID").first_child().value();
        temp_robot = {
            .robotName = robot_node.child("Name").first_child().value(),
            .robotDescription = robot_node.child("Description").first_child().value(),
            .robotID = robot_node.child("ID").first_child().value(),
            .robotIP = robot_node.child("RobotIP").first_child().value(),
            .pcUdpIP = robot_node.child("PcUdpIP").first_child().value(),
            .pcUdpPort = atoi(robot_node.child("PcUdpPort").first_child().value())};
        robotMap[tempRobotIndex] = temp_robot;
    }
    return robotMap;
}

void printRobotMap(string comment, const map<string, robot_t> &m)
{
    cout << comment;
    // iterate using C++17 facilities
    for (const auto &[key, value] : m)
    {
        cout << '[' << key << ']' << endl;
        cout << value.robotName << ", " << value.robotDescription << ", "
             << value.robotID << ", " << value.robotIP << ", "
             << value.pcUdpIP << ", " << value.pcUdpPort << endl;
    }
    // C++11 alternative:
    //  for (const auto& n : m)
    //      std::cout << n.first << " = " << n.second << "; ";
    //
    // C++98 alternative
    //  for (std::map<std::string, int>::const_iterator it = m.begin(); it != m.end(); it++)
    //      std::cout << it->first << " = " << it->second << "; ";
}