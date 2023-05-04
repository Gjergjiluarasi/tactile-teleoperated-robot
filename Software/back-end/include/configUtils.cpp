#include "configUtils.hh"

map<uint, robot_t> getRobotMap(const char *path)
{
    xml_document doc;
    xml_parse_result result = doc.load_file(path);
    cout << "Load config file result: " << result.description() << endl
         << "Document type name: " << doc.first_child().name() << endl;
    xml_node robots = doc.child("RobotMap").child("Robots");
    map<uint, robot_t> robotMap;
    uint tempRobotIndex;
    robot_t temp_robot;
    uint i = 0;
    for (xml_node robot_node = robots.child("Robot"); robot_node; robot_node = robot_node.next_sibling("Robot"))
    {
        tempRobotIndex = (uint)atoi(robot_node.child("ID").first_child().value());
        temp_robot = {
            .Name = robot_node.child("Name").first_child().value(),
            .Description = robot_node.child("Description").first_child().value(),
            .ID = (uint)atoi(robot_node.child("ID").first_child().value()),
            .FrankaIP = robot_node.child("FrankaIP").first_child().value(),
            .PcUdpIP = robot_node.child("PcUdpIP").first_child().value(),
            .PcUdpPort = atoi(robot_node.child("PcUdpPort").first_child().value()),
            .Tool = {
                .Name = robot_node.child("Tool").child("Name").first_child().value(),
                .Description = robot_node.child("Tool").child("Description").first_child().value(),
                .ID = (uint)atoi(robot_node.child("Tool").child("ID").first_child().value())},
            .PlotFlag = (bool)atoi(robot_node.child("Plot").first_child().value())};
        i = 0;
        for (xml_node sensor_node = robot_node.child("Sensors").first_child(); sensor_node; sensor_node = sensor_node.next_sibling())
        {
            temp_robot.Sensors[i].Name = sensor_node.child("Name").first_child().value();
            temp_robot.Sensors[i].Description = sensor_node.child("Description").first_child().value();
            temp_robot.Sensors[i].ID = (uint)atoi(sensor_node.child("ID").first_child().value());
            i++;
        }
        robotMap[tempRobotIndex] = temp_robot;
    }
    return robotMap;
}

void printRobotMap(string comment, const map<uint, robot_t> &m)
{
    cout << comment;
    // iterate using C++17 facilities
    // for (const auto &[key, value] : m)
    // {
    //     cout << '[' << key << ']' << endl;
    //     cout << value.Name << ", " << value.Description << ", "
    //          << value.ID << ", " << value.FrankaIP << ", "
    //          << value.PcUdpIP << ", " << value.PcUdpPort << endl;
    // }
    sensor_t sensors[NUM_SENSORS];
    // C++11 alternative:
    for (const auto &el : m)
    {
        cout << el.first << ": " << endl
             << "  " << el.second.Name << ", " << el.second.Description << ", "
             << el.second.ID << ", " << el.second.FrankaIP << ", "
             << el.second.PcUdpIP << ", " << el.second.PcUdpPort << endl
             << "\t" << el.second.Tool.Name << ", " << el.second.Description << ", " << el.second.Tool.ID << endl;
        for (const auto &sensor : el.second.Sensors)
            cout << "\t\t" << sensor.Name << ", " << sensor.Description << ", " << sensor.ID << endl;
    }
    // C++98 alternative
    //  for (std::map<std::string, int>::const_iterator it = m.begin(); it != m.end(); it++)
    //      std::cout << it->first << " = " << it->second << "; ";
}