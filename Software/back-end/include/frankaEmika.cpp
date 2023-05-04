#include "frankaEmika.hh"

// Default FrankaEmika constructor
FrankaEmika::FrankaEmika() : Robot(),
                             UdpClientServer::UdpServer(DEFAULT_FRANKA_EMIKA_PC_UDP_IP, DEFAULT_FRANKA_EMIKA_PC_UDP_PORT)
{
    _frankaIP = DEFAULT_FRANKA_EMIKA_ROBOT_IP;
    _pcUdpIP = get_addr();
    _pcUdpPort = get_port();
    plotFlag = false;
}

// Custom FrankaEmika constructor
FrankaEmika::FrankaEmika(uint a_ID, string a_name, string a_description, string a_frankaIP, string a_pcUdpIP, int a_pcUdpPort, bool a_plotFlag) : Robot(a_ID, a_name, a_description),
                                                                                                                                                  UdpClientServer::UdpServer(a_pcUdpIP, a_pcUdpPort)
{
    _frankaIP = a_frankaIP;
    _pcUdpIP = get_addr();
    _pcUdpPort = get_port();
    plotFlag = a_plotFlag;
}

// Member functions
void FrankaEmika::setFrankaIP(string a_frankaIP)
{
    _frankaIP = a_frankaIP;
}

string FrankaEmika::getFrankaIP()
{
    return _frankaIP;
}

// TO DO: set UDP IP in client class too
// void FrankaEmika::setPcUdpIP(string a_pcUdpIP)
// {
//     _pcUdpIP = a_pcUdpIP;
// }

string FrankaEmika::getPcUdpIP()
{
    return _pcUdpIP;
}

// TO DO: set UDP Port in client class too
// void FrankaEmika::setPcUdpPort(int a_pcUdpPort)
// {
//     _pcUdpPort = a_pcUdpPort;
// }

int FrankaEmika::getPcUdpPort()
{
    return _pcUdpPort;
}