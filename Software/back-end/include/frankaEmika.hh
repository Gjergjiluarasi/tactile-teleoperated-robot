#ifndef FRANKAEMIKA_HH
#define FRANKAEMIKA_HH

// System libraries

// Custom headers
#include "robot.hh"
#include "udpUtils.hh"

// Default paramters
#define DEFAULT_FRANKA_EMIKA_ROBOT_IP ""
#define DEFAULT_FRANKA_EMIKA_PC_UDP_IP ""
#define DEFAULT_FRANKA_EMIKA_PC_UDP_PORT 0

// FrankaEmika class
class FrankaEmika : public Robot, public UdpClientServer::UdpServer
{
private:
    // Private attributes
    string _frankaIP;
    string _pcUdpIP;
    int _pcUdpPort;

public:
    // Public attributes
    bool plotFlag;

    // Constructors
    FrankaEmika();
    FrankaEmika(uint a_ID, string a_name, string a_description, string a_frankaIP, string a_pcUdpIP, int a_pcUdpPort, bool a_plotFlag);
    // Member functions
    void setFrankaIP(string a_frankaIP);
    string getFrankaIP();
    // TO DO: set UDP IP in client class too
    // void setPcUdpIP(string a_pcUdpIP);
    string getPcUdpIP();
    // TO DO: set UDP Port in client class too
    // void setPcUdpPort(int a_pcUdpPort);
    int getPcUdpPort();
};

#endif