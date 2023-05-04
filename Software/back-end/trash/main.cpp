#include <iostream>
#include "configUtils.hh"
#include "udpUtils.hh"
#include "robot.hh"
#include "frankaEmika.hh"
#include "endoWrist.hh"
#define PORT 8080
#define MAX_SIZE 80

using namespace std;

/****************************************************************************/
/****************************************************************************/

#if USE_ETHERCAT_ECRT_LIB

int main()
{
    // Robot class test
    Robot testRobot;
    cout << "testRobot.name: " << testRobot.name << endl
         << "testRobot.description: " << testRobot.description << endl;
    cout << endl;
    // FrankaEmika class test
    const char *path = "config/robot_map.xml";
    map<string, robot_t> robotoMap = getRobotMap(path);
    string testFrankaEmikaName = robotoMap["0"].robotName;
    string testFrankaEmikaDescription = robotoMap["0"].robotDescription;
    uint testFrankaEmikaID = stoi(robotoMap["0"].robotID);
    string testFrankaEmikaIP = robotoMap["0"].robotIP;
    string testPcUdpIP = robotoMap["0"].pcUdpIP;
    int testPcUdpPort = robotoMap["0"].pcUdpPort;
    FrankaEmika testFrankaEmika(testFrankaEmikaID, testFrankaEmikaName, testFrankaEmikaDescription, testFrankaEmikaIP, testPcUdpIP, testPcUdpPort);
    cout << "testFrankaEmika.name: " << testFrankaEmika.name << endl
         << "testFrankaEmika.description: " << testFrankaEmika.description << endl
         << "testFrankaEmika._robotIP: " << testFrankaEmika.getRobotIP() << endl
         << "testFrankaEmika._ID: " << testFrankaEmika.getID() << endl
         << "testFrankaEmika.get_addr(): " << testFrankaEmika.get_addr() << endl
         << "testFrankaEmika.get_port(): " << testFrankaEmika.get_port() << endl
         << "testFrankaEmika._pcUdpIP: " << testFrankaEmika.getPcUdpIP() << endl
         << "testFrankaEmika._pcUdpPort: " << testFrankaEmika.getPcUdpPort() << endl;
    testFrankaEmika.send("Hello from testFrankaEmika\n", MAX_SIZE);
    cout << endl;

    // EndoWrist class test
    string testEndoWristIP = "192.168.3.110";
    uint testEndoWristID = 4;
    string testEndoWristPcUdpIP = "127.0.0.1";
    int testEndoWristPcUdpPort = PORT;
    MotorControllerParameters testEndoWristYawMotorParams = {
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristWristMotorParams = {
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristLeftGripperMotorParams = {
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristRightGripperMotorParams = {
        .position = 0,
        .velocity = 0,
        .current = 0};
    EndoWrist testEndoWrist(testEndoWristID, "testEndoWrist", "Used to test testEndoWrist class",
                            testEndoWristIP, testEndoWristPcUdpIP, testEndoWristPcUdpPort,
                            testEndoWristYawMotorParams, testEndoWristWristMotorParams,
                            testEndoWristLeftGripperMotorParams, testEndoWristRightGripperMotorParams);
    cout << "testEndoWrist.name: " << testEndoWrist.name << endl
         << "testEndoWrist.description: " << testEndoWrist.description << endl
         << "testEndoWrist._robotIP: " << testEndoWrist.getRobotIP() << endl
         << "testEndoWrist._ID: " << testEndoWrist.getID() << endl
         << "testEndoWrist.get_addr(): " << testEndoWrist.get_addr() << endl
         << "testEndoWrist.get_port(): " << testEndoWrist.get_port() << endl
         << "testEndoWrist._pcUdpIP: " << testEndoWrist.getPcUdpIP() << endl
         << "testEndoWrist._pcUdpPort: " << testEndoWrist.getPcUdpPort() << endl;
    testEndoWrist.send("Hello from testEndoWrist\n", MAX_SIZE);
    runProgramResult runProgramRes = testEndoWrist.run();
    cout << "Program result after run: " << runProgramRes << endl;

    return 0;
}

#endif

/****************************************************************************/
/****************************************************************************/

#if USE_USB_EPOS_CMD_LIB

int main()
{
    // Robot class test
    Robot testRobot;
    cout << "testRobot.name: " << testRobot.name << endl
         << "testRobot.description: " << testRobot.description << endl;
    cout << endl;

    // FrankaEmika class test
    string testFrankaEmikaIP = "192.168.3.109";
    uint testFrankaEmikaID = 3;
    string testPcUdpIP = "127.0.0.1";
    int testPcUdpPort = PORT;
    FrankaEmika testFrankaEmika(testFrankaEmikaID, "testFrankaEmika", "Used to test FrankaEmika class", testFrankaEmikaIP, testPcUdpIP, testPcUdpPort);
    cout << "testFrankaEmika.name: " << testFrankaEmika.name << endl
         << "testFrankaEmika.description: " << testFrankaEmika.description << endl
         << "testFrankaEmika._robotIP: " << testFrankaEmika.getRobotIP() << endl
         << "testFrankaEmika._ID: " << testFrankaEmika.getID() << endl
         << "testFrankaEmika.get_addr(): " << testFrankaEmika.get_addr() << endl
         << "testFrankaEmika.get_port(): " << testFrankaEmika.get_port() << endl
         << "testFrankaEmika._pcUdpIP: " << testFrankaEmika.getPcUdpIP() << endl
         << "testFrankaEmika._pcUdpPort: " << testFrankaEmika.getPcUdpPort() << endl;
    testFrankaEmika.send("Hello from testFrankaEmika\n", MAX_SIZE);
    cout << endl;

    // EndoWrist class test
    string testEndoWristIP = "192.168.3.110";
    uint testEndoWristID = 4;
    string testEndoWristPcUdpIP = "127.0.0.1";
    int testEndoWristPcUdpPort = PORT;
    MotorControllerParameters testEndoWristYawMotorParams = {
        .nodeID = 1,
        .deviceName = "EPOS4",
        .protocolStackName = "MAXON SERIAL V2",
        .interfaceName = "USB",
        .portName = "USB0",
        .baudrate = 1000000,
        .p_deviceHandle = NULL,
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristWristMotorParams = {
        .nodeID = 2,
        .deviceName = "EPOS4",
        .protocolStackName = "MAXON SERIAL V2",
        .interfaceName = "USB",
        .portName = "USB1",
        .baudrate = 1000000,
        .p_deviceHandle = NULL,
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristLeftGripperMotorParams = {
        .nodeID = 3,
        .deviceName = "EPOS4",
        .protocolStackName = "MAXON SERIAL V2",
        .interfaceName = "USB",
        .portName = "USB2",
        .baudrate = 1000000,
        .p_deviceHandle = NULL,
        .position = 0,
        .velocity = 0,
        .current = 0};
    MotorControllerParameters testEndoWristRightGripperMotorParams = {
        .nodeID = 4,
        .deviceName = "EPOS4",
        .protocolStackName = "MAXON SERIAL V2",
        .interfaceName = "USB",
        .portName = "USB3",
        .baudrate = 1000000,
        .p_deviceHandle = NULL,
        .position = 0,
        .velocity = 0,
        .current = 0};
    EndoWrist testEndoWrist(testEndoWristID, "testEndoWrist", "Used to test testEndoWrist class", testEndoWristIP, testEndoWristPcUdpIP, testEndoWristPcUdpPort, testEndoWristYawMotorParams, testEndoWristWristMotorParams, testEndoWristLeftGripperMotorParams, testEndoWristRightGripperMotorParams);
    cout << "testEndoWrist.name: " << testEndoWrist.name << endl
         << "testEndoWrist.description: " << testEndoWrist.description << endl
         << "testEndoWrist._robotIP: " << testEndoWrist.getRobotIP() << endl
         << "testEndoWrist._ID: " << testEndoWrist.getID() << endl
         << "testEndoWrist.get_addr(): " << testEndoWrist.get_addr() << endl
         << "testEndoWrist.get_port(): " << testEndoWrist.get_port() << endl
         << "testEndoWrist._pcUdpIP: " << testEndoWrist.getPcUdpIP() << endl
         << "testEndoWrist._pcUdpPort: " << testEndoWrist.getPcUdpPort() << endl;
    testEndoWrist.send("Hello from testEndoWrist\n", MAX_SIZE);
    cout << "testEndoWristYawMotor: nodeID = " << testEndoWrist._yawMotor.mParams.nodeID << ", deviceName = " << testEndoWrist._yawMotor.mParams.deviceName << ", protocolStackName = " << testEndoWrist._yawMotor.mParams.protocolStackName << ", interfaceName = " << testEndoWrist._yawMotor.mParams.interfaceName << ", portName = " << testEndoWrist._yawMotor.mParams.portName << ", baudrate = " << testEndoWrist._yawMotor.mParams.baudrate << ", p_deviceHandle = " << testEndoWrist._yawMotor.mParams.p_deviceHandle << endl;
    cout << "testEndoWristWristMotor: nodeID = " << testEndoWrist._wristMotor.mParams.nodeID << ", deviceName = " << testEndoWrist._wristMotor.mParams.deviceName << ", protocolStackName = " << testEndoWrist._wristMotor.mParams.protocolStackName << ", interfaceName = " << testEndoWrist._wristMotor.mParams.interfaceName << ", portName = " << testEndoWrist._wristMotor.mParams.portName << ", baudrate = " << testEndoWrist._wristMotor.mParams.baudrate << ", p_deviceHandle = " << testEndoWrist._wristMotor.mParams.p_deviceHandle << endl;
    cout << "testEndoWristLeftGripperMotor: nodeID = " << testEndoWrist._leftGripperMotor.mParams.nodeID << ", deviceName = " << testEndoWrist._leftGripperMotor.mParams.deviceName << ", protocolStackName = " << testEndoWrist._leftGripperMotor.mParams.protocolStackName << ", interfaceName = " << testEndoWrist._leftGripperMotor.mParams.interfaceName << ", portName = " << testEndoWrist._leftGripperMotor.mParams.portName << ", baudrate = " << testEndoWrist._leftGripperMotor.mParams.baudrate << ", p_deviceHandle = " << testEndoWrist._leftGripperMotor.mParams.p_deviceHandle << endl;
    cout << "testEndoWristRightGripperMotor: nodeID = " << testEndoWrist._rightGripperMotor.mParams.nodeID << ", deviceName = " << testEndoWrist._rightGripperMotor.mParams.deviceName << ", protocolStackName = " << testEndoWrist._rightGripperMotor.mParams.protocolStackName << ", interfaceName = " << testEndoWrist._rightGripperMotor.mParams.interfaceName << ", portName = " << testEndoWrist._rightGripperMotor.mParams.portName << ", baudrate = " << testEndoWrist._rightGripperMotor.mParams.baudrate << ", p_deviceHandle = " << testEndoWrist._rightGripperMotor.mParams.p_deviceHandle << endl;
    int motorInitResult = testEndoWrist.initMotors();
    // cout<<"Motor initalization result: "<<motorInitResult<<endl;
    unsigned int ulErrorCode = testEndoWrist._yawMotor.Init();
    testEndoWrist._yawMotor.setPosition(ulErrorCode, 75000);
    testEndoWrist._yawMotor.CloseDevice(&ulErrorCode);

    return 0;
}

#endif

/****************************************************************************/
/****************************************************************************/