#ifndef ENDOWRIST_HH
#define ENDOWRIST_HH

#include "eposMotorController.hh"

/****************************************************************************/
/****************************************************************************/

#if USE_ETHERCAT_ECRT_LIB

// System libraries

// Custom headers
#include "robot.hh"

// Default paramters

// EndoWrist class
class EndoWrist : public Robot
{
private:
    // Private attributes
    string _protocolStackName;
    string _interfaceName;
    string _portName;

public:
    // Public attributes
    EposMotorController _yawMotor, _wristMotor, _leftGripperMotor, _rightGripperMotor;
    MotorControllerParameters _yawMotorParams, _wristMotorParams, _leftGripperMotorParams, _rightGripperMotorParams;

    // Constructors
    EndoWrist();
    EndoWrist(uint a_ID,
              string a_name,
              string a_description,
              MotorControllerParameters &a_yawMotorParams,
              MotorControllerParameters &_wristMotorParams,
              MotorControllerParameters &_leftGripperMotorParams,
              MotorControllerParameters &_rightGripperMotorParams);

    // Member functions
    // runProgramResult run();
    // int initMotors();
    //  int setYawPosition(long a_yawPosition);
    //  int setWristPosition(long a_wristPosition);
    //  int setLeftGripperPosition(long a_leftGripperPosition);
    //  int setRightGripperPosition(long a_rightGripperPosition);
};

#endif

/****************************************************************************/
/****************************************************************************/

#if USE_USB_EPOS_CMD_LIB

// System libraries

// Custom headers
#include "robot.hh"

// Default paramters

// EndoWrist class
class EndoWrist : public Robot
{
private:
    // Private attributes
    long _yawAxisPosition;
    long _yawAxisVelocity;
    long _yawAxisCurrent;
    long _nodeID;
    string _deviceName;
    string _protocolStackName;
    string _interfaceName;
    string _portName;
    int _baudrate;
    void *_pKeyHandle;

public:
    // Public attributes
    EposMotorController _yawMotor, _wristMotor, _leftGripperMotor, _rightGripperMotor;
    MotorControllerParameters _yawMotorParams, _wristMotorParams, _leftGripperMotorParams, _rightGripperMotorParams;

    // Constructors
    EndoWrist();
    EndoWrist(uint a_ID,
              string a_name,
              string a_description,
              MotorControllerParameters &a_yawMotorParams,
              MotorControllerParameters &_wristMotorParams,
              MotorControllerParameters &_leftGripperMotorParams,
              MotorControllerParameters &_rightGripperMotorParams);

    // Member functions
    void set_nodeID(long a_nodeID);
    long get_nodeID();
    int initMotors();
    int setYawPosition(long a_yawPosition);
    int setWristPosition(long a_wristPosition);
    int setLeftGripperPosition(long a_leftGripperPosition);
    int setRightGripperPosition(long a_rightGripperPosition);
};

#endif

/****************************************************************************/
/****************************************************************************/
#endif