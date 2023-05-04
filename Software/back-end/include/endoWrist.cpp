#include "endoWrist.hh"

/****************************************************************************/
/****************************************************************************/

#if USE_ETHERCAT_ECRT_LIB

// Default EndoWrist constructor
EndoWrist::EndoWrist() : Robot()
{
    _protocolStackName = "CANopen Over EtherCAT";
    _interfaceName = "Ethernet";
    _portName = "eth0";
}

// Custom Robot constructor
EndoWrist::EndoWrist(uint a_ID,
                     string a_name,
                     string a_description,
                     MotorControllerParameters &a_yawMotorParams,
                     MotorControllerParameters &_wristMotorParams,
                     MotorControllerParameters &_leftGripperMotorParams,
                     MotorControllerParameters &_rightGripperMotorParams) : Robot(a_ID, a_name, a_description),
                                                                            _yawMotor(a_yawMotorParams),
                                                                            _wristMotor(_wristMotorParams),
                                                                            _leftGripperMotor(_leftGripperMotorParams),
                                                                            _rightGripperMotor(_rightGripperMotorParams)
{
    _protocolStackName = "CANopen Over EtherCAT";
    _interfaceName = "Ethernet";
    _portName = "eth0";
}

// Member functions
// runProgramResult EndoWrist::run()
// {
//     return runProgram();
// }
/*
int EndoWrist::initMotors()
{
    _yawMotor.Init();
    _wristMotor.Init();
    _leftGripperMotor.Init();
    _rightGripperMotor.Init();
}
*/

/*
int EndoWrist::setYawPosition(long a_yawPosition)
{

}

int EndoWrist::setWristPosition(long a_wristPosition)
{

}

int EndoWrist::setLeftGripperPosition(long a_leftGripperPosition)
{

}

int EndoWrist::setRightGripperPosition(long a_rightGripperPosition)
{

}
*/

#endif

/****************************************************************************/
/****************************************************************************/

#if USE_USB_EPOS_CMD_LIB

// Default EndoWrist constructor
EndoWrist::EndoWrist() : Robot()
{
    _yawAxisPosition = 0;
    _yawAxisVelocity = 0;
    _yawAxisCurrent = 0;
    _nodeID = 1;
    _deviceName = "EPOS4";
    _protocolStackName = "MAXON SERIAL V2";
    _interfaceName = "USB";
    _portName = "USB0";
    _baudrate = 1000000;
    _pKeyHandle = NULL;
}

// Custom Robot constructor
EndoWrist::EndoWrist(uint a_ID,
                     string a_name,
                     string a_description,
                     MotorControllerParameters &a_yawMotorParams,
                     MotorControllerParameters &_wristMotorParams,
                     MotorControllerParameters &_leftGripperMotorParams,
                     MotorControllerParameters &_rightGripperMotorParams) : Robot(a_ID, a_name, a_description),
                                                                            _yawMotor(a_yawMotorParams),
                                                                            _wristMotor(_wristMotorParams),
                                                                            _leftGripperMotor(_leftGripperMotorParams),
                                                                            _rightGripperMotor(_rightGripperMotorParams)
{
    _yawAxisPosition = 0;
    _yawAxisVelocity = 0;
    _yawAxisCurrent = 0;
    _nodeID = 1;
    _deviceName = "EPOS4";
    _protocolStackName = "MAXON SERIAL V2";
    _interfaceName = "USB";
    _portName = "USB0";
    _baudrate = 1000000;
    _pKeyHandle = NULL;
}

// Member functions
void EndoWrist::set_nodeID(long a_nodeID)
{
    _nodeID = a_nodeID;
}

long EndoWrist::get_nodeID()
{
    return _nodeID;
}

int EndoWrist::initMotors()
{
    _yawMotor.Init();
    _wristMotor.Init();
    _leftGripperMotor.Init();
    _rightGripperMotor.Init();
}

int EndoWrist::setYawPosition(long a_yawPosition)
{
}

int EndoWrist::setWristPosition(long a_wristPosition)
{
}

int EndoWrist::setLeftGripperPosition(long a_leftGripperPosition)
{
}

int EndoWrist::setRightGripperPosition(long a_rightGripperPosition)
{
}

#endif

/****************************************************************************/
/****************************************************************************/