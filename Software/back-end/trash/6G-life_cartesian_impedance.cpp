// 6G-life project main file

// Franka Emika includes
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "examples_common.h"

// EndoWrist includes
#include "configUtils.hh"
#include "udpUtils.hh"
#include "robot.hh"
#include "frankaEmika.hh"
#include "endoWrist.hh"
#define PORT 8080
#define MAX_SIZE 255

using namespace std;

/*** @example cartesian_impedance_control.cppw ***/

int main(int argc, char **argv)
{
  // EndoWrist part
  const char *path = "/home/mirmi1/endowrist/config/robot_map.xml";
  map<string, robot_t> robotMap = getRobotMap(path);
  printRobotMap("Configuration file contents: ", robotMap);
  string leftFR3Name = robotMap["0"].robotName;
  string leftFR3Description = robotMap["0"].robotDescription;
  uint leftFR3ID = stoi(robotMap["0"].robotID);
  string leftFR3IP = robotMap["0"].robotIP;
  string leftFR3PcUdpIP = "192.168.3.110";
  int leftFR3PcUdpPort = robotMap["0"].pcUdpPort;
  FrankaEmika leftFR3(leftFR3ID, leftFR3Name, leftFR3Description, leftFR3IP, leftFR3PcUdpIP, leftFR3PcUdpPort);
  cout << "leftFR3.name: " << leftFR3.name << endl
       << "leftFR3.description: " << leftFR3.description << endl
       << "leftFR3._robotIP: " << leftFR3.getRobotIP() << endl
       << "leftFR3._ID: " << leftFR3.getID() << endl
       << "leftFR3.get_addr(): " << leftFR3.get_addr() << endl
       << "leftFR3.get_port(): " << leftFR3.get_port() << endl
       << "leftFR3._pcUdpIP: " << leftFR3.getPcUdpIP() << endl
       << "leftFR3._pcUdpPort: " << leftFR3.getPcUdpPort() << endl;
  // leftFR3.send("Hello from leftFR3\n", MAX_SIZE);
  cout << endl;

  // EndoWrist class test
  uint leftFR3EndoWristID = stoi(robotMap["0"].robotID);  // EndoWrist ID should be same as Franka Emika robot arm ID
  string leftFR3EndoWristPcUdpIP = robotMap["0"].pcUdpIP; // PC UDP IP should be used for controlling the EndoWrist from the haptic console
  int leftFR3EndoWristPcUdpPort = robotMap["0"].pcUdpPort;
  MotorControllerParameters leftFR3EndoWristYawMotorParams = {
      .position = 0,
      .velocity = 0,
      .current = 0};
  MotorControllerParameters leftFR3EndoWristWristMotorParams = {
      .position = 0,
      .velocity = 0,
      .current = 0};
  MotorControllerParameters leftFR3EndoWristLeftGripperMotorParams = {
      .position = 0,
      .velocity = 0,
      .current = 0};
  MotorControllerParameters leftFR3EndoWristRightGripperMotorParams = {
      .position = 0,
      .velocity = 0,
      .current = 0};
  EndoWrist leftFR3EndoWrist(leftFR3EndoWristID, "Left FR3 EndoWrist", "Left FR3 EndoWrist used to control Da Vinci EndoWrist tool",
                             leftFR3EndoWristPcUdpIP, leftFR3EndoWristPcUdpPort,
                             leftFR3EndoWristYawMotorParams, leftFR3EndoWristWristMotorParams,
                             leftFR3EndoWristLeftGripperMotorParams, leftFR3EndoWristRightGripperMotorParams);
  cout << "leftFR3EndoWrist.name: " << leftFR3EndoWrist.name << endl
       << "leftFR3EndoWrist.description: " << leftFR3EndoWrist.description << endl
       << "leftFR3EndoWrist._ID: " << leftFR3EndoWrist.getID() << endl
       << "leftFR3EndoWrist.get_addr(): " << leftFR3EndoWrist.get_addr() << endl
       << "leftFR3EndoWrist.get_port(): " << leftFR3EndoWrist.get_port() << endl
       << "leftFR3EndoWrist._pcUdpIP: " << leftFR3EndoWrist.getPcUdpIP() << endl
       << "leftFR3EndoWrist._pcUdpPort: " << leftFR3EndoWrist.getPcUdpPort() << endl;
  double message[13];
  cout << "Starting server " << leftFR3EndoWrist.get_addr() << ':' << leftFR3EndoWrist.get_port() << endl;
  while (1)
    if (leftFR3EndoWrist.recv(message, MAX_SIZE))
      cout << "time in [s]: " << message[0]
           << "\tleft hand x in [m]: " << message[1]
           << "\tleft hand y in [m]: " << message[2]
           << "\tleft hand z in [m]: " << message[3]
           << "\tleft hand v_x in [m/s]: " << message[4]
           << "\tleft hand v_y in [m/s]: " << message[5]
           << "\tleft hand v_z in [m/s]: " << message[6]
           << "\tright hand x in [m]: " << message[7]
           << "\tright hand y in [m]: " << message[8]
           << "\tright hand z in [m]: " << message[9]
           << "\tright hand v_x in [m/s]: " << message[10]
           << "\tright hand v_y in [m/s]: " << message[11]
           << "\tright hand v_z in [m/s]: " << message[12] << endl;

  runProgramResult runProgramRes = initProgram();
  cout << "Program result afer initialization: " << runProgramRes << endl;
  // runProgramRes = runProgram();
  // cout << "Program result after run: " << runProgramRes << endl;

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  try
  {
    // connect to robot
    franka::Robot robot(leftFR3.getRobotIP());
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();

    franka::RobotState initial_state = robot.readOnce();

    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Quaterniond orientation_d(initial_transform.rotation());

    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    //////////////////////////////////////////////////////////////////////////////////////////////
    // EndoWrist part
    int ret = 0;
    struct timespec wakeup_time;
    // Set command result returns the status of the operating commands (PPM, PVM, CPM, CVM or CTM)
    static setCommandResult setCommandRes = setCommandResult::ERROR_SET_C;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
    wakeup_time.tv_sec += 1; /* start in future */
    wakeup_time.tv_nsec = 0;
    //////////////////////////////////////////////////////////////////////////////////////////////

    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState &robot_state,
                                         franka::Duration /*duration*/) -> franka::Torques
    {
      // get state variables
      std::array<double, 7> coriolis_array = model.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.rotation());

      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d;

      // orientation error
      // "difference" quaternion
      if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
      {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.rotation() * error.tail(3);

      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);

      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
      tau_d << tau_task + coriolis;

      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

      //////////////////////////////////////////////////////////////////////////////////////////////
      // EndoWrist part
      ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
      if (ret)
      {
        fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
        // break;
      }

      setCommandRes = setProfilePosition(&wakeup_time);

      wakeup_time.tv_nsec += PERIOD_NS;
      while (wakeup_time.tv_nsec >= NSEC_PER_SEC)
      {
        wakeup_time.tv_nsec -= NSEC_PER_SEC;
        wakeup_time.tv_sec++;
      }
      //////////////////////////////////////////////////////////////////////////////////////////////

      return tau_d_array;
    };

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(impedance_control_callback);
  }
  catch (const franka::Exception &ex)
  {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  return 0;
}