// #include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

// #ifdef _MAXON_FOUND_
// #include <maxon_epos_ethercat_sdk/Maxon.hpp>
// #endif
// // System libraries
// #include <algorithm>
// #include <array>
// #include <chrono>
// #include <cmath>
// #include <csignal>
// // #include <eigen3/Eigen/Dense>
// #include <functional>
// #include <iostream>
// #include <mutex>
// #include <string>
// #include <thread>

// // // Franka Emika libraries
// // #include <franka/duration.h>
// // #include <franka/exception.h>
// // #include <franka/model.h>
// // #include <franka/robot.h>

// // // Custom libraries
// // #include "configUtils.hh"
// // #include "endoWrist.hh"
// // #include "examples_common.h"
// // #include "frankaEmika.hh"
// // // #include "PandaKinematics_multi.hh"
// // // #include "Recorder.hpp"
// // #include "robot.hh"
// // #include "udpUtils.hh"

// // Interprocess communication libraries
// #include "multiProcessingUtils.hh"

// // Default paramters
// #define MAX_SIZE 255
// #define ROBOT_TO_CONSOLE_RATIO 1.0
// #define CONFIG_PATH "config/robot_map.xml"
// #define MAX_RAD_PER_SEC 80.0 / 60.0 * 2.0 * M_PI
// #define MIN_RAD_PER_SEC -80.0 / 60.0 * 2.0 * M_PI
// #define MOTOR_ENDOWRIST_PULLEY_RADIUS 0.048  // 0.008 // m
// #define HAPTIC_CONSOLE_MESSAGE_SIZE 21

// // Namespaces
// using namespace std;
// using namespace literals::chrono_literals;
// using namespace boost::interprocess;

// // Types
// typedef enum { SUCCESS, ERR_INIT, ERR_RUN, ERR_EXIT } AppResultState;

// // Global variables
// shared_memory_object shm(open_only, "PandaToEndoWristMemory", read_write);
// mapped_region region(shm, read_write);

// // struct shm_remove {
// //     shm_remove() { shared_memory_object::remove("EndoWristToPandaMemory"); }
// //     ~shm_remove() { shared_memory_object::remove("EndoWristToPandaMemory"); }
// // } remover;

// // // shared_memory_object::remove("EndoWristToPandaMemory");
// // shared_memory_object shm(create_only, "EndoWristToPandaMemory", read_write);
// // shm.truncate(PANDA_TO_ENDOWRIST_MEMORY_SIZE);
// // mapped_region region(shm, read_write);

// // std::memset(region.get_address(), 0, PANDA_TO_ENDOWRIST_MEMORY_SIZE);

// // Threads
// // std::unique_ptr<std::thread> RCM_kinematic_priority_thread;
// // std::unique_ptr<std::thread> controller_thread;
// std::unique_ptr<std::thread> worker_thread;

// // Control variables
// bool abrt = false;
// EthercatDeviceConfigurator::SharedPtr configurator;
// unsigned int counter = 0;

// // Functions
// // Worker function
// void worker(bool* startFlag, bool* startMove, double* dq0, double* dq1, double* dq2, double* dq3);

// // Signal handler function: Handle the interrupt signal. This is the shutdown routine. Note: This logic is executed in a
// // thread separated from the communication update!
// void signal_handler(int sig) {
//     // Pre shutdown procedure.
//     // The devices execute procedures (e.g. state changes) that are necessary for a
//     // proper shutdown and that must be done with PDO communication.
//     // The communication update loop (i.e. PDO loop) continues to run!
//     // You might thus want to implement some logic that stages zero torque / velocity commands
//     // or simliar safety measures at this point using e.g. atomic variables and checking them
//     // in the communication update loop.

//     for (const auto& master : configurator->getMasters()) {
//         master->preShutdown();
//     }
//     // stop the PDO communication at the next update of the communication loop
//     abrt = true;
//     // RCM_kinematic_priority_thread->join();
//     // controller_thread->join();
//     worker_thread->join();
//     // Completely halt the EtherCAT communication.
//     // No online communication is possible afterwards, including SDOs.
//     for (const auto& master : configurator->getMasters()) {
//         master->shutdown();
//     }
//     // Exit this executable
//     std::cout << "Shutdown" << std::endl;
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     exit(0);
// }

// // Program entry. Pass the path to the setup.yaml file as first command line argument.
// int main(int argc, char** argv) {
//     // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)

//     if (argc < 2) {
//         std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
//         return EXIT_FAILURE;
//     }
//     std::signal(SIGINT, signal_handler);

//     // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
//     configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);

//     /*
//     ** Add callbacks to the devices that support them.
//     ** If you don't want to use callbacks this part can simply be left out.
//     ** configurator->getSlavesOfType is another way of extracting only the evices
//     ** of a ceratin type.

//     ** Start all masters.
//     ** There is exactly one bus per master which is also started.
//     ** All online (i.e. SDO) configuration is done during this call.
//     ** The EtherCAT interface is active afterwards, all drives are in Operational
//     ** EtherCAT state and PDO communication may begin.
//     */
//     for (auto& master : configurator->getMasters()) {
//         if (!master->startup()) {
//             std::cerr << "Startup not successful." << std::endl;
//             return EXIT_FAILURE;
//         }
//     }

//     // Start the PDO loop in a new thread.
//     bool startFlag = false, startMove = false;
//     string frankaEmikaIP = "0.0.0.0";
//     double w_x, w_y, w_z, v_gripper, v_x, v_y, v_z;
//     v_x = 0.0;
//     v_y = 0.0;
//     v_z = 0.0;
//     w_x = 0.0;
//     w_y = 0.0;
//     w_z = 0.0;
//     v_gripper = 0.0;

//     worker_thread = std::make_unique<std::thread>(&worker, &startFlag, &startMove, &w_x, &w_y, &w_z, &v_gripper);

//     // Wait for a few PDO cycles to pass. Set anydrives into to ControlOp state (internal state machine, not EtherCAT
//     // states)
//     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     for (auto& slave : configurator->getSlaves()) {
//         std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
//     }
//     std::cout << "Startup finished" << std::endl;
//     pause();
//     return 0;
// }

// // Functions
// // Worker function
// void worker(bool* startFlag, bool* startMove, double* dq0, double* dq1, double* dq2, double* dq3) {
//     bool rtSuccess = true;
//     for (const auto& master : configurator->getMasters()) {
//         rtSuccess &= master->setRealtimePriority(99);
//     }
//     std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.")
//               << std::endl;

// // Flag to set the drive state for the elmos on first startup
// #ifdef _MAXON_FOUND_
//     bool maxonEnabledAfterStartup = false;
//     // bool maxonOperational = false;
// #endif
//     /*
//     ** The communication update loop.
//     ** This loop is supposed to be executed at a constant rate.
//     ** The EthercatMaster::update function incorporates a mechanism
//     ** to create a constant rate.
//     */

//     while (!abrt) {
//         auto start_time = std::chrono::steady_clock::now();
//         /*
//         ** Update each master.
//         ** This sends tha last staged commands and reads the latest readings over EtherCAT.
//         ** The StandaloneEnforceRate update mode is used.
//         ** This means that average update rate will be close to the target rate (if possible).
//         */
//         for (const auto& master : configurator->getMasters()) {
//             master->update(ecat_master::UpdateMode::StandaloneEnforceRate);  // TODO fix the rate compensation (Elmo
//                                                                              // reliability problem)!!
//         }

//         /*
//         ** Do things with the attached devices.
//         ** Your lowlevel control input / measurement logic goes here.
//         ** Different logic can be implemented for each device.
//         */
//         size_t slave_id = 0;
//         for (const auto& slave : configurator->getSlaves()) {
//             // Anydrive
//             if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Anydrive) {
//                 // #ifdef _ANYDRIVE_FOUND_
//                 //                 anydrive::AnydriveEthercatSlave::SharedPtr any_slave_ptr =
//                 //                     std::dynamic_pointer_cast<anydrive::AnydriveEthercatSlave>(slave);
//                 //                 if (any_slave_ptr->getActiveStateEnum() == anydrive::fsm::StateEnum::ControlOp) {
//                 //                     anydrive::Command cmd;
//                 //                     cmd.setModeEnum(anydrive::mode::ModeEnum::MotorVelocity);
//                 //                     cmd.setMotorVelocity(10);
//                 //                     any_slave_ptr->setCommand(cmd);
//                 //                 }
//                 // #endif

//             }
//             // Maxon
//             else if (configurator->getInfoForSlave(slave).type ==
//                      EthercatDeviceConfigurator::EthercatSlaveType::Maxon) {
// #ifdef _MAXON_FOUND_

//                 // Keep constant update rate
//                 // auto start_time = std::chrono::steady_clock::now();

//                 std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

//                 if (!maxonEnabledAfterStartup) {
//                     // Set maxons to operation enabled state, do not block the call!
//                     maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
//                 }

//                 // set commands if we can

//                 if (maxon_slave_ptr->lastPdoStateChangeSuccessful() &&
//                     maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::OperationEnabled) {
//                     maxon::Command command;
//                     // command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
//                     command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode);
//                     auto reading = maxon_slave_ptr->getReading();
//                     double actualVelocity = reading.getActualVelocity();
//                     double actualPosition = reading.getActualPosition();
//                     double actualTorque = reading.getActualTorque();
//                     double* w_vec = static_cast<double*>(region.get_address());
//                     if (maxon_slave_ptr->getName() == "Maxon0") {
//                         command.setTargetVelocity(*(w_vec));
//                         std::memcpy(region.get_address() + MOTOR_0_ACTUAL_VELOCITY_OFFSET, &actualVelocity, 8);
//                         std::memcpy(region.get_address() + MOTOR_0_ACTUAL_POSITION_OFFSET, &actualPosition, 8);
//                         std::memcpy(region.get_address() + MOTOR_0_ACTUAL_TORQUE_OFFSET, &actualTorque, 8);
//                         // double maxon0Error = *(w_vec)-reading.getActualVelocity();
//                         // double maxon0NormalisedError = maxon0Error / (*(w_vec));
//                         // std::cout << "actual position = reading.getActualPosition() = " <<
//                         // reading.getActualPosition()
//                         //           << "\n";
//                         // std::cout << "actual torque = reading.getActualTorque() = " << reading.getActualTorque()
//                         //           << "\n";
//                         // std::cout << "actual velocity = reading.getActualVelocity() = " <<
//                         // reading.getActualVelocity()
//                         //           << "\n";
//                         // std::cout << "target velocity = setTargetVelocity() = " << *(w_vec) << "\n";
//                         // std::cout << "velocity error = setTargetVelocity() - reading.getActualVelocity() = "
//                         //           << maxon0Error << "\n";
//                         // std::cout << "normalised velocity error = " << maxon0NormalisedError << "\n";
//                     } else if (maxon_slave_ptr->getName() == "Maxon1") {
//                         command.setTargetVelocity(*(w_vec + 1));
//                         std::memcpy(region.get_address() + MOTOR_1_ACTUAL_VELOCITY_OFFSET, &actualVelocity, 8);
//                         std::memcpy(region.get_address() + MOTOR_1_ACTUAL_POSITION_OFFSET, &actualPosition, 8);
//                         std::memcpy(region.get_address() + MOTOR_1_ACTUAL_TORQUE_OFFSET, &actualTorque, 8);
//                     } else if (maxon_slave_ptr->getName() == "Maxon2") {
//                         command.setTargetVelocity(*(w_vec + 2));
//                         std::memcpy(region.get_address() + MOTOR_2_ACTUAL_VELOCITY_OFFSET, &actualVelocity, 8);
//                         std::memcpy(region.get_address() + MOTOR_2_ACTUAL_POSITION_OFFSET, &actualPosition, 8);
//                         std::memcpy(region.get_address() + MOTOR_2_ACTUAL_TORQUE_OFFSET, &actualTorque, 8);
//                     } else if (maxon_slave_ptr->getName() == "Maxon3") {
//                         command.setTargetVelocity(*(w_vec + 3));
//                         std::memcpy(region.get_address() + MOTOR_3_ACTUAL_VELOCITY_OFFSET, &actualVelocity, 8);
//                         std::memcpy(region.get_address() + MOTOR_3_ACTUAL_POSITION_OFFSET, &actualPosition, 8);
//                         std::memcpy(region.get_address() + MOTOR_3_ACTUAL_TORQUE_OFFSET, &actualTorque, 8);
//                     } else
//                         std::cout << "Wrong slave name\n";

//                     maxon_slave_ptr->stageCommand(command);
//                 } else {
//                     MELO_WARN_STREAM("Maxon '" << maxon_slave_ptr->getName()
//                                                << "': " << maxon_slave_ptr->getReading().getDriveState());
//                 }

//                 // std::this_thread::sleep_until(start_time + std::chrono::microseconds(1));
// #endif
//             }
//             // Constant update rate
//             std::this_thread::sleep_until(start_time + std::chrono::microseconds(940));
//             auto const delta_time = std::chrono::steady_clock::now() - start_time;
//             std::cout << "delta_time: " << ((float)delta_time.count()) / 1000000.0 << " ms\n";
//         }
//         counter++;
// #ifdef _MAXON_FOUND_
//         maxonEnabledAfterStartup = true;
// #endif
//     }
// }