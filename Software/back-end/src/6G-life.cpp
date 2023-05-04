/**
 * @file 6G-life.cpp
 * @author Gjergji Luarasi (gjergji.luarasi@tum.de)
 * @brief
 * @version 0.1
 * @date 2023-03-26
 *
 * @copyright Copyright (c) 2023
 *
 */
// 6G-life project main file

// System libraries
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

// #include "matplotlibcpp.h"

// Franka Emika libraries
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

// Control systems libraries
#include "PandaKinematics_multi.hh"   //To control maxon motors and franka seperatly
// #include "PandaKinematics_multi_11DoFs.hh"  //For 11DoFs control with single jacobian
// #include "PandaKinematics_withoutRCMandendo.hh"
#include "Recorder.hpp"
#include "examples_common.h"

// Custom libraries
#include "configUtils.hh"
#include "endoWrist.hh"
#include "frankaEmika.hh"
#include "robot.hh"
#include "udpUtils.hh"

// Interprocess communication libraries
#include "multiProcessingUtils.hh"

// Default paramters
#define MAX_SIZE 255
#define ROBOT_TO_CONSOLE_RATIO 1.5
#define CONFIG_PATH "config/robot_map.xml"
#define MAX_RAD_PER_SEC 80.0 / 60.0 * 2.0 * M_PI
#define MIN_RAD_PER_SEC -80.0 / 60.0 * 2.0 * M_PI
#define MOTOR_ENDOWRIST_PULLEY_RADIUS 0.048  // 0.008 // m
#define HAPTIC_CONSOLE_MESSAGE_SIZE 21

// Namespaces
// namespace plt = matplotlibcpp;
using namespace std;
using namespace literals::chrono_literals;

// Types
typedef enum AppState { ERROR, INIT, RUN, EXIT } AppState;

typedef enum AppResultState { SUCCESS, ERR_INIT, ERR_RUN, ERR_EXIT } AppResultState;

// Old functions
AppResultState endoWristControlFunc(bool *startFlag, bool *startMove, double *dq0, double *dq1, double *dq2,
                                    double *dq3);
// Functions
// Receive UDP function
AppResultState receiveUDP(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
                          double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper);
// Control function

 AppResultState RCM_multipriority(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
                              double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper);

// AppResultState RCM_multipriority_11DoFs(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x,
//                                        double *v_y, double *v_z, double *w_x, double *w_y, double *w_z,
//                                       double *v_gripper);

// AppResultState RCM_withoutRCMandendo(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
//                                      double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper);

// Send UDP function
void endoWristSendUDP(bool *startFlag, bool *startMove, double *dq0, double *dq1, double *dq2, double *dq3);

// Load configuration function
map<uint, robot_t> loadConfiguration(void);
map<uint, robot_t> robotMap = loadConfiguration();

// Main program
int main(int argc, char **argv) {
    bool startFlag = false, startMove = false;
    string frankaEmikaIP = "0.0.0.0";
    double w_x, w_y, w_z, v_gripper, v_x, v_y, v_z;
    v_x = 0.0;
    v_y = 0.0;
    v_z = 0.0;
    w_x = 0.0;
    w_y = 0.0;
    w_z = 0.0;
    v_gripper = 0.0;

    thread receiveUDP_thread(receiveUDP, &frankaEmikaIP, &startFlag, &startMove, &v_x, &v_y, &v_z, &w_x, &w_y, &w_z,
                             &v_gripper);
     thread frankaControl(RCM_multipriority, &frankaEmikaIP, &startFlag, &startMove, &v_x, &v_y, &v_z, &w_x, &w_y,
    &w_z,&v_gripper);

    // thread frankaControl(RCM_multipriority_11DoFs, &frankaEmikaIP, &startFlag, &startMove, &v_x, &v_y, &v_z, &w_x,
    // &w_y,
    //                     &w_z, &v_gripper);

    // thread frankaControl(RCM_withoutRCMandendo, &frankaEmikaIP, &startFlag, &startMove, &v_x, &v_y, &v_z, &w_x, &w_y,
    //                      &w_z, &v_gripper);
    thread endoWristSendUDP_thread(endoWristSendUDP, &startFlag, &startMove, &w_x, &w_y, &w_z, &v_gripper);

    // thread csvTrajectory(csvTrajectoryFunc, &startFlag, &startMove, &w_x, &w_y, &w_z,
    //                      &v_gripper);
    // thread endoWristControl(endoWristControlFunc, &startFlag, &startMove, &w_x, &w_y,
    //                         &w_z, &v_gripper);
    receiveUDP_thread.join();
    frankaControl.join();
    endoWristSendUDP_thread.join();
    // csvTrajectory.join();
    // endoWristControl.join();
    return 0;
}

// Old functions
AppResultState endoWristControlFunc(bool *startFlag, bool *startMove, double *dq0, double *dq1, double *dq2,
                                    double *dq3) {
    try {
        // EndoWrist object instantiation
        MotorControllerParameters leftEndoWristYawMotorParams = {.position = 0, .velocity = 0, .current = 0};
        MotorControllerParameters leftEndoWristWristMotorParams = {.position = 0, .velocity = 0, .current = 0};
        MotorControllerParameters leftEndoWristLeftGripperMotorParams = {.position = 0, .velocity = 0, .current = 0};
        MotorControllerParameters leftEndoWristRightGripperMotorParams = {.position = 0, .velocity = 0, .current = 0};
        EndoWrist leftEndoWrist(robotMap[0].Tool.ID, robotMap[0].Tool.Name, robotMap[0].Tool.Description,
                                leftEndoWristYawMotorParams, leftEndoWristWristMotorParams,
                                leftEndoWristLeftGripperMotorParams, leftEndoWristRightGripperMotorParams);
        cout << "leftEndoWrist.name: " << leftEndoWrist.name << endl
             << "leftEndoWrist.description: " << leftEndoWrist.description << endl
             << "leftEndoWrist._ID: " << leftEndoWrist.getID() << endl;
        cout << "EndoWrist object loaded." << endl;
        runProgramResult runProgramRes = initProgram();

        cout << "EndoWrist system initialization finished with result: "
             << ((runProgramRes == runProgramResult::RUNNING)                    ? "RUNNING"
                 : (runProgramRes == runProgramResult::ERROR_INIT_MOTORS)        ? "ERROR_INIT_MOTORS"
                 : (runProgramRes == runProgramResult::ERROR_SCHED_SETSCHEDULER) ? "ERROR_SCHED_SETSCHEDULER"
                 : (runProgramRes == runProgramResult::ERROR_MEMORY_LOCK)        ? "ERROR_MEMORY_LOCK"
                                                                                 : "ERROR_SET_COMMAND")
             << endl;
        struct timespec wakeup_time;
        int ret = 0;
        /* Set priority */
        struct sched_param param = {};
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        printf("Using priority %i.", param.sched_priority);
        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            perror("sched_setscheduler failed");
        }

        /* Lock memory */

        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            fprintf(stderr, "Warning: Failed to lock memory: %s\n", strerror(errno));
        }

        stack_prefault();

        printf("Starting RT task with dt=%u ns.\n", PERIOD_NS);
        // Set command result returns the status of the operating commands (PPM, PVM, CPM, CVM or CTM)
        static setCommandResult setCommandRes = setCommandResult::ERROR_SET_C;
        clock_gettime(CLOCK_MONOTONIC, &wakeup_time);
        wakeup_time.tv_sec += 1; /* start in future */
        wakeup_time.tv_nsec = 0;
        while (true) {
            ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
            if (ret) {
                fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
                break;
            }
            // if(*startFlag)  cout << "EndoWrist velocities: " << *dq0 << ", " << *dq1 << ", " << *dq2 << ", " << *dq3
            // << endl;
            if (*startMove) setCommandRes = doCyclicSynchronousVelocity(&wakeup_time, *dq0, *dq1, *dq2, *dq3);
            // else clearInitialErrors(startMove);

            // cout << "dq0 - dq1 - dq2 - dq3: " << *dq0 << *dq1<< *dq2 << *dq3 << endl;
            wakeup_time.tv_nsec += PERIOD_NS;
            while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
                wakeup_time.tv_nsec -= NSEC_PER_SEC;
                wakeup_time.tv_sec++;
            }
        }

        return SUCCESS;
    } catch (int ErrorCode) {
        // print exception
        cout << "Error during endoWrist control " << endl;
        return ERR_EXIT;
    }
}

// Functions
// Receive UDP function
AppResultState receiveUDP(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
                          double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper) {
    // Initialize control variables
    *v_x = 0.0;
    *v_y = 0.0;
    *v_z = 0.0;
    *w_x = 0.0;
    *w_y = 0.0;
    *w_z = 0.0;
    *v_gripper = 0.0;

    // Instantiate Franka Emika object
    // FrankaEmika leftFR3 = loadFrankaEmika(robotMap);  // Load FrankaEmika
    UdpClientServer::UdpServer trajectoryServer("192.16.0.1", 8080);

    *frankaEmikaIP = "172.16.0.2";  // leftFR3.getFrankaIP();
    *startFlag = true;
    // Define haptic console message variable
    double hapticConsoleMessage[HAPTIC_CONSOLE_MESSAGE_SIZE];

    // Start receiving UDP messages
    std::cout << "Starting to receive UDP messages" << std::endl;

    while (true) {
        if (trajectoryServer.recv(hapticConsoleMessage, MAX_SIZE)) {
            *v_x = std ::min(2.0, std ::max(-2.0, hapticConsoleMessage[4])) / ROBOT_TO_CONSOLE_RATIO;
            *v_y = std ::min(2.0, std ::max(-2.0, hapticConsoleMessage[5])) / ROBOT_TO_CONSOLE_RATIO;
            *v_z = std ::min(2.0, std ::max(-2.0, hapticConsoleMessage[6])) / ROBOT_TO_CONSOLE_RATIO;
            *w_x = std ::max(MIN_RAD_PER_SEC, std::min(MAX_RAD_PER_SEC, hapticConsoleMessage[7]));
            *w_y = std ::max(MIN_RAD_PER_SEC, std::min(MAX_RAD_PER_SEC, hapticConsoleMessage[8]));
            *w_z = std ::max(MIN_RAD_PER_SEC, std::min(MAX_RAD_PER_SEC, hapticConsoleMessage[9]));
            *v_gripper = (std ::max(MIN_RAD_PER_SEC, std::min(MAX_RAD_PER_SEC, hapticConsoleMessage[10] /
                                                                                   (MOTOR_ENDOWRIST_PULLEY_RADIUS))));
        } else {
            cout << "not receiving" << endl;
            *v_x = 0.0;
            *v_y = 0.0;
            *v_z = 0.0;
            *w_x = 0.0;
            *w_y = 0.0;
            *w_z = 0.0;
            *v_gripper = 0.0;
        }
    }
    return SUCCESS;
}

// Control functions

// Control function for RCM with multipriority and maxon motors seperatly (Connected with #include
// "PandaKinematics_multi.hh")

AppResultState RCM_multipriority(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
                                 double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper) {
    std::cout << "Starting RCM_multipriority" << std::endl;

    struct shm_remove {
        shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
        ~shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
    } remover;
    // shared_memory_object::remove("PandaToEndoWristMemory");

    // Create panda to endoWrist shared memory
    shared_memory_object pandaToEndoWristSharedMemoryObject(create_only, "PandaToEndoWristMemory", read_write);
    pandaToEndoWristSharedMemoryObject.truncate(SHARED_MEMORY_SIZE);
    mapped_region pandaToEndoWristMappedRegion(pandaToEndoWristSharedMemoryObject, read_write);
    std::memset(pandaToEndoWristMappedRegion.get_address(), 0, SHARED_MEMORY_SIZE);

    // Create endoWrist to panda shared memory
    // shared_memory_object endoWristToPandaSharedMemoryObject(open_only, "EndoWristToPandaMemory", read_only);
    // mapped_region endoWristToPandaMappedRegion(endoWristToPandaSharedMemoryObject, read_only);

    try {
        // Initialize control variables
        *v_x = 0.0;
        *v_y = 0.0;
        *v_z = 0.0;
        *w_x = 0.0;
        *w_y = 0.0;
        *w_z = 0.0;
        *v_gripper = 0.0;
        Vector3d dq_endoWrist;
        dq_endoWrist << *w_x, *w_y, 2.0 * (*w_z + *v_gripper);
        const double print_rate = 1.0;
        double run_time = 20.0;
        double loop_time = 0.0;
        // recording data
        int index = 0;
        double t_rec = run_time;
        double SampletimeInit = 0.001;
        int NODataRec = 44;
        string name = "DATA";

        Recorder rec(t_rec, SampletimeInit, NODataRec, name);
        // FrankaEmika object instantiation
        // FrankaEmika leftFR3(
        //     robotMap[0].ID, robotMap[0].Name, robotMap[0].Description, robotMap[0].FrankaIP, robotMap[0].PcUdpIP,
        //  robotMap[0].PcUdpPort,  // PC UDP IP and port used for controlling the EndoWrist from the haptic console
        //     robotMap[0].PlotFlag);
        // cout << "FrankaEmika object loaded." << endl;

        // if (leftFR3.plotFlag) {
        //     cout << "leftFR3.name: " << leftFR3.name << endl
        //          << "leftFR3.description: " << leftFR3.description << endl
        //          << "leftFR3._ID: " << leftFR3.getID() << endl
        //          << "leftFR3._frankatIP: " << leftFR3.getFrankaIP() << endl
        //          << "leftFR3._pcUdpIP: " << leftFR3.getPcUdpIP() << endl
        //          << "leftFR3._pcUdpPort: " << leftFR3.getPcUdpPort() << endl
        //          << "leftFR3.plotFlag: " << leftFR3.plotFlag << endl;
        // }

        // Haptic console to robot arm communication variables
        // double hapticConsoleMessage[HAPTIC_CONSOLE_MESSAGE_SIZE];

        // connect to robot
        while (!*startFlag)
            ;
        std::cout << "Starting franka Robot with libfranka. IP:  " << *frankaEmikaIP << std::endl;
        franka::Robot robot(*frankaEmikaIP);
        std::cout << "Started franka Robot with libfranka successfully" << std::endl;
        setDefaultBehavior(robot);

        // Drive into home position
        array<double, 7> q_goal = {{0, 1 * M_PI / 6, 0, -1 * M_PI / 2.5, -M_PI_2, M_PI_2, -M_PI / 1.5}};
        MotionGenerator motion_generator(0.5, q_goal);
        cout << "WARNING: This example will move the robot! "
             << "Please make sure to have the user stop button at hand !" << endl
             << "Press Enter to continue..." << endl;
        cin.ignore();
        robot.control(motion_generator);

        double eta = 0.4;  // tool_length / 2; // the rcm location
        Vector8d qd_com, q_com, qd_comold;
        array<double, 7> qd_com7, q_com7;
        qd_com.setZero();
        array<double, 7> qmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        array<double, 7> Dqmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Vector3d p_rcm, p_rcm0;
        // double q_null_array[7] = {M_PI / 2, M_PI / 6, 0.0, M_PI / 2, 0.0, -M_PI / 2, 0.0};
        double v_leftEndEffector = 0.0, v_rightEndEffector = 0.0;
        Eigen::Matrix4d XX, XXp;
        Matrix<double, 3, 8> Je3, Jp3, Je3_str;
        Matrix<double, 8, 3> Je3_plus, Jp3_plus, Je3_str_plus;
        Matrix<double, 8, 8> N1, N2;
        Matrix<double, 8, 1> qmax, qmin;
        Matrix<double, 8, 1> obj_func, obj_func_old, obj_funcd;
        Vector5d qdi;
        Vector3d p, p_des, pd_des, p0, v_tooltip, p_tooltip, error_rcm, error_endo;
        Matrix3d eye3;
        Matrix7d eye7;
        Matrix<double, 3, 7> Je3_7;
        Matrix<double, 7, 3> Je3_7_tranpose, Je3_7_str;
        Matrix<double, 7, 7> joint_mass;
        Matrix<double, 3, 3> cartesian_mass;
        double f_x1, f_y1, f_z1, f_x2, f_y2, f_z2;
        Vector7d tau_joints;
        Vector3d F_endo1, F_endo2, F_diff;
        Matrix<double, 8, 8> eye8;
        obj_func_old.setZero();
        p_rcm.setZero();
        p_rcm0.setZero();
        p_des.setZero();
        pd_des.setZero();
        qd_comold.setZero();
        eye7.setIdentity();
        eye3.setIdentity();
        eye8.setIdentity();

        // create model
        franka::Model model = robot.loadModel();
        franka::RobotState initial_state = robot.readOnce();


        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_ini(initial_transform.translation());
        Eigen::Quaterniond orientation_ini(initial_transform.linear());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q0(initial_state.q.data());

        rcmFK(initial_state.q, eta, XX, XXp, Je3, Jp3);
        p0 = XX.block<3, 1>(0, 3);
        p_rcm0 = XXp.block<3, 1>(0, 3);
        p_tooltip = p0;
        std::cout << "q0=" << q0.transpose() << std::endl;
        std::cout << "p0=" << p0.transpose() << std::endl;
        std::cout << "p_rcm0=" << p_rcm0.transpose() << std::endl;

        q_com << q0, eta;

    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        cout << "Please do not forget to restart power for Maxon motors. If it is already done, press enter again "
             << endl;
        cin.ignore();
        cout << "WARNING: This example will move the robot! "
             << "Entering control loop" << endl;
        //////////////////////////////////////////////////////////////////////////////////////////////
        auto programStartTime = chrono::high_resolution_clock::now();
        auto startlooptime = chrono::system_clock::now();
        function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> vel_callback =
            [&](const franka::RobotState &state, franka::Duration period) -> franka::JointVelocities {
            programStartTime = chrono::high_resolution_clock::now();
            startlooptime = chrono::system_clock::now();
            // Acquire the state
            for (size_t i = 0; i < 7; i++) {
                qmsr[i] = state.q[i];
                Dqmsr[i] = state.dq[i];
            }

            rcmFK(qmsr, eta, XX, XXp, Je3, Jp3);
            p_rcm = XXp.block<3, 1>(0, 3);

            // std::cout << "p_rcm" << p_rcm[0] << " " << p_rcm[1] <<" "<< p_rcm[2] << std::endl;
            // std::cout << "error  " << (p_des - p).transpose() << std::endl;
            p = XX.block<3, 1>(0, 3);
            loop_time += period.toSec();

            //
            // for (int i = 0; i < 7; i++)
            // {
            //   q_com[i] = _q_com[i];
            //   // q_null[i] = q_null_array[i];
            // }
            //

            qmax << 2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5165, 3.0159, 0.59;
            qmin << -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159, 0;

            for (int i = 0; i < 8; i++) {
                obj_func[i] = (-1 / 16) * pow(((q_com[i] - (qmax[i] - qmin[i]) / 2) / (qmax[i] - qmin[i])), 2);
            }
            obj_funcd = (obj_func - obj_func_old) / (0.001);
            obj_func_old = obj_func;

            Jp3_plus = Jp3.transpose() * (Jp3 * Jp3.transpose()).inverse();

            N1 = eye8 - Jp3_plus * Jp3;

            Je3_str = Je3 * N1;

            Je3_str_plus = Je3_str.transpose() * (Je3_str * Je3_str.transpose()).inverse();

            N2 = N1 * (eye8 - Je3_str_plus * Je3_str);

            v_tooltip << *v_x, *v_y, *v_z;
            p_tooltip << p_tooltip + period.toSec() * v_tooltip;

            error_endo = p_tooltip - p;
            error_rcm = p_rcm0 - p_rcm;
            double P_gainrcm = 10;
            double P_gainendo = 10;
            qd_com = Jp3_plus * (P_gainrcm * eye3 * (error_rcm)) +
                     Je3_str_plus * ((v_tooltip + P_gainendo * eye3 * (error_endo)) -
                                     Je3 * Jp3_plus * (P_gainrcm * eye3 * (error_rcm))) +
                     N2 * obj_funcd;
            // size=8 including eta

            q_com = q_com + 1.0 * qd_com * period.toSec();
            eta = q_com[7];
            // jointsVelocityLimit(qd_com, q_com);
            *startMove = true;
            // cout << "*****************p\n" << p << endl;
            // cout << "*****************p_des\n" << p_des << endl;
            // cout << "*****************Jc_plus\n" << Jc_plus << endl;
            // cout << "*****************qdi\n" << qdi << endl;
            // cout << "*****************qd_com\n" << qd_com << endl;

            // double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
            // double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
            // qd_com= 0.0 * qd_com;

            qd_com[0] = std ::min(10.0 * 0.001 + qd_comold[0], std ::max(-10.0 * 0.001 + qd_comold[0], qd_com[0]));
            qd_com[1] = std ::min(10.0 * 0.001 + qd_comold[1], std ::max(-10.0 * 0.001 + qd_comold[1], qd_com[1]));
            qd_com[2] = std ::min(10.0 * 0.001 + qd_comold[2], std ::max(-10.0 * 0.001 + qd_comold[2], qd_com[2]));
            qd_com[3] = std ::min(10.0 * 0.001 + qd_comold[3], std ::max(-10.0 * 0.001 + qd_comold[3], qd_com[3]));
            qd_com[4] = std ::min(10.0 * 0.001 + qd_comold[4], std ::max(-10.0 * 0.001 + qd_comold[4], qd_com[4]));
            qd_com[5] = std ::min(10.0 * 0.001 + qd_comold[5], std ::max(-10.0 * 0.001 + qd_comold[5], qd_com[5]));
            qd_com[6] = std ::min(10.0 * 0.001 + qd_comold[6], std ::max(-10.0 * 0.001 + qd_comold[6], qd_com[6]));
            qd_com[7] = std ::min(10.0 * 0.001 + qd_comold[7], std ::max(-10.0 * 0.001 + qd_comold[7], qd_com[7]));
            qd_comold = qd_com;

            franka::JointVelocities velocities = {qd_com[0], qd_com[1], qd_com[2], qd_com[3],
                                                  qd_com[4], qd_com[5], qd_com[6]};
            // qd_com7[0] = qd_com[0];
            // qd_com7[1] = qd_com[1];
            // qd_com7[2] = qd_com[2];
            // qd_com7[3] = qd_com[3];
            // qd_com7[4] = qd_com[4];
            // qd_com7[5] = qd_com[5];
            // qd_com7[6] = qd_com[6];
            // cout << "[Loop period = " << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000
            //      << "ms]:\n";

            double *motor_0_actualPosition =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_POSITION_OFFSET);
            double *motor_1_actualPosition =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_POSITION_OFFSET);
            double *motor_2_actualPosition =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_POSITION_OFFSET);
            double *motor_3_actualPosition =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_POSITION_OFFSET);
            double *motor_0_actualVelocity =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_VELOCITY_OFFSET);
            double *motor_1_actualVelocity =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_VELOCITY_OFFSET);
            double *motor_2_actualVelocity =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_VELOCITY_OFFSET);
            double *motor_3_actualVelocity =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_VELOCITY_OFFSET);
            double *motor_0_actualTorque =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_TORQUE_OFFSET);
            double *motor_1_actualTorque =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_TORQUE_OFFSET);
            double *motor_2_actualTorque =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_TORQUE_OFFSET);
            double *motor_3_actualTorque =
                static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_TORQUE_OFFSET);

            std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, w_x, 8);
            std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, w_y, 8);
            v_leftEndEffector = *w_z - (*v_gripper*2.5 +  0.001*20*( *v_gripper*2.5 + *motor_3_actualVelocity));
            v_rightEndEffector = -*w_z - (*v_gripper*2.5 +  0.001*20*( *v_gripper*2.5 - *motor_2_actualVelocity));
            std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET,
            &v_leftEndEffector,
                        8);
            std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
                        &v_rightEndEffector, 8);
            dq_endoWrist << *w_x, *w_y, v_leftEndEffector + v_rightEndEffector;
            // if (*motor_3_actualPosition >-0.01)
            // {
            // cout << "Motor 3 velocity dif: " << *motor_3_actualPosition - *v_gripper<< " rad/s,  " << *motor_3_actualPosition
            //      << " rad/s\n";
            //      }

                // read join torques
                tau_joints << state.tau_ext_hat_filtered[0],
                              state.tau_ext_hat_filtered[1], 
                              state.tau_ext_hat_filtered[2], 
                              state.tau_ext_hat_filtered[3], 
                              state.tau_ext_hat_filtered[4], 
                              state.tau_ext_hat_filtered[5], 
                              state.tau_ext_hat_filtered[6];

                // Jacobian calculation
                //Je3_7 = Je3.block<3, 7>(0, 0);
                std::array<double, 42> jacobian_array =
                model.zeroJacobian(franka::Frame::kEndEffector, state);
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
                Je3_7 = jacobian.block<3, 7>(0, 0);
                // Read joint and task space mass matrix
                joint_mass = Eigen::Map<const Eigen::Matrix<double ,7,7>>(model.mass(state).data());
                cartesian_mass = (Je3_7*(joint_mass.inverse())*Je3_7.transpose()).inverse();
                
                // Compute feedback forces
                Je3_7_str = (joint_mass.inverse())*Je3_7.transpose()*cartesian_mass;
                F_endo1 = (Je3_7_str.transpose())*tau_joints;    
                // F_endo2 = (Je3_7 * (Je3_7.transpose() * Je3_7).inverse())*tau_joints;   
                f_x1 = std::min(10.0, std::max(-10.0, (F_endo1[0]+0.6)*0.8));
                f_y1 = std::min(10.0, std::max(-10.0, (F_endo1[1]+0.5)*0.8));
                f_z1 = std::min(10.0, std::max(-10.0, (F_endo1[2]-3.0)*0.8));     
                // f_x2 = std::min(10.0, std::max(-10.0, F_endo2[0]*0.3));
                // f_y2 = std::min(10.0, std::max(-10.0, F_endo2[1]*0.3));
                // f_z2 = std::min(10.0, std::max(-10.0, F_endo2[2]*0.3));    
                F_endo1 << f_x1, f_y1, f_z1;
                // F_endo2 << f_x2, f_y2, f_z2;
                F_diff << f_x1-f_x2, f_y1-f_y2, f_z1-f_z2;
            if (loop_time < t_rec) {
                // check NODataRec to be consistent
                rec.addToRec(index);
                rec.addToRec(loop_time);
                rec.addToRec(qmsr);
                rec.addToRec(Dqmsr);
                rec.addToRec(p_rcm);
                rec.addToRec(p_tooltip);
                rec.addToRec(p);
                rec.addToRec(eta);
                rec.addToRec(F_endo1);
            //    rec.addToRec(F_endo2);
                rec.addToRec(state.tau_ext_hat_filtered);
                rec.addToRec(F_diff);
                //rec.addToRec(dq_endoWrist);
                // rec.addToRec(*motor_0_actualPosition);
                // rec.addToRec(*motor_1_actualPosition);
                // rec.addToRec(*motor_2_actualPosition);
                // rec.addToRec(*motor_3_actualPosition);
                // rec.addToRec(*motor_0_actualVelocity);
                // rec.addToRec(*motor_1_actualVelocity);
                // rec.addToRec(*motor_2_actualVelocity);
                // rec.addToRec(*motor_3_actualVelocity);
                // rec.addToRec(*motor_0_actualTorque);
                // rec.addToRec(*motor_1_actualTorque);
                // rec.addToRec(*motor_2_actualTorque);
                // rec.addToRec(*motor_3_actualTorque);
                rec.next();
                index++;
            }
            return velocities;
        };

        *startFlag = true;
        robot.control(vel_callback);

        return SUCCESS;
    } catch (const franka::Exception &ex) {
        const double w_x_safety = 0.0;
        const double w_y_safety = 0.0;
        const double v_leftEndEffector_safety = 0.0;
        const double v_rightEndEffector_safety = 0.0;
        std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, &w_x_safety, 8);
        std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, &w_y_safety, 8);
        std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET,
                    &v_leftEndEffector_safety, 8);
        std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
                    &v_rightEndEffector_safety, 8);
        // print exception
        cout << ex.what() << endl;
        return ERR_EXIT;
    }
}

// Control Function for all 11 DoFs with single Jacobian ((Connected with #include "PandaKinematics_multi_11DoFs.hh"))

// AppResultState RCM_multipriority_11DoFs(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x,
//                                         double *v_y, double *v_z, double *w_x, double *w_y, double *w_z,
//                                         double *v_gripper) {
//     std::cout << "Starting RCM_multipriority for all DoFs" << std::endl;

//     struct shm_remove {
//         shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
//         ~shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
//     } remover;
//     // shared_memory_object::remove("PandaToEndoWristMemory");

//     // Create panda to endoWrist shared memory
//     shared_memory_object pandaToEndoWristSharedMemoryObject(create_only, "PandaToEndoWristMemory", read_write);
//     pandaToEndoWristSharedMemoryObject.truncate(SHARED_MEMORY_SIZE);
//     mapped_region pandaToEndoWristMappedRegion(pandaToEndoWristSharedMemoryObject, read_write);
//     std::memset(pandaToEndoWristMappedRegion.get_address(), 0, SHARED_MEMORY_SIZE);

//     // Create endoWrist to panda shared memory
//     // shared_memory_object endoWristToPandaSharedMemoryObject(open_only, "EndoWristToPandaMemory", read_only);
//     // mapped_region endoWristToPandaMappedRegion(endoWristToPandaSharedMemoryObject, read_only);

//     try {
//         // Initialize control variables
//         *v_x = 0.0;
//         *v_y = 0.0;
//         *v_z = 0.0;
//         *w_x = 0.0;
//         *w_y = 0.0;
//         *w_z = 0.0;
//         *v_gripper = 0.0;
//         Vector3d dq_endoWrist_ref;
//         dq_endoWrist_ref << *w_x, *w_y, *w_z;
//         const double print_rate = 1.0;
//         double run_time = 20.0;
//         double loop_time = 0.0;
//         // recording data
//         int index = 0;
//         double t_rec = run_time;
//         double SampletimeInit = 0.001;
//         int NODataRec = 44;
//         string name = "DATA";

//         Recorder rec(t_rec, SampletimeInit, NODataRec, name);
//         // FrankaEmika object instantiation
//         // FrankaEmika leftFR3(
//         //     robotMap[0].ID, robotMap[0].Name, robotMap[0].Description, robotMap[0].FrankaIP, robotMap[0].PcUdpIP,
//         //     robotMap[0].PcUdpPort,  // PC UDP IP and port used for controlling the EndoWrist from the haptic
//         console
//         //     robotMap[0].PlotFlag);
//         // cout << "FrankaEmika object loaded." << endl;

//         // if (leftFR3.plotFlag) {
//         //     cout << "leftFR3.name: " << leftFR3.name << endl
//         //          << "leftFR3.description: " << leftFR3.description << endl
//         //          << "leftFR3._ID: " << leftFR3.getID() << endl
//         //          << "leftFR3._frankatIP: " << leftFR3.getFrankaIP() << endl
//         //          << "leftFR3._pcUdpIP: " << leftFR3.getPcUdpIP() << endl
//         //          << "leftFR3._pcUdpPort: " << leftFR3.getPcUdpPort() << endl
//         //          << "leftFR3.plotFlag: " << leftFR3.plotFlag << endl;
//         // }

//         // Haptic console to robot arm communication variables
//         // double hapticConsoleMessage[HAPTIC_CONSOLE_MESSAGE_SIZE];

//         // connect to robot
//         while (!*startFlag)
//             ;
//         std::cout << "Starting franka Robot with libfranka. IP:  " << *frankaEmikaIP << std::endl;
//         franka::Robot robot(*frankaEmikaIP);
//         std::cout << "Started franka Robot with libfranka successfully" << std::endl;
//         setDefaultBehavior(robot);

//         // Drive into home position
//         array<double, 7> q_goal = {{0, 1 * M_PI / 6, 0, -1 * M_PI / 2.5, -M_PI_2, M_PI_2, -M_PI / 1.5}};
//         MotionGenerator motion_generator(0.5, q_goal);
//         cout << "WARNING: DO NOT FORGET RESET MAXON MOTORS POWER TO RESET ENCODERS! " << endl
//              << "WARNING: This example will move the robot! "
//              << "Please make sure to have the user stop button at hand !" << endl
//              << "Press Enter to continue..." << endl;
//         cin.ignore();
//         robot.control(motion_generator);

//         double eta = 0.45;  // tool_length / 2; // the rcm location
//         Vector11d qd_com, q_com, qd_comold;
//         array<double, 7> qd_com7, q_com7;
//         qd_com.setZero();
//         array<double, 7> qmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//         array<double, 7> Dqmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//         std::array<double, 3> qmsr_maxon{0.0, 0.0, 0.0};
//         std::array<double, 3> Dqmsr_maxon = {{0.0, 0.0, 0.0}};
//         Vector3d p_rcm, p_rcm0;
//         // double q_null_array[7] = {0, M_PI / 6, 0.0, M_PI / 2, 0.0, -M_PI / 2, 0.0};
//         Eigen::Matrix4d XX, XXp;
//         Matrix<double, 6, 11> Je6, Je6_str;
//         Matrix<double, 3, 11> Jp3;
//         Matrix<double, 11, 6> Je3_plus, Je6_str_plus;
//         Matrix<double, 11, 3> Jp3_plus;
//         Matrix<double, 11, 11> N1, N2;
//         Matrix<double, 11, 1> qmax, qmin;
//         Matrix<double, 11, 1> obj_func, obj_func_old, obj_funcd;
//         Vector5d qdi;
//         Vector3d p, p_des, pd_des, p0, error_rcm;
//         Vector6d error_endo;
//         Vector3d error_endo_ori;
//         Matrix3d eye3, Skew_epsd;
//         Matrix6d eye6;
//         Matrix7d eye7;
//         Matrix11d eye11;
//         Matrix<double, 8, 8> eye8;
//         p_rcm.setZero();
//         p_rcm0.setZero();
//         p_des.setZero();
//         pd_des.setZero();
//         eye7.setIdentity();
//         eye3.setIdentity();
//         eye8.setIdentity();
//         eye6.setIdentity();
//         eye11.setIdentity();
//         Vector3d Error2;
//         Vector3d p_tooltip_linear;
//         Vector3d v_tooltip_linear;
//         Vector3d p_tooltip_angular;
//         Vector3d v_tooltip_angular;
//         Vector3d w_tooltip_haptic;
//         Vector6d p_tooltip;
//         Vector6d v_tooltip;
//         p_tooltip.setZero();
//         v_tooltip.setZero();
//         double epsd_x;
//         double epsd_y;
//         double epsd_z;
//         double etad_w;
//         double epsd_x_dot;
//         double epsd_y_dot;
//         double epsd_z_dot;
//         double etad_w_dot;

//         double epse_x;
//         double epse_y;
//         double epse_z;
//         double etae_w;

//         Matrix<double, 3, 1> eps_e, eps_d, epsd_dot;

//         franka::RobotState initial_state = robot.readOnce();

//         Eigen::Map<const Eigen::Matrix<double, 7, 1>> q0(initial_state.q.data());

//         std::cout << "q0: " << q0 << "\n";

//         // Reading Maxon Motors data
//         std::array<double, 3> q0_maxon{0.0, 0.0, 0.0};

//         rcmFK(initial_state.q, eta, XX, XXp, Je6, Jp3, q0_maxon);
//         p0 = XX.block<3, 1>(0, 3);
//         p_rcm0 = XXp.block<3, 1>(0, 3);
//         p_tooltip_linear = p0;

//         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(XX.data()));
//         Eigen::Quaterniond orientation_d(initial_transform.linear());

//         epsd_x = orientation_d.x();
//         epsd_y = orientation_d.y();
//         epsd_z = orientation_d.z();
//         etad_w = orientation_d.w();

//         Skew_epsd << 0.0, -epsd_z, epsd_y, epsd_z, 0.0, -epsd_x, -epsd_y, epsd_x, 0.0;

//         std::cout << "q0=" << q0.transpose() << std::endl;
//         std::cout << "p0=" << p0.transpose() << std::endl;
//         std::cout << "p_rcm0=" << p_rcm0.transpose() << std::endl;
//         std::cout << "q_maxon=" << q0_maxon[0] << " " << q0_maxon[1] << " " << q0_maxon[2] << std::endl;
//         std::cout << "orientation in quar.=" << epsd_x << " " << epsd_y << " " << epsd_z << std::endl;
//         // std::cout << "rotmatrcies first.=" << XX << std::endl;
//         q_com << q0, eta, q0_maxon[0], q0_maxon[1], q0_maxon[2];

//         robot.setCollisionBehavior(
//             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//         cout << "Please do not forget to restart power for Maxon motors. If it is already done, press enter again "
//              << endl;
//         cin.ignore();
//         cout << "WARNING: This example will move the robot! "
//              << "Entering control loop" << endl;
//         //////////////////////////////////////////////////////////////////////////////////////////////
//         auto programStartTime = chrono::high_resolution_clock::now();
//         auto startlooptime = chrono::system_clock::now();
//         function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> vel_callback =
//             [&](const franka::RobotState &state, franka::Duration period) -> franka::JointVelocities {
//             programStartTime = chrono::high_resolution_clock::now();
//             startlooptime = chrono::system_clock::now();
//             // Acquire the state
//             for (size_t i = 0; i < 7; i++) {
//                 qmsr[i] = state.q[i];
//                 Dqmsr[i] = state.dq[i];
//             }
//             // std::cout << "franka position=" << qmsr[0] << " " << qmsr[1] << " " << qmsr[2] << qmsr[3] << " " <<
//             // qmsr[4]
//             //           << qmsr[5] << " " << qmsr[6] << "\n";

//             //----Read Maxon Motor Speed and position---------------------------------------------------------------
//             double *motor_0_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_POSITION_OFFSET);
//             double *motor_1_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_POSITION_OFFSET);
//             double *motor_2_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_POSITION_OFFSET);
//             double *motor_3_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_POSITION_OFFSET);
//             double *motor_0_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_VELOCITY_OFFSET);
//             double *motor_1_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_VELOCITY_OFFSET);
//             double *motor_2_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_VELOCITY_OFFSET);
//             double *motor_3_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_VELOCITY_OFFSET);
//             double *motor_0_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_TORQUE_OFFSET);
//             double *motor_1_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_TORQUE_OFFSET);
//             double *motor_2_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_TORQUE_OFFSET);
//             double *motor_3_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_TORQUE_OFFSET);
//             qmsr_maxon[0] = *motor_0_actualPosition;
//             qmsr_maxon[1] = *motor_1_actualPosition;
//             qmsr_maxon[2] = *motor_2_actualPosition;
//             Dqmsr_maxon[0] = *motor_0_actualVelocity;
//             Dqmsr_maxon[1] = *motor_1_actualVelocity;
//             Dqmsr_maxon[2] = *motor_2_actualVelocity;
//             // qmsr_maxon[0] = 0.0;
//             // qmsr_maxon[1] = 0.0;
//             // qmsr_maxon[2] = 0.0;
//             // Dqmsr_maxon[0] = 0.0;
//             // Dqmsr_maxon[1] = 0.0;
//             // Dqmsr_maxon[2] = 0.0;
//             //----Read Maxon Motor Speed and Position-------------------------------------------------
//             // std::cout << "maxon position=" << qmsr_maxon[0] << " " << qmsr_maxon[1] << " " << qmsr_maxon[2]
//             //           << std::endl;
//             // std::cout << "REF=" << *w_x << std::endl;
//             rcmFK(qmsr, eta, XX, XXp, Je6, Jp3, qmsr_maxon);

//             p_rcm = XXp.block<3, 1>(0, 3);

//             // std::cout << "p_rcm" << p_rcm[0] << " " << p_rcm[1] <<" "<< p_rcm[2] << std::endl;
//             // std::cout << "error  " << (p_des - p).transpose() << std::endl;
//             p = XX.block<3, 1>(0, 3);

//             // Quartenion part
//             Eigen::Affine3d transform(Eigen::Matrix4d::Map(XX.data()));
//             Eigen::Quaterniond orientation_e(transform.linear());
//             epse_x = orientation_e.x();
//             epse_y = orientation_e.y();
//             epse_z = orientation_e.z();
//             etae_w = orientation_e.w();
//             eps_e = orientation_e.vec();
//             // Quartenion part
//             // std::cout << "current orientation in quar.=" << epse_x << " " << epse_y << " " << epse_z << " " <<
//             etae_w
//             //           << std::endl;
//             // std::cout << "desired orientation in quar.=" << epsd_x << " " << epsd_y << " " << epsd_z << " " <<
//             etad_w
//             //           << std::endl;
//             /// std::cout << "current position=" << p << std::endl;
//             // std::cout << "desired position.=" << p_tooltip_linear << std::endl;
//             // std::cout << "rotmatrcies second.=" << XX << std::endl;
//             loop_time += period.toSec();

//             //
//             // for (int i = 0; i < 7; i++)
//             // {
//             //   q_com[i] = _q_com[i];
//             //   // q_null[i] = q_null_array[i];
//             // }
//             //

//             qmax << 2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5165, 3.0159, 0.59, 1.5708, 1.5708, 1.5708;
//             qmin << -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159, 0, -1.5708, -1.5708, -1.5708;

//             for (int i = 0; i < 11; i++) {
//                 obj_func[i] = (-1 / 22) * pow(((q_com[i] - (qmax[i] - qmin[i]) / 2) / (qmax[i] - qmin[i])), 2);
//             }
//             obj_funcd = (obj_func - obj_func_old) / (0.001);
//             obj_func_old = obj_func;

//             Jp3_plus = Jp3.transpose() * (Jp3 * Jp3.transpose()).inverse();

//             N1 = eye11 - Jp3_plus * Jp3;

//             Je6_str = Je6 * N1;

//             Je6_str_plus = Je6_str.transpose() * (Je6_str * Je6_str.transpose()).inverse();

//             N2 = N1 * (eye11 - Je6_str_plus * Je6_str);

//             v_tooltip_linear << *v_x, *v_y, *v_z;
//             p_tooltip_linear << p_tooltip_linear + period.toSec() * v_tooltip_linear;
//             w_tooltip_haptic << *w_x, *w_y, *w_z;

//             // eps_d << epsd_x, epsd_y, epsd_z;
//             // etad_w_dot = -0.5 * (eps_d.transpose()) * w_tooltip_haptic;
//             // epsd_dot = 0.5 * (etad_w * eye3 - Skew_epsd) * w_tooltip_haptic;

//             Eigen::Quaterniond q_dot;
//             // orientation_d = orientation_e;
//             q_dot.w() = -0.5 * (orientation_d.x() * *w_x + orientation_d.y() * *w_y + orientation_d.z() * *w_z);

//             q_dot.x() = 0.5 * (orientation_d.w() * *w_x + orientation_d.y() * *w_z - orientation_d.z() * *w_y);
//             q_dot.y() = 0.5 * (orientation_d.w() * *w_y + orientation_d.z() * *w_x - orientation_d.x() * *w_z);
//             q_dot.z() = 0.5 * (orientation_d.w() * *w_z + orientation_d.x() * *w_y - orientation_d.y() * *w_x);
//             // cout<< "orientation_d.x()1 = "<<orientation_d.x()<<std::endl;
//             orientation_d.x() = orientation_d.x() + period.toSec() * q_dot.x();
//             orientation_d.y() = orientation_d.y() + period.toSec() * q_dot.y();
//             orientation_d.z() = orientation_d.z() + period.toSec() * q_dot.z();
//             orientation_d.w() = orientation_d.w() + period.toSec() * q_dot.w();
//             // cout<< "orientation_d.x()2= "<<orientation_d.x()<<std::endl;
//             // orientation_d.normalize();
//             // cout<< "orientation_d.x()3= "<<orientation_d.x()<<std::endl;
//             //   eps_d = eps_d + period.toSec() * epsd_dot;
//             //   etad_w = etad_w + period.toSec() * etad_w_dot;
//             etad_w = orientation_d.w();
//             epsd_x = orientation_d.x();
//             epsd_y = orientation_d.y();
//             epsd_z = orientation_d.z();

//             // Eigen::Quaterniond orientation_d(etad_w, epsd_x, epsd_y, epsd_z);
//             // orientation_d = orientation_d.normalized();

//             // Skew_epsd << 0, -epsd_z, epsd_y, epsd_z, 0, -epsd_x, -epsd_y, epsd_x, 0;

//             // if (orientation_d.coeffs().dot(orientation_e.coeffs()) < 0.0) {
//             //     orientation_e.coeffs() << -orientation_e.coeffs();
//             // }

//             Eigen::Quaterniond quar_error(orientation_e.inverse() * orientation_d);
//             Eigen::Quaterniond quar_error2(orientation_d * orientation_e.inverse());

//             if (quar_error.w() < 0) {
//                 quar_error.coeffs() = -quar_error.coeffs();
//             }

//             error_endo.head(3) << p_tooltip_linear - p;

//             v_tooltip << v_tooltip_linear, w_tooltip_haptic;
//             error_endo_ori = etae_w * eps_d - etad_w * eps_e - Skew_epsd * eps_e;

//             error_endo.tail(3) << quar_error.x(), quar_error.y(), quar_error.z();
//             error_endo.tail(3) << -transform.linear() * error_endo.tail(3);
//             error_rcm = p_rcm0 - p_rcm;
//             // error_endo.tail(3) << quar_error2.vec();
//             //  cout << "Error orientation = " << error_endo.tail(3) << "\n";
//             //  cout << "Error orientation2 = " << quar_error2.vec() << "\n";
//             //    cout << "Error orientationxxxx = " << error_endo_ori << "\n";
//             // cout<<-transform.linear()*quar_error.vec()<<"\n";

//             qd_com =
//                 Jp3_plus * (3 * eye3 * (error_rcm)) +
//                 Je6_str_plus * ((v_tooltip + 5 * eye6 * (error_endo)) - Je6 * Jp3_plus * (3 * eye3 * (error_rcm))) +
//                 N2 * obj_funcd;
//             // cout<<"Jp3_plus = "<<Jp3_plus<<"\n";
//             // cout<<"Je6_str_plus = "<<Je6_str_plus<<"\n";
//             // cout<<"Je6 = "<<Je6<<"\n";

//             q_com = q_com + 1.0 * qd_com * period.toSec();

//             eta = q_com[7];
//             // jointsVelocityLimit(qd_com, q_com);
//             *startMove = true;
//             // cout << "*****************p\n" << p << endl;
//             // cout << "*****************p_des\n" << p_des << endl;
//             // cout << "*****************Jc_plus\n" << Jc_plus << endl;
//             // cout << "*****************qdi\n" << qdi << endl;
//             // cout << "*****************qd_com\n" << qd_com << endl;

//             // double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
//             // double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
//             // qd_com= 0.0 * qd_com;

//             qd_com[0] = std ::min(10.0 * 0.001 + qd_comold[0], std ::max(-10.0 * 0.001 + qd_comold[0], qd_com[0]));
//             qd_com[1] = std ::min(10.0 * 0.001 + qd_comold[1], std ::max(-10.0 * 0.001 + qd_comold[1], qd_com[1]));
//             qd_com[2] = std ::min(10.0 * 0.001 + qd_comold[2], std ::max(-10.0 * 0.001 + qd_comold[2], qd_com[2]));
//             qd_com[3] = std ::min(10.0 * 0.001 + qd_comold[3], std ::max(-10.0 * 0.001 + qd_comold[3], qd_com[3]));
//             qd_com[4] = std ::min(10.0 * 0.001 + qd_comold[4], std ::max(-10.0 * 0.001 + qd_comold[4], qd_com[4]));
//             qd_com[5] = std ::min(10.0 * 0.001 + qd_comold[5], std ::max(-10.0 * 0.001 + qd_comold[5], qd_com[5]));
//             qd_com[6] = std ::min(10.0 * 0.001 + qd_comold[6], std ::max(-10.0 * 0.001 + qd_comold[6], qd_com[6]));
//             qd_com[7] = std ::min(10.0 * 0.001 + qd_comold[7], std ::max(-10.0 * 0.001 + qd_comold[7], qd_com[7]));
//             qd_comold = qd_com;

//             franka::JointVelocities velocities = {qd_com[0], qd_com[1], qd_com[2], qd_com[3],
//                                                   qd_com[4], qd_com[5], qd_com[6]};
//             qd_com7[0] = qd_com[0];
//             qd_com7[1] = qd_com[1];
//             qd_com7[2] = qd_com[2];
//             qd_com7[3] = qd_com[3];
//             qd_com7[4] = qd_com[4];
//             qd_com7[5] = qd_com[5];
//             qd_com7[6] = qd_com[6];

//             qd_com[8] = std ::min(13.0, std ::max(-13.0, qd_com[8]));
//             qd_com[9] = std ::min(13.0, std ::max(-13.0, qd_com[9]));
//             qd_com[10] = std ::min(13.0, std ::max(-13.0, qd_com[10]));

//             // cout << "[Loop period = " << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000
//             //      << "ms]:\n";
//             // Set Maxon Motors Input
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, &qd_com[8], 8);
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, &qd_com[9], 8);
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET, &qd_com[10], 8);
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET, &qd_com[10], 8);
//             // std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, w_x, 8);
//             // std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, w_y, 8);
//             // std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET, w_z,
//             //             8);
//             // std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
//             //             w_z, 8);
//             dq_endoWrist_ref << *w_x, *w_y, *w_z;
//             // cout << "Motor 0 parameters: " << *motor_0_actualPosition << " rad,  " << *motor_0_actualVelocity
//             //     << " rad/s\n";
//             // cout << "q1d = " << qd_com[0] << "; "
//             //      << "q2d = " << qd_com[1] << "; "
//             //      << "q3d = " << qd_com[2] << "; "
//             //      << "q4d = " << qd_com[3] << "; "
//             //      << "q5d = " << qd_com[4] << "; "
//             //      << "q6d = " << qd_com[5] << "; "
//             //      << "q7d = " << qd_com[6] << "; "
//             //      << "q8d = " << qd_com[8] << "; "
//             //      << "q9d = " << qd_com[9] << "; "
//             //      << "q10d = " << qd_com[10] << "; "
//             //      << "\n";

//             if (loop_time < t_rec) {
//                 // check NODataRec to be consistent
//                 rec.addToRec(index);
//                 rec.addToRec(loop_time);
//                 rec.addToRec(qmsr);
//                 rec.addToRec(qd_com7);
//                 rec.addToRec(p_rcm);
//                 rec.addToRec(p_tooltip_linear);
//                 rec.addToRec(p);
//                 rec.addToRec(eta);
//                 rec.addToRec(dq_endoWrist_ref);
//                 // rec.addToRec(*motor_0_actualPosition);
//                 // rec.addToRec(*motor_1_actualPosition);
//                 // rec.addToRec(*motor_2_actualPosition);
//                 // rec.addToRec(*motor_3_actualPosition);
//                 // rec.addToRec(*motor_0_actualVelocity);
//                 // rec.addToRec(*motor_1_actualVelocity);
//                 // rec.addToRec(*motor_2_actualVelocity);
//                 // rec.addToRec(*motor_3_actualVelocity);
//                 // rec.addToRec(*motor_0_actualTorque);
//                 // rec.addToRec(*motor_1_actualTorque);
//                 // rec.addToRec(*motor_2_actualTorque);
//                 // rec.addToRec(*motor_3_actualTorque);
//                 // //  rec.addToRec(eps_e);
//                 // rec.addToRec(eps_d);
//                 rec.next();
//                 index++;
//             }
//             return velocities;
//         };

//         *startFlag = true;
//         robot.control(vel_callback);

//         return SUCCESS;
//     } catch (const franka::Exception &ex) {
//         const double w_x_safety = 0.0;
//         const double w_y_safety = 0.0;
//         const double v_leftEndEffector_safety = 0.0;
//         const double v_rightEndEffector_safety = 0.0;
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, &w_x_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, &w_y_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET,
//                     &v_leftEndEffector_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
//                     &v_rightEndEffector_safety, 8);
//         // print exception
//         cout << ex.what() << endl;
//         return ERR_EXIT;
//     }
// }

// AppResultState RCM_withoutRCMandendo(string *frankaEmikaIP, bool *startFlag, bool *startMove, double *v_x, double *v_y,
//                                      double *v_z, double *w_x, double *w_y, double *w_z, double *v_gripper) {
//     std::cout << "Starting RCM_withoutRCMandendo" << std::endl;

//     struct shm_remove {
//         shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
//         ~shm_remove() { shared_memory_object::remove("PandaToEndoWristMemory"); }
//     } remover;
//     // shared_memory_object::remove("PandaToEndoWristMemory");

//     // Create panda to endoWrist shared memory
//     shared_memory_object pandaToEndoWristSharedMemoryObject(create_only, "PandaToEndoWristMemory", read_write);
//     pandaToEndoWristSharedMemoryObject.truncate(SHARED_MEMORY_SIZE);
//     mapped_region pandaToEndoWristMappedRegion(pandaToEndoWristSharedMemoryObject, read_write);
//     std::memset(pandaToEndoWristMappedRegion.get_address(), 0, SHARED_MEMORY_SIZE);

//     // Create endoWrist to panda shared memory
//     // shared_memory_object endoWristToPandaSharedMemoryObject(open_only, "EndoWristToPandaMemory", read_only);
//     // mapped_region endoWristToPandaMappedRegion(endoWristToPandaSharedMemoryObject, read_only);

//     try {
//         // Initialize control variables
//         *v_x = 0.0;
//         *v_y = 0.0;
//         *v_z = 0.0;
//         *w_x = 0.0;
//         *w_y = 0.0;
//         *w_z = 0.0;
//         *v_gripper = 0.0;
//         Vector3d dq_endoWrist;
//         dq_endoWrist << *w_x, *w_y, 2.0 * (*w_z + *v_gripper);
//         const double print_rate = 1.0;
//         double run_time = 20.0;
//         double loop_time = 0.0;
//         // recording data
//         int index = 0;
//         double t_rec = run_time;
//         double SampletimeInit = 0.001;
//         int NODataRec = 44;
//         string name = "DATA";

//         Recorder rec(t_rec, SampletimeInit, NODataRec, name);
//         // FrankaEmika object instantiation
//         // FrankaEmika leftFR3(
//         //     robotMap[0].ID, robotMap[0].Name, robotMap[0].Description, robotMap[0].FrankaIP, robotMap[0].PcUdpIP,
//         //     robotMap[0].PcUdpPort,  // PC UDP IP and port used for controlling the EndoWrist from the haptic console
//         //     robotMap[0].PlotFlag);
//         // cout << "FrankaEmika object loaded." << endl;

//         // if (leftFR3.plotFlag) {
//         //     cout << "leftFR3.name: " << leftFR3.name << endl
//         //          << "leftFR3.description: " << leftFR3.description << endl
//         //          << "leftFR3._ID: " << leftFR3.getID() << endl
//         //          << "leftFR3._frankatIP: " << leftFR3.getFrankaIP() << endl
//         //          << "leftFR3._pcUdpIP: " << leftFR3.getPcUdpIP() << endl
//         //          << "leftFR3._pcUdpPort: " << leftFR3.getPcUdpPort() << endl
//         //          << "leftFR3.plotFlag: " << leftFR3.plotFlag << endl;
//         // }

//         // Haptic console to robot arm communication variables
//         // double hapticConsoleMessage[HAPTIC_CONSOLE_MESSAGE_SIZE];

//         // connect to robot
//         while (!*startFlag)
//             ;
//         std::cout << "Starting franka Robot with libfranka. IP:  " << *frankaEmikaIP << std::endl;
//         franka::Robot robot(*frankaEmikaIP);
//         std::cout << "Started franka Robot with libfranka successfully" << std::endl;
//         setDefaultBehavior(robot);

//         // Drive into home position
//         array<double, 7> q_goal = {{0, 1 * M_PI / 6, 0, -1 * M_PI / 2.5, -M_PI_2, M_PI_2, -M_PI / 1.5}};
//         MotionGenerator motion_generator(0.5, q_goal);
//         cout << "WARNING: This example will move the robot! "
//              << "Please make sure to have the user stop button at hand !" << endl
//              << "Press Enter to continue..." << endl;
//         cin.ignore();
//         robot.control(motion_generator);

//         double eta = 0.4;  // tool_length / 2; // the rcm location
//         Vector7d qd_com, q_com, qd_comold;
//         array<double, 7> qd_com7, q_com7;
//         qd_com.setZero();
//         array<double, 7> qmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//         array<double, 7> Dqmsr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//         // double q_null_array[7] = {M_PI / 2, M_PI / 6, 0.0, M_PI / 2, 0.0, -M_PI / 2, 0.0};
//         double v_leftEndEffector = 0.0, v_rightEndEffector = 0.0;
//         Eigen::Matrix4d XX, XXp;
//         Matrix<double, 6, 7> Je6;
//         Matrix<double, 7, 6> Je6_plus;
//         Matrix<double, 3, 7> Je3;
//         Matrix<double, 7, 3> Je3_plus;
//         Vector3d  p_des, pd_des, error_rcm, v_tooltip_linear, v_tooltip_angular, p_tooltip_linear;
//         Vector6d error_endo, v_tooltip, p_tooltip;
//         Matrix3d eye3;
//         Matrix7d eye7;
//         Matrix6d eye6;
//         Matrix<double, 8, 8> eye8;
//         p_des.setZero();
//         pd_des.setZero();
//         qd_comold.setZero();
//         eye7.setIdentity();
//         eye3.setIdentity();
//         eye8.setIdentity();
//         eye6.setIdentity();
//         franka::Model model = robot.loadModel();
//         franka::RobotState initial_state = robot.readOnce();
        
//         Eigen::Map<const Eigen::Matrix<double, 7, 1>> q0(initial_state.q.data());

//         //rcmFK(initial_state.q, XX, Je6);
       
        
//         Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
//          Eigen::Vector3d p0(initial_transform.translation());
//         Eigen::Quaterniond orientation_d(initial_transform.rotation());
//         p_tooltip_linear = p0;
//         std::cout << "q0=" << q0.transpose() << std::endl;
//         std::cout << "p0=" << p0.transpose() << std::endl;
//         q_com << q0;

//         robot.setCollisionBehavior(
//             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
//             {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}}, {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

//         cout << "Please do not forget to restart power for Maxon motors. If it is already done, press enter again "
//              << endl;
//         cin.ignore();
//         cout << "WARNING: This example will move the robot! "
//              << "Entering control loop" << endl;
//         //////////////////////////////////////////////////////////////////////////////////////////////
//         auto programStartTime = chrono::high_resolution_clock::now();
//         auto startlooptime = chrono::system_clock::now();
//         function<franka::JointVelocities(const franka::RobotState &, franka::Duration)> vel_callback =
//             [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
//             programStartTime = chrono::high_resolution_clock::now();
//             startlooptime = chrono::system_clock::now();
//             // Acquire the state
//             for (size_t i = 0; i < 7; i++) {
//                 qmsr[i] = robot_state.q[i];
//                 Dqmsr[i] = robot_state.dq[i];
//             }

//             //rcmFK(qmsr, XX, Je6);

//             // std::cout << "p_rcm" << p_rcm[0] << " " << p_rcm[1] <<" "<< p_rcm[2] << std::endl;
//             // std::cout << "error  " << (p_des - p).transpose() << std::endl;
            
//             loop_time += period.toSec();
//             Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
//             Eigen::Vector3d p(transform.translation());
//             Eigen::Quaterniond orientation_e(transform.rotation());
//            // orientation_e.normalize();
//             //
//             // for (int i = 0; i < 7; i++)
//             // {
//             //   q_com[i] = _q_com[i];
//             //   // q_null[i] = q_null_array[i];
//             // }
//             //
//         std::array<double, 42> jacobian_array =
//           model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
//             Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

//             v_tooltip_linear << *v_x, *v_y, *v_z;
//             v_tooltip_angular << *w_x, *w_z, *w_z;
//             p_tooltip_linear << p_tooltip_linear + period.toSec() * v_tooltip_linear;

//             double P_gainrcm = 5;
//             double P_gainendo = 5;

//             Eigen::Quaterniond q_dot;
//             // orientation_d = orientation_e;
//             *w_x = 0.0;
//             *w_y = 0.0;
//             *w_z = 0.0;
            
//             q_dot.w() = -0.5 * (orientation_d.x() * *w_x + orientation_d.y() * *w_y + orientation_d.z() * *w_z);

//             q_dot.x() = 0.5 * (orientation_d.w() * *w_x + orientation_d.y() * *w_z - orientation_d.z() * *w_y);
//             q_dot.y() = 0.5 * (orientation_d.w() * *w_y - orientation_d.z() * *w_x + orientation_d.x() * *w_z);
//             q_dot.z() = 0.5 * (orientation_d.w() * *w_z - orientation_d.x() * *w_y + orientation_d.y() * *w_x);
//             // cout<< "orientation_d.x()1 = "<<orientation_d.x()<<std::endl;
//             orientation_d.x() = orientation_d.x() + period.toSec() * q_dot.x();
//             orientation_d.y() = orientation_d.y() + period.toSec() * q_dot.y();
//             orientation_d.z() = orientation_d.z() + period.toSec() * q_dot.z();
//             orientation_d.w() = orientation_d.w() + period.toSec() * q_dot.w();
//            // orientation_d = q_dot*orientation_d;
//             orientation_d.normalize();

//             error_endo.head(3) << p_tooltip_linear - p;

//             v_tooltip << v_tooltip_linear, v_tooltip_angular;





//             //Option-1
//             // Eigen::Quaterniond quar_error( orientation_d*orientation_e.inverse());
//             // error_endo.tail(3) << quar_error.x(), quar_error.y(), quar_error.z();
//             // cout << "Error pos : " << error_endo.head(3) << "\n";
//             // cout << "Error ori : " << error_endo.tail(3) << "\n";


//             //Option-2

            
//             if (orientation_d.coeffs().dot(orientation_e.coeffs()) < 0.0) {
//                 orientation_e.coeffs() << -orientation_e.coeffs();
//             }
//             Eigen::Quaterniond quar_error( orientation_e.inverse()*orientation_d);
//             error_endo.tail(3) << -transform.rotation() * error_endo.tail(3);
//             cout << "Error ori : " << error_endo.tail(3) << "\n";
//             Je6_plus = jacobian.transpose() * (jacobian * jacobian.transpose()).inverse();

//             qd_com = Je6_plus * ((v_tooltip + P_gainendo * eye6 * (error_endo)));
//             // size=8 including eta

//             q_com = q_com + 1.0 * qd_com * period.toSec();
//             // jointsVelocityLimit(qd_com, q_com);
//             *startMove = true;
//             // cout << "*****************p\n" << p << endl;
//             // cout << "*****************p_des\n" << p_des << endl;
//             // cout << "*****************Jc_plus\n" << Jc_plus << endl;
//             // cout << "*****************qdi\n" << qdi << endl;
//             // cout << "*****************qd_com\n" << qd_com << endl;

//             // double cycle = std::floor(std::pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
//             // double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
//             // qd_com= 0.0 * qd_com;

//             qd_com[0] = std ::min(10.0 * 0.001 + qd_comold[0], std ::max(-10.0 * 0.001 + qd_comold[0], qd_com[0]));
//             qd_com[1] = std ::min(10.0 * 0.001 + qd_comold[1], std ::max(-10.0 * 0.001 + qd_comold[1], qd_com[1]));
//             qd_com[2] = std ::min(10.0 * 0.001 + qd_comold[2], std ::max(-10.0 * 0.001 + qd_comold[2], qd_com[2]));
//             qd_com[3] = std ::min(10.0 * 0.001 + qd_comold[3], std ::max(-10.0 * 0.001 + qd_comold[3], qd_com[3]));
//             qd_com[4] = std ::min(10.0 * 0.001 + qd_comold[4], std ::max(-10.0 * 0.001 + qd_comold[4], qd_com[4]));
//             qd_com[5] = std ::min(10.0 * 0.001 + qd_comold[5], std ::max(-10.0 * 0.001 + qd_comold[5], qd_com[5]));
//             qd_com[6] = std ::min(10.0 * 0.001 + qd_comold[6], std ::max(-10.0 * 0.001 + qd_comold[6], qd_com[6]));
//             //  qd_com[7] = std ::min(10.0 * 0.001 + qd_comold[7], std ::max(-10.0 * 0.001 + qd_comold[7], qd_com[7]));
//             qd_comold = qd_com;

//             franka::JointVelocities velocities = {qd_com[0], qd_com[1], qd_com[2], qd_com[3],
//                                                   qd_com[4], qd_com[5], qd_com[6]};
//             qd_com7[0] = qd_com[0];
//             qd_com7[1] = qd_com[1];
//             qd_com7[2] = qd_com[2];
//             qd_com7[3] = qd_com[3];
//             qd_com7[4] = qd_com[4];
//             qd_com7[5] = qd_com[5];
//             qd_com7[6] = qd_com[6];
//             // cout << "[Loop period = " << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000
//             //      << "ms]:\n";
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, w_x, 8);
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, w_y, 8);
//             v_leftEndEffector = *w_z + *v_gripper;
//             v_rightEndEffector = *w_z + *v_gripper;
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET, &v_leftEndEffector,
//                         8);
//             std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
//                         &v_rightEndEffector, 8);
//             dq_endoWrist << *w_x, *w_y, v_leftEndEffector + v_rightEndEffector;
//             double *motor_0_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_POSITION_OFFSET);
//             double *motor_1_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_POSITION_OFFSET);
//             double *motor_2_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_POSITION_OFFSET);
//             double *motor_3_actualPosition =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_POSITION_OFFSET);
//             double *motor_0_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_VELOCITY_OFFSET);
//             double *motor_1_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_VELOCITY_OFFSET);
//             double *motor_2_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_VELOCITY_OFFSET);
//             double *motor_3_actualVelocity =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_VELOCITY_OFFSET);
//             double *motor_0_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_ACTUAL_TORQUE_OFFSET);
//             double *motor_1_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_ACTUAL_TORQUE_OFFSET);
//             double *motor_2_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_ACTUAL_TORQUE_OFFSET);
//             double *motor_3_actualTorque =
//                 static_cast<double *>(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_ACTUAL_TORQUE_OFFSET);


//             if (loop_time < t_rec) {
//                 // check NODataRec to be consistent
//                 rec.addToRec(index);
//                 rec.addToRec(loop_time);
//                 rec.addToRec(qmsr);
//                 rec.addToRec(qd_com7);
//                 //       rec.addToRec(p_tooltip);
//                 // rec.addToRec(p_tooltip);
//                 // rec.addToRec(p);
//                 // rec.addToRec(eta);
//                 // rec.addToRec(dq_endoWrist);
//                 // rec.addToRec(*motor_0_actualPosition);
//                 // rec.addToRec(*motor_1_actualPosition);
//                 // rec.addToRec(*motor_2_actualPosition);
//                 // rec.addToRec(*motor_3_actualPosition);
//                 // rec.addToRec(*motor_0_actualVelocity);
//                 // rec.addToRec(*motor_1_actualVelocity);
//                 // rec.addToRec(*motor_2_actualVelocity);
//                 // rec.addToRec(*motor_3_actualVelocity);
//                 // rec.addToRec(*motor_0_actualTorque);
//                 // rec.addToRec(*motor_1_actualTorque);
//                 // rec.addToRec(*motor_2_actualTorque);
//                 // rec.addToRec(*motor_3_actualTorque);
//                 rec.next();
//                 index++;
//             }
//             return velocities;
//         };

//         *startFlag = true;
//         robot.control(vel_callback);

//         return SUCCESS;
//     } catch (const franka::Exception &ex) {
//         const double w_x_safety = 0.0;
//         const double w_y_safety = 0.0;
//         const double v_leftEndEffector_safety = 0.0;
//         const double v_rightEndEffector_safety = 0.0;
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_0_TARGET_VELOCITY_OFFSET, &w_x_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_1_TARGET_VELOCITY_OFFSET, &w_y_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_2_TARGET_VELOCITY_OFFSET,
//                     &v_leftEndEffector_safety, 8);
//         std::memcpy(pandaToEndoWristMappedRegion.get_address() + MOTOR_3_TARGET_VELOCITY_OFFSET,
//                     &v_rightEndEffector_safety, 8);
//         // print exception
//         cout << ex.what() << endl;
//         return ERR_EXIT;
//     }
// }

// End of Control functions

// Send UDP function
void endoWristSendUDP(bool *startFlag, bool *startMove, double *dq0, double *dq1, double *dq2, double *dq3) {
    UdpClientServer::UdpClient client("127.0.0.1", 8000);
    double endoWristMessage[6];
    cout << "Starting client " << client.get_addr() << ':' << client.get_port() << endl;
    while (!*startFlag)
        ;
    auto startlooptime = chrono::system_clock::now();
    while (*startFlag) {
        startlooptime = chrono::system_clock::now();
        if (*startMove) {
            endoWristMessage[0] = (*startFlag == true) ? 1.0 : 0.0;
            endoWristMessage[1] = (*startMove == true) ? 1.0 : 0.0;
            endoWristMessage[2] = *dq0;
            endoWristMessage[3] = *dq1;
            endoWristMessage[4] = *dq2;
            endoWristMessage[5] = *dq3;
            client.send(endoWristMessage, 6);
            // cout << "EndoWrist velocities: " << endoWristMessage[0] << ", " << endoWristMessage[1] << ", "
            //      << endoWristMessage[2] << ", " << endoWristMessage[3] << ", " << endoWristMessage[4] << ", "
            //      << endoWristMessage[5] << endl;
        }
        this_thread::sleep_until(startlooptime + chrono::microseconds(1000));
    }
}



// Load configuration function
map<uint, robot_t> loadConfiguration(void) {
    // Configuration file parsing
    auto programStartTime = chrono::high_resolution_clock::now(), configStartTime = programStartTime;
    map<uint, robot_t> robotMap = getRobotMap(CONFIG_PATH);
    cout << "[" << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000 << "ms]: "
         << "Configuration file loaded." << endl;
    if (robotMap[0].PlotFlag) printRobotMap("Configuration file contents: \n", robotMap);
    return robotMap;
}