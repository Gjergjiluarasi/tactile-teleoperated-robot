#include "app.hh"

// Default App constructor
App::App::App()
{
    name = DEFAULT_APP_NAME;
    description = DEFAULT_APP_DESCRIPTION;
    state = INIT;
}

// Custom App constructor
App::App::App(string a_name, string a_description)
{
    name = a_name;
    description = a_description;
    state = INIT;
}

// Member functions
AppResultState App::App::RCM(void)
{
    try
    {
        // Configuration file parsing
        // this_thread::sleep_for(500ms);
        auto programStartTime = chrono::high_resolution_clock::now(), configStartTime = programStartTime;
        chrono::duration<float> configDurationTime;

        // configStartTime = chrono::high_resolution_clock::now();
        map<uint, robot_t> robotMap = getRobotMap(CONFIG_PATH); // Fetch configuration data
        // configDurationTime = chrono::high_resolution_clock::now() - configStartTime;
        cout << "[" << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000 << "ms]: "
             << "Configuration file loaded." << endl;
        if (robotMap[0].PlotFlag)
            printRobotMap("Configuration file contents: \n", robotMap);

        // FrankaEmika object instantiation
        FrankaEmika leftFR3(robotMap[0].ID, robotMap[0].Name, // Initialize Franka Emika robot object
                            robotMap[0].Description, robotMap[0].FrankaIP,
                            "127.0.0.1" /*robotMap[0].PcUdpIP*/, robotMap[0].PcUdpPort, // PC UDP IP and port should be used for controlling the EndoWrist from the haptic console
                            robotMap[0].PlotFlag);
        cout << "[" << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000 << "ms]: "
             << "FrankaEmika object loaded." << endl;

        // EndoWrist object instantiation
        MotorControllerParameters leftEndoWristYawMotorParams = {
            .position = 0,
            .velocity = 0,
            .current = 0};
        MotorControllerParameters leftEndoWristWristMotorParams = {
            .position = 0,
            .velocity = 0,
            .current = 0};
        MotorControllerParameters leftEndoWristLeftGripperMotorParams = {
            .position = 0,
            .velocity = 0,
            .current = 0};
        MotorControllerParameters leftEndoWristRightGripperMotorParams = {
            .position = 0,
            .velocity = 0,
            .current = 0};
        EndoWrist leftEndoWrist(robotMap[0].Tool.ID, robotMap[0].Tool.Name, robotMap[0].Tool.Description,
                                leftEndoWristYawMotorParams, leftEndoWristWristMotorParams,
                                leftEndoWristLeftGripperMotorParams, leftEndoWristRightGripperMotorParams);
        cout << "[" << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000 << "ms]: "
             << "EndoWrist object loaded." << endl;

        if (leftFR3.plotFlag)
        {
            cout << "leftFR3.name: " << leftFR3.name << endl
                 << "leftFR3.description: " << leftFR3.description << endl
                 << "leftFR3._ID: " << leftFR3.getID() << endl
                 << "leftFR3._frankatIP: " << leftFR3.getFrankaIP() << endl
                 << "leftFR3._pcUdpIP: " << leftFR3.getPcUdpIP() << endl
                 << "leftFR3._pcUdpPort: " << leftFR3.getPcUdpPort() << endl
                 << "leftFR3.plotFlag: " << leftFR3.plotFlag << endl
                 << "leftEndoWrist.name: " << leftEndoWrist.name << endl
                 << "leftEndoWrist.description: " << leftEndoWrist.description << endl
                 << "leftEndoWrist._ID: " << leftEndoWrist.getID() << endl;
        }


        if (leftFR3.plotFlag)
        {
            int n = 5000; // 5000 data points
            vector<double> x(n), y(n);
            for (int i = 0; i < n; ++i)
            {
                x.at(i) = i * i;
                y.at(i) = sin(2 * M_PI * 2 * i / 360.0);
            }
            plt::figure();                                         // declare a new figure (optional if only one is used)
            plt::plot(x, y, "b-", {{"label", "sin(2*pi*i/360)"}}); // red dashed line // automatic coloring: tab:blue
            plt::show(false);
            plt::xlim(0, 1000 * 1000);    // x-axis interval: [0, 1e6]
            plt::title("Standard usage"); // set a title
            plt::legend();                // enable the legend
            plt::savefig("log/plot.jpg"); // save the figure
            plt::show();
        }

        runProgramResult runProgramRes = initProgram();
        cout << "[" << (chrono::high_resolution_clock::now() - programStartTime).count() / 1000 << "ms]: "
             << "EndoWrist system initialization finished with result: "
             << ((runProgramRes == runProgramResult::RUNNING) ? "RUNNING" : (runProgramRes == runProgramResult::ERROR_INIT_MOTORS)      ? "ERROR_INIT_MOTORS"
                                                                        : (runProgramRes == runProgramResult::ERROR_SCHED_SETSCHEDULER) ? "ERROR_SCHED_SETSCHEDULER"
                                                                        : (runProgramRes == runProgramResult::ERROR_MEMORY_LOCK)        ? "ERROR_MEMORY_LOCK"
                                                                                                                                        : "ERROR_SET_COMMAND")
             << endl;
        // runProgramRes = runProgram();
        // cout << "Program result after run: " << runProgramRes << endl;

        // Haptic console to robot arm communication variables
        double hapticConsoleMessage[13];

        // Compliance parameters
        const double translational_stiffness{150.0};
        const double rotational_stiffness{10.0};
        Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
        stiffness.setZero();
        stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
        damping.setZero();
        damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) * Eigen::MatrixXd::Identity(3, 3);
        damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) * Eigen::MatrixXd::Identity(3, 3);

        // connect to robot
        franka::Robot robot(leftFR3.getFrankaIP());
        setDefaultBehavior(robot);

        // Drive into home position
        array<double, 7> q_goal = {{0, 1 * M_PI / 6, 0, -1 * M_PI / 2.5, -M_PI_2, M_PI_2, -M_PI / 1.5}};
        MotionGenerator motion_generator(0.5, q_goal);
        cout << "WARNING: This example will move the robot! "
             << "Please make sure to have the user stop button at hand !" << endl
             << "Press Enter to continue..." << endl;
        cin.ignore();
        robot.control(motion_generator);

        double eta = 0.45;
        Vector8d qd_cmd, q_cmd;
        qd_cmd.setZero();

        array<double, 7> q_measured = {0, 0, 0, 0, 0, 0, 0};
        array<double, 7> dq_measured = {0, 0, 0, 0, 0, 0, 0};

        Vector3d p_rcm, p_rcm0;

        Vector3d p_tooltip;
        Vector3d v_tooltip;
        p_tooltip.setZero();
        v_tooltip.setZero();

        double man, manJc;
        Eigen::Matrix4d XX, XXp;
        Matrix<double, 6, 7> J;
        Matrix<double, 3, 7> J3;
        Matrix<double, 3, 5> Jc;
        Matrix<double, 8, 5> B;
        Matrix<double, 3, 8> Je3;
        Matrix<double, 5, 3> Jc_plus;
        Vector5d qdi;
        Vector3d p, p_des, pd_des, p0;
        Matrix3d eye3;
        Matrix7d eye7;

        p_rcm.setZero();
        p_rcm0.setZero();

        // p_des.setZero();
        // pd_des.setZero();
        eye7.setIdentity();
        Jc_plus.setZero();

        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();
        franka::RobotState initial_state = robot.readOnce();

        // Initial readout of RCM forward kinematics
        // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q0(initial_state.q.data());
        rcmFK(initial_state.q, eta, XX, XXp, Jc, B, Je3, man, manJc);

        // equilibrium point is the initial position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.rotation());

        // set collision behavior
        robot.setCollisionBehavior({{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                                   {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                                   {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}},
                                   {{200.0, 200.0, 200.0, 200.0, 200.0, 200.0}});

        // start real-time control loop
        cout << "WARNING: Collision thresholds are set to high values. "
             << "Make sure you have the user stop at hand!" << endl
             << "After starting try to push the robot and see how it reacts." << endl
             << "Press Enter to continue..." << endl;
        cin.ignore();

        array<double, 16> initial_pose;
        double time = 0.0;

        double v_x;
        double v_y;
        double v_z;

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

        function<franka::JointVelocities(const franka::RobotState &, franka::Duration)>
            vel_callback = [&](const franka::RobotState &state, franka::Duration period) -> franka::JointVelocities
        {
            ////////////////////////////////////////////////////////////////////////////////////////////////
            // EndoWrist part
            // ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, NULL);
            // if (ret)
            // {
            // fprintf(stderr, "clock_nanosleep(): %s\n", strerror(ret));
            // // break;
            // }

            setCommandRes = setProfilePosition(&wakeup_time);

            wakeup_time.tv_nsec += PERIOD_NS;
            while (wakeup_time.tv_nsec >= NSEC_PER_SEC)
            {
                wakeup_time.tv_nsec -= NSEC_PER_SEC;
                wakeup_time.tv_sec++;
            }
            //////////////////////////////////////////////////////////////////////////////////////////////

            // Acquire the state
            for (size_t i = 0; i < 7; i++)
            {
                q_measured[i] = state.q[i];
                dq_measured[i] = state.dq[i];
            }

            rcmFK(q_measured, eta, XX, XXp, Jc, B, Je3, man, manJc);
            p = XX.block<3, 1>(0, 3);
            p_tooltip = XX.block<3, 1>(0, 3);

            Jc_plus = Jc.transpose() * (Jc * Jc.transpose()).inverse();

            if (leftFR3.recv(hapticConsoleMessage, MAX_SIZE))
            {
                v_x = -min(2.0, max(-2.0, hapticConsoleMessage[4])) / ROBOT_TO_CONSOLE_RATIO;
                v_y = -min(2.0, max(-2.0, hapticConsoleMessage[5])) / ROBOT_TO_CONSOLE_RATIO;
                v_z = min(2.0, max(-2.0, hapticConsoleMessage[6])) / ROBOT_TO_CONSOLE_RATIO;
                v_tooltip << v_x, v_y, v_z;
                p_tooltip << p_tooltip + period.toSec() * v_tooltip;
            }
            else
            {
                cout << "Not receiving" << endl;
                v_tooltip << v_x, v_y, v_z;
                p_tooltip << p_tooltip + period.toSec() * v_tooltip;
            }

            qdi = Jc_plus * (v_tooltip + 5 * eye3 * (p_tooltip - p)); //+1.0*(eye7 - Jc_plus*Jc)*(q_null - q);
            qd_cmd = B * qdi;                                         // size=8 including eta
            q_cmd = q_cmd + 1.0 * qd_cmd * period.toSec();
            if (((int)hapticConsoleMessage[0] * 1000) % 1000 == 0)
            {
                cout << ((int)hapticConsoleMessage[0] * 1000) % 1000 << endl;
                cout << "timestamp:" << hapticConsoleMessage[0] << endl
                     << "x:" << hapticConsoleMessage[1] << endl
                     << "y:" << hapticConsoleMessage[2] << endl
                     << "z:" << hapticConsoleMessage[3] << endl
                     << "vx:" << hapticConsoleMessage[4] << endl
                     << "vy:" << hapticConsoleMessage[5] << endl
                     << "vz:" << hapticConsoleMessage[6] << endl
                     << "raw joint velocities:" << endl
                     << qd_cmd(0) << "," << qd_cmd(1) << "," << qd_cmd(2) << "," << qd_cmd(3) << "," << qd_cmd(4) << "," << qd_cmd(5) << "," << qd_cmd(6) << "," << qd_cmd(7) << endl;
            }
            jointsVelocityLimit(qd_cmd, q_cmd);

            if (((int)hapticConsoleMessage[0] * 1000) % 1000 == 0)
            {
                cout << "limited joint velocities:" << endl
                     << qd_cmd(0) << "," << qd_cmd(1) << "," << qd_cmd(2) << "," << qd_cmd(3) << "," << qd_cmd(4) << "," << qd_cmd(5) << "," << qd_cmd(6) << "," << qd_cmd(7) << endl;
            }

            eta = q_cmd[7];

            franka::JointVelocities velocities = {qd_cmd[0], qd_cmd[1], qd_cmd[2], qd_cmd[3], qd_cmd[4], qd_cmd[5], qd_cmd[6]};
            return velocities;
        };

        robot.control(vel_callback);

        return SUCCESS;
    }
    catch (const franka::Exception &ex)
    {
        // print exception
        cout << ex.what() << endl;
        return ERR_EXIT;
    }
}