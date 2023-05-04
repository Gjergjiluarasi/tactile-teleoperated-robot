% Example of code to display a Panda 3 robot in a desired configuration

%% Definition of the joints positions
% Joint configuration for the Panda robot
    q = [0,0,0,0,0,0,0];
% Joint configuration of the Endowrist
    qe = [0,0,0,0];

%% Calculation of the robot and the tool models
% Computation of the pose of the robot's effector
    [P_r,T_r] = DGM_Panda(q);
% Computation of the pose of the Endowrist
    [T_Ew1, T_Ew2]=DGM_Endo(qe);

% Position of the grippers with respect to the robot's base
    T_tool1 = T_r*T_Ew1; % Position of the reference point of the first jaw
    T_tool2 = T_r*T_Ew2; % Position of the reference point of the second jaw