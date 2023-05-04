% Computation of the DGM of the Endowrist
% Written by Valentin Le Mesle
% 
% Input : vector of four angles associated to the endowrist's configuration
%           1- Rotation of the shaft
%           2- Rotation of the wrist
%           3- Actuation of the first jaw
%           4- Actuation of the second jaw
%
% Functions dependencies: TransformationMatrix()
% 
% Notes: - Built using Modified Denavit Hartenberg (MDH) method
%        - Angle of the second jaw has to be expressed with respect to the
%        shaft (independant positions). Use q3=q4 for closed gripper
%
% Possible ameliorations: - Find better names for transformation matrices
%                         - Set hard coded values (offsets) as named parameters

function [T_EEF, T_EEF2]=DGM_Endo(qe)

%% Definition of the transformation matrices of the kinematic chain
    % From base to tip (shaft)
    T_Ew_int = TransformationMatrix(0,0,7.65E-02,3.815E-02);
    T_Ew_int2= TransformationMatrix(pi/2,pi/2,0,2.314E-02);
    T_Ew_shaft= TransformationMatrix(qe(1),-pi/2,0,0+0.545);
    
    %Include mobility of the wrist
    T_Ew_Wrist=TransformationMatrix(qe(2)+pi/2,pi/2,0,0);
    
    % TCP (Tool Center Point) for both parts of the gripper
    T_Ew_EEF = TransformationMatrix(qe(3),-pi/2,1.05E-2,0);
    T_Ew_EEF2 = TransformationMatrix(qe(3)+qe(4),pi,0,0);
    
    % Grasping distance
    T_EW_grasp = TransformationMatrix(0,0,0.027,0);

%% Compute the pose of the effectors
    % Previous testing transformations
    % T_shaft = T_Ew_int*T_Ew_int2;
    % T_TCP=T_Ew_int*T_Ew_int2*T_Ew_shaft;
    % % end of tool and 
    % T_wrist=T_TCP*T_Ew_Wrist*T_Ew_EEF;
    % T_wrist2=T_wrist*T_Ew_EEF2;
    % % Tip of tool
    % T_EEF=T_wrist*T_EW_grasp;
    % T_EEF2=T_wrist2*T_EW_grasp;

    %Common part
    T_com = T_Ew_int*T_Ew_int2*T_Ew_shaft*T_Ew_Wrist*T_Ew_EEF;
    %Independent jaws
    T_EEF = T_com*T_EW_grasp;
    T_EEF2 = T_com*T_Ew_EEF2*T_EW_grasp;

end