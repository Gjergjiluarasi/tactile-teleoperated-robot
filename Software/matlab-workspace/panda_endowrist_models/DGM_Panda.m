% Computation of the DGM of the Panda 3 robot
% Written by Valentin Le Mesle
% 
% Input : q - vector of seven angles associated to the robot's configuration
%           (from base to effector)
%
% Functions dependencies: TransformationMatrix()
% 
% Notes: - Built using Modified Denavit Hartenberg (MDH) method
%
% Possible ameliorations: - Find better names for transformation matrices
%                         - Set hard coded values (offsets) as named parameters
%                         - Extract orientation of the frame associated to
%                         the effector

function [P,T_robot] = DGM_Panda(q)

% %% Example of input vector
% % Value of the 7 joints (in rad)
% q=[0,0,0,0,0,0,pi/6];

%% Computation of the transformation matrix of each joint
% Homogeneous transformation matrix related to every motor
% Note: T1 refers to the transformation from frame 0 to frame 1
% Note2: TransformationMatrix uses the MDH parameters rules
    T1 = TransformationMatrix(q(1)  ,0      ,0       ,0.333  );
    T2 = TransformationMatrix(q(2)  ,-pi/2  ,0       ,0      );
    T3 = TransformationMatrix(q(3)  ,pi/2   ,0       ,0.316  );
    T4 = TransformationMatrix(q(4)  ,pi/2   ,0.0825  ,0      );
    T5 = TransformationMatrix(q(5)  ,-pi/2  ,-0.0825 ,0.384  );
    T6 = TransformationMatrix(q(6)  ,pi/2  	,0       ,0      );
    T7 = TransformationMatrix(q(7)  ,pi/2   ,0.088   ,0.107  );

%% Computation of the DGM and extraction of the effector's position
% Computation of the transformation matrix of the whole robot
    T_robot=T1*T2*T3*T4*T5*T6*T7;

% Extraction of the position of the end effector
    Px= T_robot(1,4);
    Py= T_robot(2,4);
    Pz= T_robot(3,4);

% Extraction of the orientation of the effector (To do)


%% Formatting of the pose vector related to the end effector
P = [Px;Py;Pz];

end



%% Extra code used for testing
% Addition of the Endowrist effector
% qe=[0,pi/4,pi/8,pi/8]
% 
% 
% %% Model of the complete robot
% T_r1 = T_robot*T_EEF
% T_r2= T_robot*T_EEF2
% T_rtcp = T_robot*T_wrist
% %% Affichage Endowrist
% 
% T_ref = [1 0 0 0;...
%      0 1 0 0;...
%      0 0 1 0;...
%      0 0 0 1];
%  close all
% figure
% %axis equal
% hold on
% % axis([-1 0.1 -1 1 -1 1])
% Plot_frame(T_ref,[0,0,0]')
% %  Plot_frame(T_shaft,T_ref(1:3,4))
%   % Plot shaft
%   plot3([T_TCP(1,4) T_shaft(1,4)],[T_TCP(2,4) T_shaft(2,4)],[T_TCP(3,4) T_shaft(3,4)],'--k','linewidth',2)
%  Plot_frame(T_TCP,T_shaft(1:3,4))
%   %plot wrist
%   plot3([T_TCP(1,4) T_wrist(1,4)],[T_TCP(2,4) T_wrist(2,4)],[T_TCP(3,4) T_wrist(3,4)],'--r')
%  % Plot_frame(T_wrist,T_TCP(1:3,4))
%   Plot_frame(T_EEF,T_wrist(1:3,4))
%     plot3([T_EEF(1,4) T_wrist(1,4)],[T_EEF(2,4) T_wrist(2,4)],[T_EEF(3,4) T_wrist(3,4)],'--b','linewidth',2)
%     Plot_frame(T_EEF2,T_wrist(1:3,4))
%     plot3([T_EEF2(1,4) T_wrist(1,4)],[T_EEF2(2,4) T_wrist(2,4)],[T_EEF2(3,4) T_wrist(3,4)],'--m')