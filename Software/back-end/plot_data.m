close all;
clear all;
run('./DATA.m')
DATAm = DATAm(~all(DATAm == 0, 2),:);
index = DATAm(:,1);
looptime = DATAm(:,2);
qmsr = DATAm(:,3:9);
Dqmsr = DATAm(:,10:16);
p_rcm = DATAm(:,17:19);
p_des = DATAm(:,20:22);
p = DATAm(:,23:25);
eta = DATAm(:,26);
F_endo = DATAm(:,27:29);
tau_joints = DATAm(:,30:36);
F_diff = DATAm(:,37:39);
%   
% eta = DATAm(:,26);
% dqendo_ref = DATAm(:,27:29);
% qendo = DATAm(:,30:33);
% dqendo = DATAm(:,34:37);
% Torqueendo = DATAm(:,38:41);
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, dqendo_ref);
% title('dq endo ref')
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, qendo);
% title('q endo')
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, dqendo);
% title('dq endo')
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, qmsr);
% title('qmsr')
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, Dqmsr);
% title('dqmsr')

figure();
hold on;
grid minor;
plot(looptime, p_des-p);
title('error-tracking')

figure();
hold on;
grid minor;
plot(looptime, p_rcm-p_rcm(1,:));
title('error-rcm')
legend();%, 'tauJ d');

figure();
hold on;
grid minor;
plot(looptime, F_endo);
title('F endo')
legend();%, 'tauJ d');

figure();
hold on;
grid minor;
plot(looptime, F_endo2);
title('F endo2')
legend();%, 'tauJ d');

figure();
hold on;
grid minor;
plot(looptime, F_diff);
title('F diff')
legend();%, 'tauJ d');

figure();
hold on;
grid minor;
plot(looptime, tau_joints);
title('taujoint')
legend();%, 'tauJ d');
% 
% figure()
% subplot(3,1,1)
% plot(looptime, eta);
% %plot(pandatime, tauJ_d(:,2));
% title('eta')
% subplot(3,1,2)
% plot(looptime, man);
% %plot(pandatime, tauJ_d(:,2));
% title('man')
% subplot(3,1,3)
% plot(looptime, manJc);
% %plot(pandatime, tauJ_d(:,2));
% title('manJc')

% 
% figure()
% plot3(p(1:15000,1),p(1:15000,2),p(1:15000,3))
% hold on
% plot3(p_des(1:15000,1),p_des(1:15000,2),p_des(1:15000,3),"O")
% hold off
% 
% figure();
% hold on;
% grid minor;
% plot(looptime, tau_cal);
% title('tau cal')
% legend()