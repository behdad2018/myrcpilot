input = readtable('19.csv');



close all;

input.flight_mode

time=(input.last_step_ns-input.last_step_ns(1))*1e-9;
% 
% figure
% plot(input.T_ref);
% 
% figure;
% plot(input.theta_ref);
% 
% figure;
% plot(input.phi_ref);
% 
% figure;
% plot(input.uwind);
% 
% figure;
% plot(input.vwind);
% 
% figure;
% plot(input.wwind);
% 
% figure;
% plot(input.rho);

figure;
plot(time',input.rpm2);

% subplot(1,3,1);
% hold on;
% plot(input.xbee_x, input.xbee_y);
% plot([0 1 -1 0 0 0 0], [0 0 0 0 1 -1 0]);
% xlabel('X Position (m)')
% ylabel('Y Position (m)')
% ylim([-1.2 1.2])
% xlim([-1.2 1.2])
% 
% subplot(1,3,2);
% hold on;
% plot((0:numel(input.xbee_x)-1)*0.005, input.xbee_x);
% plot((0:numel(input.xbee_x)-1)*0.005, input.sp_X);
% xlabel('Time (s)')
% ylabel('X Position (m)')
% ylim([-1.2 1.2])
% 
% subplot(1,3,3);
% hold on;
% plot((0:numel(input.xbee_y)-1)*0.005, input.xbee_y);
% plot((0:numel(input.xbee_y)-1)*0.005, input.sp_Y);
% xlabel('Time (s)')
% ylabel('Y Position (m)')
% ylim([-1.2 1.2])


% #define CROSS_T0 (3)
% #define CROSS_T1 (CROSS_T0 + 14) 17
% #define CROSS_T2 (CROSS_T1 + 8)  25
% #define CROSS_T3 (CROSS_T2 + 12) 37
% #define CROSS_T4 (CROSS_T3 + 8)  45
% #define CROSS_T5 (CROSS_T4 + 8)  53
% #define CROSS_T6 (CROSS_T5 + 12) 65
% #define CROSS_T7 (CROSS_T6 + 8)  
% #define CROSS_T8 (CROSS_T7 + 1)
% 
