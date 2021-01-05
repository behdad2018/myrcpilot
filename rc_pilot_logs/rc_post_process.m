input = readtable('15.csv');



close all;

input.flight_mode(1)

time=(input.last_step_ns-input.last_step_ns(1))*1e-9;
% 
figure
plot(time',input.T_ref);
xlabel('Time [s]')
title('Tref'); 
% 
figure;
plot(time',input.theta_ref);
xlabel('Time [s]')
title('theta_ref'); 
% 
figure;
plot(time',figure);
xlabel('Time [s]')
title('phi_ref'); 
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
figure;
plot(time',input.rho);
xlabel('Time [s]')
title('rho'); 

figure(1);
plot(time',input.u_Z);
xlabel('Time [s]')
title('Uz'); 

figure;
plot(time',input.thrust_mot1);
hold on;
plot(time',input.thrust_mot2);
plot(time',input.thrust_mot3);
plot(time',input.thrust_mot4);
title('mot thrust')
xlabel('Time [s]')
legend('thrust1','thrust2','thrust3','thrust4');

figure;
plot(time',input.mot_1);
hold on;
plot(time',input.mot_2);
plot(time',input.mot_3);
plot(time',input.mot_4);
title('mot throttle')
xlabel('Time [s]')
legend('throttle1','throttle2','throttle3','throttle4');

figure;
plot(time',input.rpm1);
hold on;
plot(time',input.rpm2);
plot(time',input.rpm3);
plot(time',input.rpm4);
title('mot RPM')
xlabel('Time [s]')
legend('RPM1','RPM2','RPM3','RPM4');

figure;
plot(time',input.T_ref);
xlabel('Time [s]');
title('Ref Net Thrust');

figure;
plot(time',input.uwind);
hold on;
plot(time',input.vwind);
plot(time',input.wwind);
xlabel('Time [s]');
title('Relative velocities');
legend('U','V','W');

figure;
plot(time',input.sp_roll);
hold on;
plot(time',input.sp_pitch);
plot(time',input.sp_yaw);
xlabel('Time [s]');
title('setpoint Euler');
legend('sp_roll','sp_pitch','sp_yaw');

figure;
plot(time',input.roll);
hold on;
plot(time',input.pitch);
plot(time',input.yaw);
xlabel('Time [s]');
title('Euler');
legend('roll','pitch','yaw');

%rpmtothrottle(8505.2754,[0.071,-0.049,-0.039])

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
