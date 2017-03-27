%% Steering wheel control
clc
clear all
close all
% Real system parameters
tau_v=0.05;                 % speed time constant
% Position reference. To change the position profile, add blocks in steering_wheel_system.mdl
t=20;                       % Simulation stop time
ts=2;                       % First step time
A=30;                       % Final value (First step)
% Temporals specifications
tau_bc1=0.9*tau_v;          % loop closed time constant (speed)
% Controllers parameters
% Speed loop [First order response. Controller PI]
Ti2=tau_v;                  % Integral
K2=Ti2/tau_bc1;             % Proportional
% Position loop [Second order response. Controller P]
dseta=1.5;                  % System type:  Over-damped dseta>1;
                            %               Critical-damped dseta=1;
                            %               Under-damped dseta<1;
                            %               Undamped dseta=0
wn=1;                       % Natural frecuency [rad/s]
K1=wn/2*dseta;              % Proportional

%% Simulation & graphs
sim('steering_wheel_system');
figure(1)
subplot(2,1,1)
plot(time,speed_ref,time,speed_steering_wheel, 'LineWidth',1)
title('Speed')
xlabel('Time [s]')
ylabel('[]')                % Add units
legend('Reference','Siganl')
grid
subplot(2,1,2)
plot(time,pos_ref,time,pos_steering_wheel, 'LineWidth',1)
title('Position')
xlabel('Time [s]')
ylabel('[]')                % Add units
legend('Reference','Signal')
=======
%% Steering wheel control
clc
clear all
close all
% Real system parameters
tau_v=0.05;                 % speed time constant
% Position reference. To change the position profile, add blocks in steering_wheel_system.mdl
t=20;                       % Simulation stop time
ts=2;                       % First step time
A=30;                       % Final value (First step)
% Temporals specifications
tau_bc1=0.9*tau_v;          % loop closed time constant (speed)
% Controllers parameters
% Speed loop [First order response. Controller PI]
Ti2=tau_v;                  % Integral
K2=Ti2/tau_bc1;             % Proportional
% Position loop [Second order response. Controller P]
dseta=1.5;                  % System type:  Over-damped dseta>1;
                            %               Critical-damped dseta=1;
                            %               Under-damped dseta<1;
                            %               Undamped dseta=0
wn=1;                       % Natural frecuency [rad/s]
K1=wn/2*dseta;              % Proportional

%% Simulation & graphs
sim('steering_wheel_system');
figure(1)
subplot(2,1,1)
plot(time,speed_ref,time,speed_steering_wheel, 'LineWidth',1)
title('Speed')
xlabel('Time [s]')
ylabel('[]')                % Add units
legend('Reference','Siganl')
grid
subplot(2,1,2)
plot(time,pos_ref,time,pos_steering_wheel, 'LineWidth',1)
title('Position')
xlabel('Time [s]')
ylabel('[]')                % Add units
legend('Reference','Signal')
grid