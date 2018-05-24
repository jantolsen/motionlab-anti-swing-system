% Preamble
clear all;
close all;
clc;

% Constant
g = 9.81;   % Gravity [m/s^2]
dt = 1e-3;  % Step time
L = 2.0;    % Wire length [m]
L_t = 0;    % Wire velocity [m/s]

%% Initial Values
% Robot joint angles
q1_init = 0;
q2_init = 0;
q3_init = -pi/2;

q_init = [q1_init, q2_init, q3_init];

% Stewart Platform Orientation
eta1_init = 0;
eta2_init = 0;
eta3_init = 0;
eta4_init = 0;
eta5_init = 0;
eta6_init = 0;

eta_init = [eta1_init, eta2_init, eta3_init, ...
            eta4_init, eta5_init, eta6_init];
        
% Pendulum
phix_init = 2*pi/180;
phiy_init = 4*pi/180;

phi_init = [phix_init, phiy_init];

% Tool-Point
Pt_init = [-2.395, 3.811, -3.255];

%% Virtual Damping

% Damping Coefficients
c_x = 10;
c_y = 10;

%% Cascade Controller

Kp_xt = 5;
Kd_xt = 8.7;

% Kp_xt = 0.7328;
% Ki_xt = 0.003054;

Kp_yt = 0.7294;
Ki_yt = 0.09731;

Kp_zt = -1;
Ki_zt = -0.1391;

