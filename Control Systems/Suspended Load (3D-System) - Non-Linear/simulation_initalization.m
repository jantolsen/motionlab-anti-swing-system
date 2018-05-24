% Preamble
% clear all;
close all;
clc;

%% System Setup
%% Non-Linear Control for the 3D Pendulum System

% Preamble
clear all;
close all;
clc;

% Constants
dt = 1e-3;  % Time step
L = 2.0;    % Wire length
g = 9.81;   % Gravity [m/s^2]
d_y = 4;    % Virtual damping coefficient
d_x = 7;

% Initial values
% Position
x_init   = 2.6192;
y_init   = 0;
z_init   = 2.24;

phix_init = 15*pi/180*1;
phiy_init = 20*pi/180*1;

pt_init = [x_init, y_init, z_init];
phi_init = [phix_init, phiy_init];

% Velocity
x_t_init = 0;
y_t_init = 0;
z_t_init = 0;

phix_t_init = 0;
phiy_t_init = 0;

pt_t_init = [x_t_init, y_t_init, z_t_init];
phi_t_init = [phix_t_init, phiy_t_init];

init = [pt_init, pt_t_init, phi_init, phi_t_init];

%% Extended Kalman Filter

% Initial state covariance
P = 1e-3;                       
% Process noise covariance
Q = diag([0.001^2, 0.01^2, 0.1^2, ...
          0.001^2, 0.01^2, 0.1^2]);
% Measurement noise covariance
R = diag([0.001^2, 0.01^2, ...
          0.001^2, 0.001^2]);
      
%% Cascade Control Parameters

Kpx_t = 4.786;
Kdx_t = 3.324;

Kpy_t = 1.002;
Kdy_t = 0.4079;

Kpx = 0.577;
Kix = 0.103;

Kpy = 0.7879;
Kiy = 0.07209;