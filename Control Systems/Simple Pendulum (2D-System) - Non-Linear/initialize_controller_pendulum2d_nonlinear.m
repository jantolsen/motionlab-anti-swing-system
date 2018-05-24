%% Linear Control for the 2D Pendulum System
% This script contain the derivation of the linearized model,
% design of the Kalman Filter Estimator and the Extended Kalman Filter
% aswell as the controller schemes for the state-feedback pre-filter
% control, and the state-feedback integral control

clear all;
close all;
clc;

%% Initial Values

% Constants
dt = 1e-3;  % Time step
L = 2.0;    % Wire length
g = 9.81;   % Gravity [m/s^2]
d = 5;      % Virtual damping coefficient

% Initial values
x_init = 0;
x_t_init = 0;
theta_init = 00*pi/180;
theta_t_init = 0;

%% Estimator Design

% Equilibrium point
u0 = 0;
x0 = [0; 0; 0; 0];

% Covariance matrices
P = 1e-3;                               % Initial state covariance
Q = diag([0, 0.001^2, 0.01^2, 0.01^2]); % Process noise covariance
R = diag([0.001^2, 0.01^2]);            % Measurement noise covariance

%% Transfer function finder and PD Controller

s = tf('s');

% 3 zeros - 5 poles
G = (0.9977*s^3 + 3.789*s^2 + 4.626*s + 19.53)/...
    (s^5 + 4.819*s^4 + 38.53*s^3 + 19.13*s^2 + 0.01153*s);
% G = (-0.2361*s + 0.9867) / (s^2)

% pzmap(G)
% zpk(G)
% step(G)

Kp = 0.1;
Kd = 0.85;
PD = Kp + Kd*s;

% H = (PD*G)/(1 + PD*G);
% step(H)

