close all
clear all
clc;

%% Simulation Setup
% Declaring system constants
dt = 0.01; %1e-3;  % Sample time [s]
g = 9.81;   % Gravity [m/s^2]
L = 2.0;    % Wire length [m]
d = 5;      % Virtual damping coefficient

% Initial values
theta_init = 15 *pi/180;    % Initial angular position of pendulum
theta_t_init = 0;           % Initial angular velocity of pendulum
theta_tt_init = 0;          % Initial angular acceleration of pendulum
x_init = 0;                 % Initial position of tool-point
x_t_init = 0;               % Initial velocity of tool-point