% Preamble
clear all;
close all;
clc;

% This script computes the state-transition, measurment function, 
% and the related Jacobian matrices for the Extended Kalman Filter
% used for the state estimation of the 3D pendulum system

%% Symbolic Derivation

% Parameters
syms dt 'real'                  % Time step
x = sym('x', [10,1], 'real');   % State vector
u = sym('u', [3,1], 'real');    % Input vector

% Constants
L = 2.0;    % Wire length [m]
L_t = 0;    % Wire length rate [m/s]
g = 9.81;   % Gravity [m/s^2]

% State vector
pt = x(1:3);
pt_t = x(4:6);
phi = x(7:8);
phi_t = x(9:10);

% Input vector
pt_tt = u(1:3);

% System ode
x_t = [pt_t;
       pt_tt;
       phi_t;
       pendulumDynamics(pt_tt, L, L_t, phi, phi_t)];
   
% State transition function 
% x_t = f(x,u)
f = x + x_t*dt;
f = simplify(f);

% State transition Jacobian
F = jacobian(f,x);
F = simplify(F);

% Measurement function
h = [pt;
     zeros(3,1);
     phi;
     zeros(2,1)];
h = simplify(h);

% Measurement function Jacobian
H = jacobian(h,u);
H = simplify(H);
   
%% Make functions
% State transition
% matlabFunction(f, 'File', 'f.m', 'Vars', {x, u, dt});
% matlabFunction(F, 'File', 'fJacobian.m', 'Vars', {x, u, dt});

% Measurement
matlabFunction(h, 'File', 'h.m', 'Vars', {x});
matlabFunction(H, 'File', 'hJacobian.m', 'Vars', {x});



