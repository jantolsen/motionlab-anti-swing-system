%% Linear Control for the 3D Pendulum System
% This scripts contains the derivation of the linarized model
% Selection of the Extended Kalman Filter estimator paramters
% aswell as different controller schemes

% Preamble
% clear all;
close all;
clc;

%% Initial values

% Constants
dt = 1e-3;  % Time step
L = 2.0;    % Wire length
g = 9.81;   % Gravity [m/s^2]

% Initial values
% Position
x_init   = 2.619;
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

%% Symbolic Linearization
% Parameters
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

% Non-linear function of system ODE
% x_t = f(x,u)
f = [pt_t;
     pt_tt;
     phi_t;
     pendulumDynamics(pt_tt, L, L_t, phi, phi_t)];
 
% Non-linear function
% y = h(x,u)
h = [pt; phi];

% State-space
A_sym = jacobian(f, x);
B_sym = jacobian(f, u);
C_sym = jacobian(h, x);
D_sym = jacobian(h, u);

% Equilibrium states
u0 = zeros(3,1);                                   % Input vector
x0 = [pt; zeros(3,1); zeros(2,1); zeros(2,1)];    % State vector

% Update the Matrices 
% with the linearization around at equilibrium states
A_sym = subs(A_sym, [x,u],[x0,u0]);
B_sym = subs(B_sym, [x,u],[x0,u0]);
C_sym = subs(C_sym, [x,u],[x0,u0]);
D_sym = subs(D_sym, [x,u],[x0,u0]);

%% Simulink State-space

% Update the symoblic Matrices with constant values 
% and create numeric Matrices
A = double(subs(A_sym));
B = double(subs(B_sym));
C = double(subs(C_sym));
D = double(subs(D_sym));

%% Extended Kalman Filter 

Nx = length(x); % Number of states

% Initial state covariance
P_EKF = 1e-3;

% Process noise covariance
Q_EKF = eye(Nx)*0.001^2;
Q_EKF(1:3,1:3) = eye(3)*0.00001^2;  % Pt
Q_EKF(4:6,4:6) = eye(3)*0.001^2;    % Pt_t
Q_EKF(7:8,7:8) = eye(2)*0.001^2;    % phi
Q_EKF(9:10,9:10) = eye(2)*0.01^2;   % phi_t

% Process noise covariance
R_EKF = eye(Nx)*1^2;
R_EKF(1:3,1:3) = eye(3)*0.001^2;    % Pt
R_EKF(4:5,4:5) = eye(2)*0.01^2;     % phi

%% State-Feedback
% Weight
w = 10;

% Weight matrix for the states
Q_lqr = diag([ 100*w,	100*w,	100*w, ...  % Pt
                10*w,	10*w,	10*w, ...  	% Pt_t
              1000*w, 1000*w, ...           % Phi
               100*w,  100*w]);             % Phi

% Weight matrix for the control
R_lqr = diag([1, 1, 1]);    % Size = columns of B

K = lqr(A,B,Q_lqr, R_lqr);

% New State-space model
Ar = A - B*K;
F = ss(Ar, B, C, D);

% Show step response
% figure(1)
% step(F)

%% Pre-Filter
% C_xyz = C(1:3,:);
C_xyz = C(1:3,:);               % Only direct output from input states
N = inv(C_xyz*inv(B*K - A)*B);  % Pre-filter constant

%% Integral Control
Ai = [zeros(size(C_xyz,1)), C_xyz;
      zeros(size(A,1), size(C_xyz,1)), A];
Bi = [zeros(size(C_xyz,1), size(B,2)); B];
Ci = [zeros(size(C_xyz,1)), C_xyz];
Di = zeros(size(C_xyz,1),size(Bi,2));

% Weight matrix for the states
Q_lqr_IC = diag([ 100*w,	100*w,	100*w, ...  % z (error states)
                  100*w,	100*w,	100*w, ...  % Pt
                   10*w,	 10*w,	 10*w, ...	% Pt_t
                 1000*w,   1000*w, ...          % Phi
                  100*w,    100*w]);            % Phi
                        
% Weight matrix for the control
R_lqr_IC = diag([1, 1, 1]);    % Size = columns of B

KI = lqr(Ai, Bi, Q_lqr_IC, R_lqr_IC);
Ke = KI(:,1:3);
Ko = KI(:,4:end);

% New State-space model
Ar = Ai - Bi*KI;
Hi = ss(Ar, Bi, Ci, Di);

% Show step response
% figure(2)
% step(Hi)