clear all;
close all;
clc;

%% Symbolic
x = sym('x', [4,1], 'real');

syms L g U 'real'

f = [x(2);
     U;
     x(4);
     (-g*sin(x(3)) - cos(x(3))*U)/L];
h = [x(1), 0, x(3), 0];
    
A_sym = jacobian(f, x);
B_sym = jacobian(f, U);
C_sym = jacobian(h, x);
D_sym = jacobian(h, U);

% Equilibrium point
U = 0;
x3 = 0;
subs(A_sym)
subs(B_sym)

%% Simulink State-space

% Clear symbolic
clear L g

dt = 1e-3;
L = 2.0;
g = 9.81;

% Initial values
x_init = 0;
x_t_init = 0;
theta_init = 10*pi/180;
theta_t_init = 0;

%% State-space model
A = [0, 1, 0, 0;
     0, 0, 0, 0;
     0, 0, 0, 1;
     0, 0, -g/L, 0];
B = [0, 1, 0, -1/L]';
C = [1, 0, 0, 0;
     0, 0, 1, 0];
D = [0,0]';

G = ss(A,B,C,D);

% Covariance matrices
P = 1e-3;                               % Initial state covariance
Q = diag([0, 0.001^2, 0.01^2, 0.01^2]); % Process noise covariance
R = diag([0.001^2, 0.01^2]);            % Measurement noise covariance

%% Feedback gain
w = 10;
Q_lqr = diag([10*w, 10*w, 1000*w, 10*w]);	% Weight matrix for the states
R_lqr = 1;                                  % Weight matrix for the control

K = lqr(A,B,Q_lqr, R_lqr);

Ar = A-B*K;
H = ss(Ar, B, C, D);
% figure(1)
% step(H)

%% Pre filter
C1 = [1, 0, 0, 0];
N1 = inv(C1*inv(B*K - A)*B);

%% Integral control
Ai = [0, C1;
      [0, 0, 0, 0]', A];
Bi = [0; B];
Ci = [[0;0], C];
Di = [D];

Q_lqr = diag([100*w, 100*w, 10*w, 10000*w, 10*w]);
R_lqr = 1;

KI = lqr(Ai, Bi, Q_lqr, R_lqr);
Ki = KI(1);
K0 = KI(2:5);

Ar = Ai - Bi*KI;
Hi = ss(Ar, Bi, Ci, Di);
% figure(2)
% step(Hi)

