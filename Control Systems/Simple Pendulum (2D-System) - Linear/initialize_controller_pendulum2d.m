clear all;
close all;
clc;

%% Initial Values

% Constants
dt = 1e-3;  % Time step
L = 4.0;    % Wire length
g = 9.81;   % Gravity [m/s^2]

% Initial values
x_init = 0;
x_t_init = 0;
theta_init = 20*pi/180;
theta_t_init = 0;

%% Symbolic

% Parameters
% syms g L 'real'

x = sym('x', [4,1], 'real');
u = sym('u', 'real');

% State vector
xt = x(1);
xt_t = x(2);
theta = x(3);
theta_t = x(4);

% Input vector
xt_tt = u;

% Non-linear function of system ODE
% x_t = f(x,u)
f = [xt_t;
     u;
     theta_t;
     (-g*sin(theta) - cos(theta)*xt_tt)/L];

% Non-linear function
% y = h(x,u)
h = [xt, theta];

% State-space
A_sym = jacobian(f, x);
B_sym = jacobian(f, u);
C_sym = jacobian(h, x);
D_sym = jacobian(h, u);

% Equilibrium point
u0 = 0;
x0 = [0; 0; 0; 0];

% Update the Matrices 
% with the linearization around at equilibrium states
% A_sym = subs(A_sym, [x,u],[x0,u]);
% B_sym = subs(B_sym, [x,u],[x0,u]);
% C_sym = subs(C_sym, [x,u],[x0,u]);
% D_sym = subs(D_sym, [x,u],[x0,u]);

A_sym = subs(A_sym, x, x0);
A_sym = subs(A_sym, u, u0);

B_sym = subs(B_sym, x, x0);
B_sym = subs(B_sym, u, u0);

C_sym = subs(C_sym, x, x0);
C_sym = subs(C_sym, u, u0);

D_sym = subs(D_sym, x, x0);
D_sym = subs(D_sym, u, u0);


%% Simulink State-space

% Update the symoblic Matrices with constant values 
% and create numeric Matrices
A = double(subs(A_sym));
B = double(subs(B_sym));
C = double(subs(C_sym));
D = double(subs(D_sym));

% C = C(1:2:3,:)    % Removing zero row entries
% State space system
G = ss(A,B,C,D);

Ob = obsv(G);
Cr = ctrb(G);
rank(Cr);
rank(Ob);

% Step response
% figure(1)
% step(G)

%% Estimator Design
% Covariance matrices
P = 1e-3;                               % Initial state covariance
Q = diag([0, 0.001^2, 0.01^2, 0.01^2]); % Process noise covariance
R = diag([0.001^2, 0.01^2]);            % Measurement noise covariance

%% State-Feedback gain
% Weighting factor
w = 10;

% Weight matrix for the states
Q_LQR = diag([10*w, 10*w, 1000*w, 10*w]);

% Weight matrix for the control
R_LQR = 1;                                  

% LQR feedback gain
K = lqr(A,B,Q_LQR, R_LQR);

% New State-space model
Ar = A - B*K;

F = ss(Ar, B, C, D);
% figure(1)
% step(F)

%% Pre filter
Cm = [1, 0, 0, 0];              % Only direct output from input states
N = inv(Cm*inv(B*K - A)*B);    % Pre-filter constant

%% Integral control
Ai = [zeros(size(Cm,1)), -Cm;
      zeros(size(A,1), size(Cm,1)), A];
Ai_old = [zeros(size(Cm,1)), Cm;
      zeros(size(A,1), size(Cm,1)), A];  
Bi = [zeros(size(Cm,1), size(B,2)); B];
Ci = [zeros(size(Cm,1)), Cm];
Di = zeros(size(Cm,1),size(Bi,2));

% Weight matrix for the states
% width additional error state(s)
Q_LQR = diag([100*w, 100*w, 10*w, 10000*w, 10*w]);

% Weight matrix for the control
R_LQR = 1;

% Calculating gains
KI_old = lqr(Ai_old, Bi, Q_LQR, R_LQR);
Ki = KI_old(1);
K0 = KI_old(2:5);

KI = lqr(Ai, Bi, Q_LQR, R_LQR);
Ke = -KI(1);
Ko = KI(2:5);

Ar = Ai - Bi*KI;


% Cc = [[0;0],C];
% Dc = zeros(size(Cc,1),size(Bi,2));
% Hi = ss(Ar, Bi, Cc, Dc);
% 
% figure(2)
% step(Hi)

