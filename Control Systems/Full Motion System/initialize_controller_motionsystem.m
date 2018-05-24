% Preamble
clear all;
close all;
clc;

%% Initial Values
% Load in calibration structure
motionlab = load('calib.mat');

% Transformation matrices found from calibration 
Hgn = motionlab.calib.WORLD_TO_EM8000.H; 

% Constant
g = 9.81;   % Gravity [m/s^2]
dt = 1e-3;  % Step time

% Robot joint angles
q1_init = 0;
q2_init = 0;
q3_init = -pi/2;

q_init = [q1_init, q2_init, q3_init];

% Stewart Platform
eta_init = zeros(6,1);
v_init = zeros(6,1);
v_t_init = zeros(6,1);

% Tool point {r} -> {t}
p_init = [2.6192; 0; 2.24];
p_t_init = zeros(3,1);
p_tt_init = zeros(3,1);

% Winch
L_init = 2.0;
L_t_init = 0;

% Pendulum
phix_init = 15*pi/180*1;
phiy_init = 20*pi/180*1;
phi_init = [phix_init; phiy_init];

phi_t_init = zeros(2,1);
phi_tt_init = zeros(2,1);

% Tool-Point {n}->{t}
Pt_init = [-2.395; 3.811; -3.255];

% ---------------------------%
% Constructing the Estimator %
% ---------------------------%
%% Symbolic Derivation 

% State Vector
x = sym('x', [35,1], 'real');
% Input Vector
u = sym('u', [4,1], 'real');

% State vector
eta = x(1:6);
v = x(7:12);
v_t = x(13:18);
p = x(19:21);
p_t = x(22:24);
p_tt = x(25:27);
L = x(28);
L_t = x(29);
phi = x(30:31);
phi_t = x(32:33);
phi_tt = x(34:35);

% Input vector
p_tt = u(1:3);
L_ref = u(4);

% Tool-point motion relative to Stewart Platform neutral frame ({n} -> {t})
[pt, pt_t, pt_tt] = toolPointMotion(eta, v, v_t, p, p_t, p_tt);

% Non-linear function of system ODE
% x_t = f(x,u)
f = [stewartJacobian(eta)*v;
     v_t;
     zeros(6,1);
	 p_t;
	 p_tt;
	 zeros(3,1);
	 L_t;
	 winchMotion(L_ref, L, L_t);
	 phi_t;
	 pendulumDynamics(pt_tt, L, L_t, phi, phi_t)
	 zeros(2,1)];
   
% Non-linear function
% y = h(x,u)
h = [eta;
     v;
     zeros(6,1);
     p;
     p_t;
     zeros(3,1)
     L;
     0;
     phi;
     phi_t;
     zeros(2,1)];

% State-space
A_sym = jacobian(f,x);
B_sym = jacobian(f,u);
C_sym = jacobian(h,x);
D_sym = jacobian(h,u);

% Equilibrium states

% Input vector
u0 = [zeros(3,1); L_init];

% State vector
x0 = [eta_init;             
      v_init; 
	  v_t_init;
	  p_init;
	  p_t_init;
      p_tt_init;
      L_init;
	  L_t_init;
	  phi_init;
	  phi_t_init;
	  phi_tt_init];

% Update the Matrices 
% with the linearization around at equilibrium states
A_sym = subs(A_sym, x, x0);
A_sym = subs(A_sym, u, u0);

B_sym = subs(B_sym, x, x0);
B_sym = subs(B_sym, u, u0);

C_sym = subs(C_sym, x, x0);
C_sym = subs(C_sym, u, u0);

D_sym = subs(D_sym, x, x0);
D_sym = subs(D_sym, u, u0);

%% State Space Model

% Update the symoblic Matrices with constant values 
% and create numeric Matrices
A = double(subs(A_sym));
B = double(subs(B_sym));
C = double(subs(C_sym));
D = double(subs(D_sym));

% State space system
% G = ss(A,B,Cn,Dn);
G = ss(A,B,C,D);
Ob = obsv(G);
Cr = ctrb(G);
rank(Cr);
rank(Ob);

% Step response
% figure(1)
% step(G)

%% Extended Kalman Filter
Nx = size(x,1);    % Number of states
Nz = size(h,1);   % Number of measurements

% Process noise covariance
Q_EKF = eye(Nx)*0.001^2;
Q_EKF(13:18,13:18) = eye(6)*0.05^2; % V_t
Q_EKF(25:27,25:27) = eye(3)*0.05^2; % P_tt
Q_EKF(29,29) = 0.05^2;              % L_t
Q_EKF(32:33,32:33) = eye(2)*0.1^2;  % Phi_t
Q_EKF(32:33,32:33) = eye(2)*0.1^2;  % Phi_tt

% Measurement noise covariance
R_EKF = eye(Nz)*0.001^2;
R_EKF(1:6,1:6) = eye(6)*0.01^2;
R_EKF(7:12,7:12) = eye(6)*0.01^2;
R_EKF(30:31,30:31) = eye(2)*0.1^2;
R_EKF(31:32,31:32) = eye(2)*0.1^2;

% Initial state covariance
P_EKF = eye(Nx)*0.001^2;

% Initial states
EKF_init = [eta_init; 
            v_init; 
            v_t_init;
            p_init;
            p_t_init;
            p_tt_init;
            L_init;
            L_t_init;
            phi_init;
            phi_t_init;
            phi_tt_init];

% ----------------------------%
% Constructing the Controller %
% ----------------------------%
%% Symbolic Derivation Controller

% State Vector
x = sym('x', [10,1], 'real');
% Input Vector
u = sym('u', [3,1], 'real');

% State vector
p = x(1:3);
p_t = x(4:6);
phi = x(7:8);
phi_t = x(9:10);


% pt = x(13:15);
% pt_t = x(16:18);
% pt_tt = x(19:21);



% Input vector
p_tt = u(1:3);

% Assigning Intial values to parameters of the winch
L = L_init;
L_t = L_t_init;

% Assuming no platform motion
eta0 = zeros(6,1);
v0 = zeros(6,1);
v_t0 = zeros(6,1);

% Tool-point motion relative to Stewart Platform neutral frame ({n} -> {t})
[Pt, Pt_t, Pt_tt] = toolPointMotion(eta0, v0, v_t0, p, p_t, p_tt);
 
f = [p_t;
	 p_tt;
	 phi_t;
	 pendulumDynamics(Pt_tt, L, L_t, phi, phi_t)];
 
% Non-linear function
% y = h(x,u)
h = [p;
     p_t;
     phi;
     phi_t];
 
% State-space
A_sym = jacobian(f,x);
B_sym = jacobian(f,u);
C_sym = jacobian(h,x);
D_sym = jacobian(h,u);

% Equilibrium states

% Input vector
u0 = zeros(3,1); %; L_init];

% State vector
x0 = [[2.6919; 0; 2.24];    % P {r}->{t}
	  zeros(3,1);           % P_t
	  zeros(2,1);           % Phi
	  zeros(2,1)];          % Phi_t
  
% Update the Matrices 
% with the linearization around at equilibrium states

% A_sym = subs(A_sym, [u,x],[u0,x0]);
% B_sym = subs(B_sym, [u,x],[u0,x0]);
% C_sym = subs(C_sym, [u,x],[u0,x0]);
% D_sym = subs(D_sym, [u,x],[u0,x0]);  

% (The expressions above yields an error)
A_sym = subs(A_sym, x, x0);
A_sym = subs(A_sym, u, u0);

B_sym = subs(B_sym, x, x0);
B_sym = subs(B_sym, u, u0);

C_sym = subs(C_sym, x, x0);
C_sym = subs(C_sym, u, u0);

D_sym = subs(D_sym, x, x0);
D_sym = subs(D_sym, u, u0);

%% State Space Model

% Update the symoblic Matrices with constant values 
% and create numeric Matrices
A = double(subs(A_sym));
B = double(subs(B_sym));
C = double(subs(C_sym));
D = double(subs(D_sym));

% State space system
G = ss(A,B,C,D);

Ob = obsv(G);
Cr = ctrb(G);
rank(Cr);
rank(Ob);

%% State-Feedback

Nx = length(x); % Number of states

% Weighting factor
w = 10;

% Weight matrix for the states
Q_LQR = eye(Nx);

Q_LQR(1:3,1:3) = eye(3)*1000*w;     % P
Q_LQR(4:6,4:6) = eye(3)*10*w;       % P_t
Q_LQR(7:8,7:8) = eye(2)*1000*w;	% Phi
Q_LQR(9:10,9:10) = eye(2)*100*w;	% Phi_t

% Weight matrix for the control
R_LQR = eye(size(B,2));    % Size = columns of B

% LQR feedback gain
K = lqr(A,B,Q_LQR, R_LQR);

% New State-space model
Ar = A - B*K;

F = ss(Ar, B, C, D);

% Show step response
% figure(1)
% step(F)

%% Pre-Filter

Cm = C(1:3,:);
%       C(7,:)];               % Only direct output from input states
N = inv(Cm*inv(B*K - A)*B);  % Pre-filter constant


%% Integral Control
% Cm = C;
Ai = [zeros(size(Cm,1)), Cm;
      zeros(size(A,1), size(Cm,1)), A];
Bi = [zeros(size(Cm,1), size(B,2)); B];
Ci = [zeros(size(Cm,1)), Cm];
Di = zeros(size(Cm,1),size(Bi,2));

% Weight matrix for the states
Q_lqr_IC = diag([ 100*w,	100*w,	100*w, ...  % z (error states)
                  100*w,	100*w,	100*w, ...  % Pt
                   10*w,	 10*w,	 10*w, ...	% Pt_t
                 1000*w,   1000*w, ...          % Phi
                  100*w,    100*w]);            % Phi

% New Intergrator state
Ni = size(Cm,1); % Number of states

% Weight matrix for the states
Q_LQR_IC = eye(Nx + Ni);

Q_LQR_IC(1:3,1:3) = eye(3)*1000*w;      % Integrator States
Q_LQR_IC(4:6,4:6) = eye(3)*100*w;      % P
Q_LQR_IC(7:9,7:9) = eye(3)*10*w;      % P_t
Q_LQR_IC(10:11,10:11) = eye(2)*1000*w;	% Phi
Q_LQR_IC(12:13,12:13) = eye(2)*100*w;	% Phi_t

% Weight matrix for the control
R_LQR_IC = eye(size(B,2));    % Size = columns of B

% % Calculating gains
KI = lqr(Ai, Bi, Q_LQR_IC, R_LQR_IC);
Ki = KI(:,1:Ni);
Ko = KI(:,Ni+1:end);