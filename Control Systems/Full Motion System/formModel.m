rfunction model = formModel()

%% Symbolic Derivation

% Parameters
syms dt 'real'              % Time step

% State Vector
x = sym('x', [35,1], 'real');
% Input Vector
u = sym('u', [4,1], 'real');

% % Stewart
% eta = sym('eta', [6,1], 'real');
% v = sym('v', [6,1], 'real');
% v_t = sym('v_t', [6,1], 'real');
% % Robot
% p = sym('p', [3,1], 'real');
% p_t = sym('p_t', [3,1], 'real');
% p_tt = sym('p_tt', [3,1], 'real');
% % Winch
% L = sym('L', [1,1], 'real');
% L_t = sym('L', [1,1], 'real');
% % Pendulum
% phi = sym('phi', [2,1], 'real');
% phi_t = sym('phi_t', [2,1], 'real');
% phi_tt = sym('phi_tt', [2,1], 'real');

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

% Tool-point motion relative to Stewart Platform neutral frame 
% {t}/{n} given in {n}
[pt, pt_t, pt_tt] = toolPointMotion(eta, v, v_t, p, p_t, p_tt);

% System ODE
ode = [stewartJacobian(eta)*v;
       v_t;
       zeros(6,1);
       p_t;
       p_tt;
       zeros(3,1);
       L_t;
       winchMotion(L_ref, L, L_t);
       phi_t;
       pendulum_dynamics(pt_tt, phi, phi_t, L, L_t);
       zeros(2,1)];

% State transition function and Jacobian
f = x + ode*dt;
f = simplify(f);

F = jacobian(f,x);
F = simplify(F);

% Measurement function and Jacobian
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
 
h = simplify(h);

H = jacobian(h,x);
H = simplify(H);

%% Make functions
% State transition
matlabFunction(f, 'File', 'f.m', 'Vars', {x, u, dt});
matlabFunction(F, 'File', 'fJacobian.m', 'Vars', {x, u, dt});

% Measurement
matlabFunction(h, 'File', 'h.m', 'Vars', {x, dt});
matlabFunction(H, 'File', 'hJacobian.m', 'Vars', {x, dt});

model.f = f;
model.F = F;
model.h = h;
model.H = H;

end
       
%% Child Functions

function L_tt = winchMotion(L_ref, L, L_t)
    % Parameters
    Kdc = 1.0;
    omega = 4*2*pi;
    zeta = 0.7;
    
    % ODE
    L_tt = L_ref*Kdc*omega^2 - 2*zeta*omega*L_t - omega^2*L;
end

function [Pt, Pt_t, Pt_tt] = toolPointMotion(eta, v, v_t, P, P_t, P_tt)
    
    % Calibrated transformation matrix {b} -> {r}
    Hbr = [-0.4972,	0.8676,	-0.0050,	-1.0820;
            0.8676,	0.4972,	 0.0012,     1.5360;
            0.0036,-0.0037, -1.0000,	-1.0245;
                 0,      0,       0,     1.0000];  
            
    % Body fixed velocity and acceleration skew matrices
    Sw = math3d.Skew(v(4:6));
    Sw_t = math3d.Skew(v_t(4:6));
            
    % Ship/Stewart relative to static csys {n} -> {b}
    Rnb = math3d.Rzyx(eta(4:6));
    Rnb_t = Rnb*Sw;
    Rnb_tt = Rnb*Sw*Sw + Rnb*Sw_t;
            
    % Constant offset between stewart platform and robot base
    % {b} -> {r}
    r = Hbr(1:3,4);
    Rbr = Hbr(1:3,1:3);

    % Tool-point position relative to Stewart platform
    % {t}/{n} given in {n}
    Pt = eta(1:3) + Rnb*(r + Rbr*P);

    % Tool-point velocity relative to Stewart platform
    % {t}/{n} given in {n}
    Pt_t = v(1:3) + Rnb_t*(r + Rbr*P) + Rnb*(Rbr*P_t);

    % Tool-point acceleration relative to Stewart platform
    % {t}/{n} given in {n}
    Pt_tt = v_t(1:3) + Rnb_tt*(r + Rbr*P) ...
           + 2*Rnb_t*(Rbr*P_t) + Rnb*(Rbr*P_tt);
end

function phi_tt = pendulum_dynamics(Pt_tt, phi, phi_t, L, L_t)
    
    % Constant Parameters
    g = 9.81;   % Gravity
    
    % Tool-point acceleration components
    xt_tt = Pt_tt(1); 
    yt_tt = Pt_tt(2); 
    zt_tt = Pt_tt(3);

    % Pendulum euler angles (angular position)
    phix = phi(1);
    phiy = phi(2);

    % Pendulum euler angles (angular velocity)
    phix_t = phi_t(1);
    phiy_t = phi_t(2);
    
     % Pendulum ODE of the euler angles
    % (found from euler-lagrange equation,
    %  derived by the use of Maple)
    phix_tt = (xt_tt*cos(phix) + yt_tt*sin(phix)*sin(phiy) ...
              - zt_tt*sin(phix)*cos(phiy) ...
              - g*sin(phix)*cos(phiy) ...
              - 2*L_t*phix_t ...
              - L*phiy_t^2*sin(phix)*cos(phix)) / L;

    phiy_tt = (- yt_tt*cos(phiy) - zt_tt*sin(phiy) ...
              - g*sin(phiy) + 2*L_t*phiy_t*cos(phix) ...
              + 2*L*phix_t*phiy_t*sin(phix)) ...
              / (L*cos(phix));

    phi_tt = [phix_tt; phiy_tt];
end

% Spherical Pendulum kinematics
function Pp = pendulum_kinematics(Pt, phi, L)
    % Tool-point position 
    % {r} -> {t}
    xt = Pt(1);
    yt = Pt(2);
    zt = Pt(3);

    % Pendulum euler angles
    phix = phi(1);
    phiy = phi(2);

    % Pendulum position 
    % {r} -> {p}
    xp = xt - L*sin(phix);
    yp = yt + L*cos(phix)*sin(phiy);
    zp = zt - L*cos(phix)*cos(phiy);

    Pp = [xp; yp; zp];
end

% Calculate the jacobian of Stewart motion
function J = stewartJacobian(eta)            
    % Input:
    %   eta : Motion driver
    % Output:
    %   J   : Jacobian Matrix

    T = Tphi(eta(4:6), 'zyx');

    J = [eye(3), zeros(3);
         zeros(3), T];
end

% Transformation matrix
function T = Tphi(phi, type)
    rx = phi(1);
    ry = phi(2);
    rz = phi(3);

    cx = cos(rx);
    sx = sin(rx);

    cy = cos(ry);
    sy = sin(ry);

    cz = cos(rz);
    sz = sin(rz);

    T = eye(3);

    if cy ~= 0.0
        if strcmp(type, 'xyz')
            T = [     cz,   -sz,  0;
                   cy*sz, cy*cz,  0;
                  -sy*cz, sy*sz, cy];

        elseif strcmp(type, 'zyx')
            T = [    cy, sx*sy, cx*sy;
                      0, cx*cy,-sx*cy;
                      0,    sx,    cx];
        end
        T = 1.0/cy*T;
    end
end
 
 
     