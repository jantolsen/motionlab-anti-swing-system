%% Symbolic derivation of the Forward Kinematics
% This script is used to derive the governing equations for the
% forward kinematic of the Comau Robot

% Symbolic variables and Dimensions
syms q1 q2 q3
syms q1_t q2_t q3_t
syms q1_tt q2_tt q3_tt
syms d1 a1 a2 a3 L

% Variable Definitions
q = [q1; q2; q3];
q_t = [q1_t; q2_t; q3_t];
q_tt = [q1_tt; q2_tt; q3_tt];

% Constructing the DH-table
A1 = DH(-q1, d1, a1, sym(pi)/2);
A2 = DH(sym(pi)/2 - q2, 0, a2, 0);
A3 = DH(q3 + sym(pi)/2 + q2, 0, a3, sym(pi)/2);
A4 = DH(sym(pi), L, 0, 0);

% Transformation matrix
% (Robot base to tool-point {r} -> {t})
T04 = A1*A2*A3*A4;
T04 = simplify(expand(T04));  % Cleaner expression 

% Rotation matrix
Rt = T04(1:3,1:3);

% Tool-point position
Pt = T04(1:3,4);

% Jacobian matrix
J = jacobian(Pt,q);

% Tool-point velocity
Pt_t = J*q_t;

% Jacobiandot
J_t = jacobian(J*q_t,q);

% Tool-point acceleration
Pt_tt = J_t*q_tt;

%% Functions

% Transformation Matrix  for Z-rotation
function Rz = RotZ(theta)
    Rz = [cos(theta), -sin(theta), 0, 0;
          sin(theta), cos(theta),  0, 0;
          0         , 0         ,  1, 0;
          0         , 0         ,  0, 1];
end

% Transformation Matrix  for X-rotation
function Rx = RotX(alpha)
    Rx = [1, 0         , 0          , 0;
          0, cos(alpha), -sin(alpha), 0;
          0, sin(alpha), cos(alpha) , 0;
          0, 0         , 0          , 1];
end

% Transformation Matrix for Z-translation
function Tz = TransZ(d)
    Tz = [1, 0, 0, 0;
          0, 1, 0, 0;
          0, 0, 1, d;
          0, 0, 0, 1];
end

% Transformation Matrix for X-translation
function Tx = TransX(a)
    Tx = [1, 0, 0, a;
          0, 1, 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
end

% Denavit-Hartenberg Table Convention
function A = DH(theta, d, a, alpha)
    A = RotZ(theta)*TransZ(d)*TransX(a)*RotX(alpha);
end
