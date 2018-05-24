clear all;
close all;
clc;

%% Symbolioc derivation of the 2D pendulum Lagrangian

% Symbolic variables and dimensions
syms xt zt xt_t zt_t Pp Pp_t
syms Lw Lw_t phi phi_t mp g

% Position
Pp = [xt_t + Lw*sin(phi);
       zt_t - Lw*cos(phi)];

% Velocity
Pp_t = [xt_t + Lw*cos(phi)*phi_t + Lw_t*sin(phi);
         zt_t + Lw*sin(phi)*phi_t - Lw_t*cos(phi)];
     
% Kinetic Energy
Ek = 1/2 * mp * Pp_t.'*Pp_t;

simplify(expand(Ek))    % Simplified expression

% Potential Energy
Ep = mp*g*zt - mp*g*Lw*cos(phi);

% Lagrange
L = Ek - Ep;

%% Symbolic derivation of the 3D pendulum Lagrangian

% Symbolic variables and dimensions
syms xt yt zt
syms xt_t yt_t zt_t
syms phix phiy phix_t phiy_t
syms Lw Lw_t mp g

% Position
Pp = [xt - Lw*sin(phix);
      yt + Lw*cos(phix)*sin(phiy);
      zt - Lw*cos(phix)*cos(phiy)];
  
% Velocity
Pp_t = [xt_t - Lw*phix_t*cos(phix) - Lw*sin(phix);
        yt_t - Lw*phix_t*sin(phix)*sin(phiy) ...
        + Lw*phiy_t*cos(phix)*cos(phiy) + Lw_t*cos(phix)*sin(phiy);
        zt_t + Lw*phix_t*sin(phix)*cos(phiy) ...
        + Lw*phiy_t*cos(phix)*sin(phiy) - Lw_t*cos(phix)*cos(phiy)];
    
% Kinetic Energy
Ek = 1/2*mp*Pp_t.'*Pp_t;

simplify(expand(Ek))    % Simplified expression