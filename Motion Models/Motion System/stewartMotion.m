function [eta, v, v_t] = stewartMotion(eta, eta_t, eta_tt)
    % Calculate the relative Velocity and Acceleration component 
    % of the Stewart platform
    %
    % Input: 
    %	eta     : Orientation of {b} -> {n}
    %	eta_t   : Velocity of {b} -> {n}
    %   eta_tt	: Acceleration of {b} -> {n}
    %
    % Output:
    %   eta     : Orientation of {b} -> {n}
    %   v       : Body fixed velocity {n}/{b} given in {b}
    %   v_t      : Body fixed acceleration {n}/{b} given in {b}
    
% s : Angle Sequence
% ('zyx' is used for simulation)
% ('xyz' is used when motion data is obtained from MRU)
s = 'zyx';

% Velocity
J = vJacobian(eta, s);
v = J\eta_t;

% Acceleration
J_t = aJacobian(eta, eta_t, s);
v_t = J\(eta_tt - J_t*v);

end

function J = vJacobian(eta, s)
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);

    if strcmp(s, 'xyz')
        T11 = cos(psi)/cos(theta);
        T12 = -sin(psi)/cos(theta);
        T13 = 0;
        
        T21 = sin(psi);
        T22 = cos(psi);
        T23 = 0;
        
        T31 = -(cos(psi)*sin(theta))/cos(theta);
        T32 = (sin(psi)*sin(theta))/cos(theta);
        T33 = 1;
        
        T = [T11,	T12,	T13;
             T21,   T22,    T23;
             T31,   T32,    T33];
         
        J = [eye(3), zeros(3);
             zeros(3), T];
        
    elseif strcmp(s, 'zyx')
        T11 = 1;
        T12 = (sin(phi)*sin(theta))/cos(theta);
        T13 = (cos(phi)*sin(theta))/cos(theta);
        
        T21 = 0;
        T22 = cos(phi);
        T23 = -sin(phi);
        
        T31 = 0;
        T32 = sin(phi)/cos(theta);
        T33 = cos(phi)/cos(theta);
        
        T = [T11,	T12,	T13;
             T21,   T22,    T23;
             T31,   T32,    T33];
         
        J = [eye(3), zeros(3);
             zeros(3), T];
    end
end

function J_t = aJacobian(eta, eta_t, s)
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);
    
    phi_t = eta_t(4);
    theta_t = eta_t(5);
    psi_t = eta_t(6);

    if strcmp(s, 'xyz')
        
        T11 = (cos(psi)*sin(theta)*theta_t ...
               - cos(theta)*sin(psi)*psi_t)/cos(theta)^2;
        T12 = -(cos(theta)*cos(psi)*psi_t ...
              + sin(theta)*sin(psi)*theta_t)/cos(theta)^2;
        T13 = 0;
        
        T21 = cos(psi)*psi_t;
        T22 = -sin(psi)*psi_t;
        T23 = 0;
        
        T31 = -(cos(psi)*theta_t ...
              - cos(theta)*sin(theta)*sin(psi)*psi_t)/cos(theta)^2;
        T32 = (sin(psi)*theta_t ...
             + cos(theta)*cos(psi)*sin(theta)*psi_t)/cos(theta)^2;
        T33 = 0;
        
        T = [T11,	T12,	T13;
             T21,   T22,    T23;
             T31,   T32,    T33];
         
        J_t = [zeros(3), zeros(3);
               zeros(3), T];
        
    elseif strcmp(s, 'zyx')
      
        T11 = 0;
        T12 = (sin(phi)*theta_t ...
            + cos(phi)*cos(theta)*sin(theta)*phi_t)/cos(theta)^2;
        T13 = (cos(phi)*theta_t ...
            - cos(theta)*sin(phi)*sin(theta)*phi_t)/cos(theta)^2;
        
        T21 = 0;
        T22 = -sin(phi)*phi_t;
        T23 = -cos(phi)*phi_t;
        
        T31 = 0;
        T32 = (cos(phi)*cos(theta)*phi_t ...
            + sin(phi)*sin(theta)*theta_t)/cos(theta)^2;
        T33 = -(cos(theta)*sin(phi)*phi_t ...
            - cos(phi)*sin(theta)*theta_t)/cos(theta)^2;
        T = [T11,	T12,	T13;
             T21,   T22,    T23;
             T31,   T32,    T33];
         
        J_t = [zeros(3), zeros(3);
               zeros(3), T];
    end
end