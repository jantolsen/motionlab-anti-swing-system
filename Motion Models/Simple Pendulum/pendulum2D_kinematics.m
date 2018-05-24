function Pp = pendulum_kinematics(Pt, theta)
    
    % Constants
    g = 9.81;   % Gravity
    L = 2.0;    % Wire length
    
    % Pendulum Kinematics
    Pp = Pt + L*[sin(theta); -cos(theta)];