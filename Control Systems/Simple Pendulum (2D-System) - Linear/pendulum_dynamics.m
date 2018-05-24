function theta_tt = pendulum_dynamics(x_tt, theta, theta_t)
    
    % Constants
    g = 9.81;   % Gravity
    L = 2.0;    % Wire length
    
    % Pendulum ODE
    theta_tt = (-g*sin(theta) - cos(theta)*x_tt)/L;