function F = fJacobian_simulink(x,u)
    % Jacboian of the State-transition function for 
    % the Extended Kalman Filter 
    % related to the linear 2D Pendulum system
    
    % Declaring system constants
    dt = 1e-3;  % Sample time [s]
    g = 9.81;   % Gravity [m/s^2]
    L = 2.0;    % Wire length [m]
    
    % State vector
    % x(1) : theta
    % x(2) : theta_t
    % x(3) : theta_tt
    
    F = [1, -(u*sin(x(1))/L - g*cos(x(1))/L)*dt, 0;
         dt, 1, 0;
         0, 0, 1];