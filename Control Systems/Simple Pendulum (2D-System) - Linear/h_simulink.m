function h = h_simulink(x)
    % Jacboian of the Measurement function for 
    % the Extended Kalman Filter 
    % related to the linear 2D Pendulum system
    
    % State vector
    % x(1) : x
    % x(2) : x_t
    % x(3) : theta
    % x(4) : theta_t
    
    h = [x(1), x(3)];
end
