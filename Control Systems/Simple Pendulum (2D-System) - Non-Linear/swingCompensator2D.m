function x_tt = swingCompensator2D(theta, theta_t)
    % Compute the virtual damping to the simple pendulum system
    
    % Constants
    L = 2.0;    % Wire length
    d = 5;      % Damping coefficient
    
    x_tt = d*L*theta_t/cos(theta);
end