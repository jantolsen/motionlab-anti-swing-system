function f = f_simulink(x,u)
    % State-transition function for the Extended Kalman Filter 
    % related to the linear 2D Pendulum system
    
    % Declaring system constants
    dt = 1e-3;  % Sample time [s]
    g = 9.81;   % Gravity [m/s^2]
    L = 2.0;    % Wire length [m]
    
    % State vector
    % x(1) : x
    % x(2) : x_t
    % x(3) : theta
    % x(4) : theta_t
     
 f = [x(1) + x(2)*dt;
      x(2) + u*dt;
      x(3) + x(4)*dt;
      x(4) + (-g*sin(x(3)) - cos(x(3))*u)*dt/L];
end