function f = f_simulink3d(x,u)

    % Declaring system constants
    dt = 1e-3;  % Sample time [s]
    g = 9.81;   % Gravity [m/s^2]
    L = 2.0;    % Wire length [m]
    
    % State vector
    phix = x(1);
    phiy = x(2);
    phix_t = x(3);
    phiy_t = x(4);
    phix_tt = x(5);
    phiy_tt = x(6);
    
    % Input vector
    x_tt = u(1);
    y_tt = u(2);
    z_tt = u(3);
    
    f = [phix + phix_t*dt;
         phiy + phiy_t*dt;
         phix_t + (x_tt*cos(phix) + y_tt*sin(phix)*sin(phiy) ...
            - (z_tt - g)*sin(phix)*cos(phiy) ...
            - L*phiy_t^2*sin(phix)*cos(phix))*(dt/L);
         phiy_t + (-y_tt*cos(phiy) - (z_tt + g)*sin(phiy) ...
            + 2*L*phix_t*phiy_t*sin(phix))*(dt/L);
         phix_tt;
         phiy_tt];