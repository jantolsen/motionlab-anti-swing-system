function phi_tt = pendulum_dynamics(Pt_tt, phi, phi_t)
    
    g = 9.81;   % Gravity
    L = 2.0;    % Wire length
    L_t = 0;    % Constant wire length
    
    % Tool-point acceleration components
    xt_tt = Pt_tt(1); 
    yt_tt = Pt_tt(2); 
    zt_tt = Pt_tt(3);

    % Pendulum euler angles (angular position)
    phix = phi(1);
    phiy = phi(2);

    % Pendulum euler angles (angular velocity)
    phix_t = phi_t(1);
    phiy_t = phi_t(2);
    
     % Pendulum ODE of the euler angles
    % (found from euler-lagrange equation,
    %  derived by the use of Maple)
    phix_tt = (xt_tt*cos(phix) + yt_tt*sin(phix)*sin(phiy) ...
              - zt_tt*sin(phix)*cos(phiy) ...
              - g*sin(phix)*cos(phiy) ...
              - 2*L_t*phix_t ...
              - L*phiy_t^2*sin(phix)*cos(phix)) / L;

    phiy_tt = (- yt_tt*cos(phiy) - zt_tt*sin(phiy) ...
              - g*sin(phiy) + 2*L_t*phiy_t*cos(phix) ...
              + 2*L*phix_t*phiy_t*sin(phix)) ...
              / (L*cos(phix));

    phi_tt = [phix_tt; phiy_tt];
