function L_tt = winchMotion(L_ref, L, L_t)
    % Parameters
    Kdc = 1.0;
    omega = 4*2*pi;
    zeta = 0.7;
    
    % ODE
    L_tt = L_ref*Kdc*omega^2 - 2*zeta*omega*L_t - omega^2*L;
end