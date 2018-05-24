function h = h_simulink3d(x)

    % State vector
    phix = x(1);
    phiy = x(2);
    phix_t = x(3);
    phiy_t = x(4);
    
    h = [phix, phiy, phix_t, phiy_t];