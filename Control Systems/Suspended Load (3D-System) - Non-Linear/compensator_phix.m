function x_tt = compensator_phix(phix, phix_t, d_x)
    % Constants
    L = 2.0;  % Wire Length
    
    x_tt = -L*d_x*phix_t /cos(phix);