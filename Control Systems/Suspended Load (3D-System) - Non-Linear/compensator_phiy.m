function y_tt = compensator_phiy(phix, phiy, phiy_t, d_y)
    % Constants
    L = 2.0;  % Wire Length
   
    y_tt = L*d_y*phiy_t*cos(phix) /cos(phiy);