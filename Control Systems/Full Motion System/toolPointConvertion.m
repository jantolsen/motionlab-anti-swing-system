function [P, P_t, P_tt] = toolPointConvertion(eta, v, v_t, Pt, Pt_t, Pt_tt)
    % Transformation matrix found from calibration
    % Robot pose relative to motion frame of Stewart Platform
    % {b} -> {r}
    Hbr = [-0.4972,	0.8676,	-0.0050,	-1.0820;    
            0.8676,	0.4972,	 0.0012,     1.5360;
            0.0036,-0.0037, -1.0000,	-1.0245;
                 0,      0,       0,     1.0000];  
            
    % Body fixed velocity and acceleration skew matrices
    Sw = math3d.Skew(v(4:6));
    Sw_t = math3d.Skew(v_t(4:6));
            
    % Ship/Stewart relative to static csys {n} -> {b}
    Rnb = math3d.Rzyx(eta(4:6));
    Rnb_t = Rnb*Sw;
    Rnb_tt = Rnb*Sw*Sw + Rnb*Sw_t;
            
    % Constant offset between stewart platform and robot base
    % {b} -> {r}
    r = Hbr(1:3,4);
    Rbr = Hbr(1:3,1:3);
    
    % Tool-point position relative to Robot Base
    % {t}/{r} given in {r}
    P = inv(Rbr)*(inv(Rnb)*(Pt - eta(1:3)) - r);
    
    % Tool-point velocity relative to Robot Base
    % {t}/{r} given in {r}
    P_t = inv(Rbr)*(inv(Rnb)*(Pt_t - v(1:3) - Rnb_t*(r + Rbr*P)));
    
    % Tool-point acceleration relative to Robot Base
    % {t}/{r} given in {r}
    P_tt = inv(Rbr)*(inv(Rnb)*(Pt_tt - v_t(1:3) - Rnb_tt*(r + Rbr*P) ...
                               - 2*Rnb_t*(Rbr*P_t)));