function Pp = pendulumKinematics(Pt, phi, L)
    % Calculates the kinematics of the suspended load
    
    % Input: 
    %	Pt_tt   : Tool-point position
    %	phi     : Pendulum Angular position euler angles
    %   L       : Wire length
    
    
    % Tool-point position 
    xt = Pt(1);
    yt = Pt(2);
    zt = Pt(3);

    % Pendulum euler angles
    phix = phi(1);
    phiy = phi(2);

    % Pendulum position 
    xp = xt - L*sin(phix);
    yp = yt + L*cos(phix)*sin(phiy);
    zp = zt - L*cos(phix)*cos(phiy);

    Pp = [xp; yp; zp];
end