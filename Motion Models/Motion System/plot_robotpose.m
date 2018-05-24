function plot_robotpose(q, Hr)
    % Plot the pose of the Comau Robot
    % This includes links and local coordinate systems

    % Inputs:
    % Hr : Transformation matrix of the robot base [4x4]
    % q  : Robot joints [3x1]
    
    % Static link lengths
    a1 = 0.350;
    a2 = 1.160;
    a3 = 0.250;
    d1 = 0.830;
    d4 = 1.4922;
    d6 = 0.210;
    dt = 0.567;
    L = d4 + d6 + dt;
    
    % Joint angles
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);

    % Robot DH Table
    T1 = math3d.DH(-q1, d1, a1, pi/2);
    T2 = math3d.DH(pi/2 - q2, 0, a2, 0);
    T3 = math3d.DH(q3 + pi/2 + q2, 0, a3, pi/2);
    T4 = math3d.DH(pi, L, 0, 0);

    % Relative transformation
    Hr1 = Hr*T1;    % Robot base to joint q1 {r -> q1}
    Hr2 = Hr1*T2;   % Robot base to joint q2 {r -> q2}
    Hr3 = Hr2*T3;   % Robot base to joint q3 {r -> q2}
    Hr4 = Hr3*T4;   % Robot base to tool-point {r -> t}

    % Plot coordinate systems
    plot_coordinatesystem(Hr);      % Base
    plot_coordinatesystem(Hr1);     % Joint q1
    plot_coordinatesystem(Hr2);     % Joint q2
    plot_coordinatesystem(Hr3);     % Joint q3
    plot_coordinatesystem(Hr4);     % Tool-Point

    % Plot links
    plot_line(Hr(1:3,4), Hr1(1:3,4));	% Base to joint q1        
    plot_line(Hr1(1:3,4), Hr2(1:3,4));	% joint q1 to joint q2 
    plot_line(Hr2(1:3,4), Hr3(1:3,4));	% joint q2 to joint q3 
    plot_line(Hr3(1:3,4), Hr4(1:3,4));	% joint q3 to Tool-point
end