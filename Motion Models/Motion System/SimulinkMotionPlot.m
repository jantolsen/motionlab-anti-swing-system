function stop = SimulinkMotionPlot(q, eta, Pt, Pp)
    % Creates a 3D-animation for the motion of the motion-lab system
    % Requires values for the Comau Robot joints and stewart platform
    % Input: 
    %	q	: Robot joint angular position
    %   eta	: Stewart platform orientation
    %	Pt	: Tool-point position
    %	phi	: Pendulum Angular position euler angles
    % Output:
    %	stop :  Bool value to stop the anitmation
    
    % Persistent variables, to only be initialized once
    persistent h;           % Plot
    persistent motionlab;   % Calibration structure
    % Transformation matrices
    persistent Hgn;         % World to EM8000  {g} -> {n}
    persistent Hbr;         % EM8000 to Comau  {b} -> {r}
    persistent Hg;          % World coordinate system (Reference CSYS)
    
    stop = false;
    
    %% Initialization 
    % One time intitialization of plotting settings
    if isempty(h)
        % Close previous all plots
        close all
        
        % Plotting setup
        h = figure('Name','Motion Lab Animation');
        
        xlabel('x-axis')
        ylabel('y-axis')
        zlabel('z-axis')
        xlim([-5, 3])
        ylim([-3, 5])
        zlim([0, 8])
        
        hold on;
        grid on;
        view(-30,20);
        
        % Load in calibration structure
        motionlab = load('calib.mat');
        
        % Transformation matrices found from calibration 
        Hgn = motionlab.calib.WORLD_TO_EM8000.H;  % {g} -> {n}
        Hbr = motionlab.calib.EM8000_TO_COMAU.H;  % {b} -> {r}
        
        % Reference coordinate system (world coordinate)
        Hg = eye(4);
    end
    
    %% Drawing
    if ishandle(h)
        cla;
        
        % Plot the motion of the Stewart Platform
        Hgb = plot_stewartplatformpose(eta, Hg, Hgn);
        
        % Tranformation from World CSYS to Robot Base
        Hgr = Hgb*Hbr;
        
        % Plot Robot pose relative to input CSYS
        plot_robotpose(q, Hgr);
        
        % Plot Pendulum pose relative to input CSYS
        plot_pendulum(Pt, Pp, Hgn);
        
        drawnow;
    end
    
    %% Destructor
    if ~ishandle(h)
        stop = true;
    end
    
end

%% Child functions

% Plot the pose of the stewart platform
% This includes the reference world coordinate
function Hgb = plot_stewartplatformpose(eta, Hg, Hgn)
    % Input:
    %   eta     : Stewart platform orientation
    %   Hg      : World coordinate system (Reference CSYS)
    %   Hgn     : Transformation matrix, World to EM8000 {g} -> {n}
    % Output:
    %   Hgb     : Transformation matrix, World to EM8000 {g} -> {b}
    
    % Motion from intial platform pose to motion
    % {n} -> {b}
    Hnb = eye(4);
    Hnb(1:3,1:3) = math3d.Rzyx(eta(4:6));	% Rotation matrix
    Hnb(1:3,4) = eta(1:3);                  % Translation vector

    % Relative transformation
    Hgb = Hgn*Hnb;  % World to EM8000 (motion) {g} -> {b}

    % Plot coordinate system
    plot_coordinatesystem(Hg);      % Reference
    plot_coordinatesystem(Hgn);     % EM8000 static
    plot_coordinatesystem(Hgb);     % EM8000 motion
end

% Plot the pose of the Comau Robot
% This includes links and local coordinate systems
function plot_robotpose(q, Hr)
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
    Hr1 = Hr*T1;   % Robot base to joint q1 {r -> q1}
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

% Plot the position of the Pendulum
function plot_pendulum(Pt, Pp, Hr)
    % Inputs:
    % Pt    :   Tool-point position [3x]
    % Pp    :   Pendulum position [3x1]
    % Hr    :   Transformation matrix of the robot base [4x4]
    
    % Transforming the Position vector relative to world CSYS
    Pt = Hr*[Pt; 1];
    Pp = Hr*[Pp; 1];
    
    % Correction of Z-direction
    L = 2.0;    % Initial Wire length
    Pp(3) = Pp(3) - 2*L;
    
    % Pendulum positions
    xp = Pp(1);
    yp = Pp(2);
    zp = Pp(3);
    
    % Plot wire
    plot_line(Pt(1:3), Pp(1:3));

    % Plot Load
    plot3(xp, yp, zp, 'ro', 'LineWidth', 5);
end
        
% Plot a line in 3D plots
function plot_line(p1, p2)
    % Inputs:
    %   p1  :   Point 1 [x1; y1; z1]
    %   p2  :   Point 2 [x2; y2; z2]

    % Pre-allocate
    line = zeros(2,3);

    line(1,:) = p1;
    line(2,:) = p2;

    X = line(:,1);
    Y = line(:,2);
    Z = line(:,3);

    plot3(X, Y, Z, 'k', 'LineWidth', 1)
end
        
% Plot the coordinate system in 3D plots
% with default rgb-colors to the related axis.
function plot_coordinatesystem(H)
    % Inputs:
    %   H       :   Transformation-matrix (size = 4x4)

    % Pre-allocate
    axis = zeros(2,3);
    
    % Rotation and Translation
    d = H(1:3,4);   % Translation vector
    R = H(1:3,1:3); % Rotation matrix

    % Axis colors
    colors = {'r', 'g', 'b'};
    
    % Length of each axis in csys
    csys_len = 0.5;
    
    % Plot all 3 axis
    % (Looping through plotting one axis at a time)
    for i = 1:3
        axis(1,:) = d;                      % csys origo [x0 y0 z0]
        axis(2,:) = d + R(:,i)*csys_len;    % end point of axis [x y z]

        X = axis(:,1);  % Vector of the x-component
        Y = axis(:,2);  % Vector of the y-component
        Z = axis(:,3);  % Vector of the z-component

        % Plot one of the axis with the related color
        plot3(X, Y, Z, colors{i}, 'LineWidth', 1)
    end
end
        
