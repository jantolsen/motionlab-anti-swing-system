function stop = SimulinkMotionPlot_Comau(q)
    % Creates a 3D-animation for the motion of the Comau Robot
    % Requires values for the Comau Robot joints
    
    % Persistent variables, to only be initialized once
    persistent h;           % Plot
    persistent Hr;          % Transformation Matrix
    stop = false;
    
    %% Initialization 
    % One time intitialization of plotting settings
    if isempty(h)
        % Plotting setup
        h = figure('Name','ComauRobot Animation');
        xlabel('x-axis')
        ylabel('y-axis')
        zlabel('z-axis')
        xlim([-2, 4])
        ylim([-2, 4])
        zlim([-1, 4])
        hold on;
        grid on;
        view(-20,20);
        
        % Relative Transformation to Robot Base
        Hr = eye(4);
    end
    
    %% Drawing
    if ishandle(h)
        cla;
        plot_robotpose(q, Hr);
        drawnow;
    end
    
    %% Destructor
    if ~ishandle(h)
        stop = true;
    end
end

%% Child functions

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