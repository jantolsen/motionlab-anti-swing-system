function stop = SimulinkMotionPlot_ComauPendulum(q, Pt, Pp)
    % Creates a 3D-animation for the motion of the motion-lab system
    % Requires values for the Comau Robot joints and stewart platform
    % Input: 
    %	q	: Robot joint angular position
    %	Pt	: Tool-point position
    %	phi	: Pendulum Angular position euler angles
    % Output:
    %	stop :  Bool value to stop the anitmation
    
    % Persistent variables, to only be initialized once
    persistent h;           % Plot
    persistent Hr;          % Transformation Matrix
    stop = false;
    
    stop = false;
    
    %% Initialization 
    % One time intitialization of plotting settings
    if isempty(h)
        % Plotting setup
        h = figure('Name','StewartPlatform Animation');
        
        xlabel('x-axis')
        ylabel('y-axis')
        zlabel('z-axis')
        xlim([-5, 5])
        ylim([-5, 5])
        zlim([0, 8])
        hold on;
        grid on;
        view(-40,20);
        
        % Reference coordinate system (world coordinate)
        Hr = eye(4);
    end
    
    %% Drawing
    if ishandle(h)
        cla;
        plot_robotpose(q, Hr); % Plot Robot pose
        plot_pendulum(Pt, Pp, Hr); % Plot Pendulum pose
        drawnow;
    end
    
    %% Destructor
    if ~ishandle(h)
        stop = true;
    end
end