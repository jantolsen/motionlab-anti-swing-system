function stop = SimulinkPendulumPlot(xt, theta)
    % Creates a 2D-animation for the pendulum motion
    % Input:
    %   xt :   Position of tool-point
    %   theta : Euler angle of pendulum
    % Output:
    %   stop : Bool value to stop the animation
    
    % Persistent variables, to only be initialized once
    persistent h;           % Plot
    persistent L;
    
    stop = false;
    
    %% Initialization 
    % One time intitialization of plotting settings
    if isempty(h)
        % Plotting setup
        h = figure('Name','StewartPlatform Animation');
        L = 2.0;
        xlabel('x-axis')
        ylabel('y-axis')
%         zlabel('z-axis')
        xlim([-5, 5])
        ylim([-5, 1])
%         zlim([0, 8])
        hold on;
        grid on;
%         view(-40,20);
    end
    
     %% Drawing
    if ishandle(h)
        cla;
        Pt = [xt, 0];
        Pp = Pt + L*[sin(theta), -cos(theta)];
        
        plot_line(Pt, Pp);
        plot(Pp(1), Pp(2), 'ro', 'LineWidth', 2);

        drawnow;
    end
    
    %% Destructor
    if ~ishandle(h)
        stop = true;
    end

    function plot_line(p1, p2)
%         Pre-allocate
        line = zeros(2,2);
        line(1,:) = p1;
        line(2,:) = p2;

        plot(line(:,1), line(:,2), 'k')
    end
end