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