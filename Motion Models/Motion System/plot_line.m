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