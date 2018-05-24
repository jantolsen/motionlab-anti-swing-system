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