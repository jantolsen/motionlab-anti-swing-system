% Plot the pose of the stewart platform
% This includes the reference world coordinate
function Hgb = plot_stewartplatform(eta, Hg, Hgn)
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