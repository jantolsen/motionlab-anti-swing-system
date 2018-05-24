% Calculate the jacobian of Stewart motion
function J = stewartJacobian(eta)            
    % Input:
    %   eta : Motion driver
    % Output:
    %   J   : Jacobian Matrix

    T = Tphi(eta(4:6), 'zyx');

    J = [eye(3), zeros(3);
         zeros(3), T];
end

% Transformation matrix
function T = Tphi(phi, type)
    rx = phi(1);
    ry = phi(2);
    rz = phi(3);

    cx = cos(rx);
    sx = sin(rx);

    cy = cos(ry);
    sy = sin(ry);

    cz = cos(rz);
    sz = sin(rz);

    T = eye(3);

    if cy ~= 0.0
        if strcmp(type, 'xyz')
            T = [     cz,   -sz,  0;
                   cy*sz, cy*cz,  0;
                  -sy*cz, sy*sz, cy];

        elseif strcmp(type, 'zyx')
            T = [    cy, sx*sy, cx*sy;
                      0, cx*cy,-sx*cy;
                      0,    sx,    cx];
        end
        T = 1.0/cy*T;
    end
end