function R = Rq(q)

    % https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    q0 = q(1);
    q1 = q(2);
    q2= q(3);
    q3 = q(4);

    R(1,1) = 1 - 2*q2^2 - 2*q3^2;
    R(1,2) = 2*(q1*q2 - q3*q0);
    R(1,3) = 2*(q1*q3 + q2*q0);

    R(2,1) = 2*(q1*q2  + q3*q0);
    R(2,2) = 1 - 2*q1^2 - 2*q3^2;
    R(2,3) = 2*(q2*q3 - q1*q0);

    R(3,1) = 2*(q1*q3 - q2*q0);
    R(3,2) = 2*(q2*q3 + q1*q0);
    R(3,3) = 1 - 2*q1^2 - 2*q2^2;

end


