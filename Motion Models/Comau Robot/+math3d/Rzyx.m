function R = Rzyx(phi)

rx = phi(1);
ry = phi(2);
rz = phi(3);

R = [
    cos(ry)*cos(rz), cos(rz)*sin(rx)*sin(ry) - cos(rx)*sin(rz), sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry);
    cos(ry)*sin(rz), cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz), cos(rx)*sin(ry)*sin(rz) - cos(rz)*sin(rx);
           -sin(ry),                           cos(ry)*sin(rx),                           cos(rx)*cos(ry)
];

R11 = cos(ry)*cos(rz);
R12 = cos(rz)*sin(rx)*sin(ry);
R13 = sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry);

R21 = cos(ry)*sin(rz);
R22 = cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz);
R23 = cos(rx)*sin(ry)*sin(rz) - cos(rz)*sin(rx);

R31 = -sin(ry);
R32 = cos(ry)*sin(rx);
R33 = cos(rx)*cos(ry);

R = [R11, R12, R13;
     R21, R22, R23;
     R31, R32, R33];