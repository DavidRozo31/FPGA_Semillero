Rz = @(t) [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
Rx = @(t) [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];


DH:
thetaZ1 = pi/4;
thetaX1 = pi/2;
Lz1 = 5;
Lx1 = 0;

T01 =   [cos(thetaZ1) -cos(thetaX1)*sin(thetaZ1)  sin(thetaX1)*sin(thetaZ1) Lx1*cos(thetaZ1);
         sin(thetaZ1)  cos(thetaX1)*cos(thetaZ1) -cos(thetaZ1)*sin(thetaX1) Lx1*sin(thetaZ1);
         0             sin(thetaX1)               cos(thetaX1)              Lz1;
         0                        0                        0                1];


thetaZ2 = 0;
thetaX2 = 0;
Lz2 = 0;
Lx2 = 10.7;

T12 =   [cos(thetaZ2) -cos(thetaX2)*sin(thetaZ2)  sin(thetaX2)*sin(thetaZ2)  Lx2*cos(thetaZ2);
         sin(thetaZ2)  cos(thetaX2)*cos(thetaZ2) -cos(thetaZ2)*sin(thetaX2)  Lx2*sin(thetaZ2);
         0             sin(thetaX2)               cos(thetaX2)               Lz2;
         0                        0                        0                 1];


thetaZ3 = 0;
thetaX3 = 0;
Lz3 = 0;
Lx3 = 13;

T23 =   [cos(thetaZ3) -cos(thetaX3)*sin(thetaZ3)  sin(thetaX3)*sin(thetaZ3)  Lx3*cos(thetaZ3);
         sin(thetaZ3)  cos(thetaX3)*cos(thetaZ3) -cos(thetaZ3)*sin(thetaX3)  Lx3*sin(thetaZ3);
         0             sin(thetaX3)               cos(thetaX3)               Lz3;
         0                        0                        0                 1];


thetaZ4 = pi/2 + pi/2;
thetaX4 = pi/2;
Lz4 = 0;
Lx4 = 0;

T34 =   [cos(thetaZ4) -cos(thetaX4)*sin(thetaZ4)  sin(thetaX4)*sin(thetaZ4)  Lx4*cos(thetaZ4);
         sin(thetaZ4)  cos(thetaX4)*cos(thetaZ4) -cos(thetaZ4)*sin(thetaX4)  Lx4*sin(thetaZ4);
         0             sin(thetaX4)               cos(thetaX4)               Lz4;
         0                        0                        0                 1];


thetaZ5 = pi/4;
thetaX5 = 0;
Lz5 = 18;
Lx5 = 0;

T45 =   [cos(thetaZ5) -cos(thetaX5)*sin(thetaZ5)  sin(thetaX5)*sin(thetaZ5)  Lx5*cos(thetaZ5);
         sin(thetaZ5)  cos(thetaX5)*cos(thetaZ5) -cos(thetaZ5)*sin(thetaX5)  Lx5*sin(thetaZ5);
         0             sin(thetaX5)               cos(thetaX5)               Lz5;
         0                        0                        0                 1];

T05 = T01 * T12 * T23 * T34 * T45

m = T05(1:3, 1:3);
r = rad2deg(tr2rpy(m, 'zyx'))

pos = T05(1:3, 4)
