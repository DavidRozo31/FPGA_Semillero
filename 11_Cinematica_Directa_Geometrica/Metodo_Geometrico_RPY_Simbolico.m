% Metodo geometrico -- construccion matriz por matriz, EN SIMBOLICO
% (mismo codigo/estilo que Metodo_Geometrico_RPY.m, pero con theta1..theta5
% sin numero, para ver como queda cada matriz). Requiere Symbolic Math Toolbox.

syms theta1 theta2 theta3 theta4 theta5 real

% Longitudes (cm)
L1 = 5;
L2 = 10.7;
L3 = 13;
L45 = 18;    % L4+L5


% ===================== Articulacion 1 =====================
alpha1 = pi/2;

R1 = simplify([cos(theta1)  -sin(theta1)*cos(alpha1)   sin(theta1)*sin(alpha1);
               sin(theta1)   cos(theta1)*cos(alpha1)  -cos(theta1)*sin(alpha1);
                   0                sin(alpha1)               cos(alpha1)     ])


% ===================== Articulacion 2 =====================
alpha2 = 0;

R2 = simplify([cos(theta2)  -sin(theta2)*cos(alpha2)   sin(theta2)*sin(alpha2);
               sin(theta2)   cos(theta2)*cos(alpha2)  -cos(theta2)*sin(alpha2);
                   0                sin(alpha2)               cos(alpha2)     ])


% ===================== Articulacion 3 =====================
alpha3 = 0;

R3 = simplify([cos(theta3)  -sin(theta3)*cos(alpha3)   sin(theta3)*sin(alpha3);
               sin(theta3)   cos(theta3)*cos(alpha3)  -cos(theta3)*sin(alpha3);
                   0                sin(alpha3)               cos(alpha3)     ])


% ===================== Articulacion 4 (con el desfase +pi/2 de la tabla DH) =====================
alpha4 = pi/2;

R4 = simplify([cos(theta4+pi/2)  -sin(theta4+pi/2)*cos(alpha4)   sin(theta4+pi/2)*sin(alpha4);
               sin(theta4+pi/2)   cos(theta4+pi/2)*cos(alpha4)  -cos(theta4+pi/2)*sin(alpha4);
                      0                    sin(alpha4)                   cos(alpha4)          ])


% ===================== Articulacion 5 -- muñeca (rota sobre Z, no sobre X) =====================
R5 = [cos(theta5)  -sin(theta5)   0;
      sin(theta5)   cos(theta5)   0;
          0              0        1]


% ===================== Multiplicacion en cadena, UNA matriz a la vez =====================
R02 = simplify(R1 * R2)

R03 = simplify(R02 * R3)

R04 = simplify(R03 * R4)

R05 = simplify(R04 * R5)


% ===================== Yaw, Pitch, Roll -- paso a paso (formulas del profesor) =====================
R11 = R05(1,1);
R21 = R05(2,1);
R31 = R05(3,1);
R32 = R05(3,2);
R33 = R05(3,3);

yaw   = simplify( atan2(R21, R11) )

pitch = simplify( atan2(-R31, sqrt(R11^2 + R21^2)) )

roll  = simplify( atan2(R32, R33) )


% ===================== Posicion (atajo geometrico) =====================
phi2   = theta2;
phi23  = theta2 + theta3;
phi234 = theta2 + theta3 + theta4;

r = L2*cos(phi2) + L3*cos(phi23) + L45*cos(phi234);
z = L1 + L2*sin(phi2) + L3*sin(phi23) + L45*sin(phi234);
x = simplify(r*cos(theta1));
y = simplify(r*sin(theta1));

pos = [x y z]
