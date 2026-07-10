% Metodo geometrico -- construccion matriz por matriz (Paso 4 y 5 en papel,
% igual estilo que el codigo del metodo DH)

% Longitudes (cm)
L1 = 5;
L2 = 10.7;
L3 = 13;
L45 = 18;    % L4+L5


% ===================== Articulacion 1 =====================
theta1 = pi/4;
alpha1 = pi/2;

R1 = [cos(theta1)  -sin(theta1)*cos(alpha1)   sin(theta1)*sin(alpha1);
      sin(theta1)   cos(theta1)*cos(alpha1)  -cos(theta1)*sin(alpha1);
          0                sin(alpha1)               cos(alpha1)     ]


% ===================== Articulacion 2 =====================
theta2 = 0;
alpha2 = 0;

R2 = [cos(theta2)  -sin(theta2)*cos(alpha2)   sin(theta2)*sin(alpha2);
      sin(theta2)   cos(theta2)*cos(alpha2)  -cos(theta2)*sin(alpha2);
          0                sin(alpha2)               cos(alpha2)     ]


% ===================== Articulacion 3 =====================
theta3 = 0;
alpha3 = 0;

R3 = [cos(theta3)  -sin(theta3)*cos(alpha3)   sin(theta3)*sin(alpha3);
      sin(theta3)   cos(theta3)*cos(alpha3)  -cos(theta3)*sin(alpha3);
          0                sin(alpha3)               cos(alpha3)     ]


% ===================== Articulacion 4 (con el desfase +pi/2 de la tabla DH) =====================
theta4 = pi/2;
alpha4 = pi/2;

R4 = [cos(theta4+pi/2)  -sin(theta4+pi/2)*cos(alpha4)   sin(theta4+pi/2)*sin(alpha4);
      sin(theta4+pi/2)   cos(theta4+pi/2)*cos(alpha4)  -cos(theta4+pi/2)*sin(alpha4);
             0                    sin(alpha4)                   cos(alpha4)          ]


% ===================== Articulacion 5 -- muñeca (rota sobre Z, no sobre X) =====================
theta5 = pi/4;

R5 = [cos(theta5)  -sin(theta5)   0;
      sin(theta5)   cos(theta5)   0;
          0              0        1]


% ===================== Multiplicacion en cadena, UNA matriz a la vez =====================
R02 = R1 * R2

R03 = R02 * R3

R04 = R03 * R4

R05 = R04 * R5


% ===================== Yaw, Pitch, Roll -- paso a paso (formulas del profesor) =====================
R11 = R05(1,1);
R21 = R05(2,1);
R31 = R05(3,1);
R32 = R05(3,2);
R33 = R05(3,3);

yaw   = rad2deg( atan2(R21, R11) )

pitch = rad2deg( atan2(-R31, sqrt(R11^2 + R21^2)) )

roll  = rad2deg( atan2(R32, R33) )

% chequeo contra la funcion del toolbox (debe dar lo mismo)
r_rpy_check = rad2deg(tr2rpy(R05, 'zyx'))


% ===================== Posicion (atajo geometrico) =====================
phi2   = theta2;
phi23  = theta2 + theta3;
phi234 = theta2 + theta3 + theta4;

r = L2*cos(phi2) + L3*cos(phi23) + L45*cos(phi234);
z = L1 + L2*sin(phi2) + L3*sin(phi23) + L45*sin(phi234);
x = r*cos(theta1);
y = r*sin(theta1);

pos = [x y z]
