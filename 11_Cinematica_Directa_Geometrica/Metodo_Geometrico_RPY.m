% Metodo geometrico -- construccion matriz por matriz (Paso 4 y 5 en papel,
% igual estilo que el codigo del metodo DH)
%
% Para cambiar de caso: solo hay que editar theta1 (linea 24),
% theta2 (linea 33), theta3 (linea 42), theta4 (linea 51) y
% theta5 (linea 60) con los valores de la tabla. alpha1..alpha4 NO
% se tocan (son fijos, vienen de la geometria del robot).
%
%              theta1   theta2   theta3   theta4   theta5
% Caso 1         0        0        0        0        0     (singularidad)
% Caso 2       pi/4       0        0      pi/2     pi/4     (limpio)
% Caso 3         0      pi/4       0        0        0      (limpio) <- este esta activo ahora
%
% Compara pos/yaw/pitch/roll contra la fila correspondiente de tb.vhd.

% Longitudes (cm)
L1 = 5;
L2 = 10.7;
L3 = 13;
L45 = 18;    % L4+L5


% ===================== Articulacion 1 =====================
theta1 = 0;
alpha1 = pi/2;

R1 = [cos(theta1)  -sin(theta1)*cos(alpha1)   sin(theta1)*sin(alpha1);
      sin(theta1)   cos(theta1)*cos(alpha1)  -cos(theta1)*sin(alpha1);
          0                sin(alpha1)               cos(alpha1)     ]


% ===================== Articulacion 2 =====================
theta2 = pi/4;
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
theta4 = 0;
alpha4 = pi/2;

R4 = [cos(theta4+pi/2)  -sin(theta4+pi/2)*cos(alpha4)   sin(theta4+pi/2)*sin(alpha4);
      sin(theta4+pi/2)   cos(theta4+pi/2)*cos(alpha4)  -cos(theta4+pi/2)*sin(alpha4);
             0                    sin(alpha4)                   cos(alpha4)          ]


% ===================== Articulacion 5 -- muñeca (rota sobre Z, no sobre X) =====================
theta5 = 0;

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

% rho = sqrt(R11^2+R21^2) es tambien la magnitud de (R33,R32) en la
% singularidad (las dos se anulan juntas -- ver seccion 3.6/3.7 del
% README de la leccion 11). Sirve para DETECTAR la singularidad antes
% de calcular yaw/roll.
rho = sqrt(R11^2 + R21^2);

RHO_THRESH = 1e-4;   % umbral de "rho ~ 0" -- ajustable

if rho < RHO_THRESH
    % ESTANDARIZACION (acordada con el profesor): en pitch=+-90 grados,
    % yaw y roll individuales no estan matematicamente definidos (solo
    % yaw+roll lo esta), y cada plataforma (FPGA, MATLAB, STM32) los
    % repartiria distinto segun su propio ruido de redondeo. Para que
    % TODAS den el mismo numero aqui, se fija por convencion:
    yaw  = 0;
    roll = 180;
    fprintf('[SINGULARIDAD detectada: rho=%.2e < %.2e] yaw y roll fijados por convencion.\n', rho, RHO_THRESH);
else
    yaw  = rad2deg( atan2(R21, R11) );
    roll = rad2deg( atan2(R32, R33) );
end

pitch = rad2deg( atan2(-R31, rho) )   % pitch SIEMPRE es confiable, nunca se estandariza
yaw
roll

% chequeo contra la funcion del toolbox (da el reparto "natural" del
% toolbox, sin estandarizar -- va a diferir de yaw/roll en la singularidad,
% eso es esperado, ver seccion 3.7 del README)
r_rpy_check = rad2deg(tr2rpy(R05, 'zyx'))


% ===================== Verificacion del profesor: reconstruir R05 =====================
% Con yaw,pitch,roll ya extraidos, arma Rz(yaw)*Ry(pitch)*Rx(roll) y
% compara contra R05. Fuera de la singularidad el error debe dar ~0.
% DENTRO de la singularidad, como yaw/roll ahora estan ESTANDARIZADOS
% (fijos en 0/180 en vez del reparto que reconstruye R05 exacto), el
% error puede NO dar ~0 -- es el costo esperado de estandarizar: se
% gana consistencia entre plataformas, se pierde la reconstruccion
% exacta de esa R05 puntual (que de todas formas era una entre
% infinitas posibles ahi).
yr = deg2rad(yaw);
pr = deg2rad(pitch);
rr = deg2rad(roll);

Rz_yaw   = [cos(yr) -sin(yr) 0; sin(yr) cos(yr) 0; 0 0 1];
Ry_pitch = [cos(pr) 0 sin(pr); 0 1 0; -sin(pr) 0 cos(pr)];
Rx_roll  = [1 0 0; 0 cos(rr) -sin(rr); 0 sin(rr) cos(rr)];

R_reconstruida = Rz_yaw * Ry_pitch * Rx_roll

% Tabla lado a lado: R05 (columnas 1-3) | separador de NaN | R_reconstruida (columnas 5-7)
disp('Comparacion  [ R05  |  R_reconstruida ]:')
comparacion = [R05, NaN(3,1), R_reconstruida]

% Diferencia elemento por elemento (debe salir ~0 en las 9 casillas)
diferencia = R_reconstruida - R05

error_reconstruccion = norm(diferencia)


% ===================== Posicion (atajo geometrico) =====================
phi2   = theta2;
phi23  = theta2 + theta3;
phi234 = theta2 + theta3 + theta4;

r = L2*cos(phi2) + L3*cos(phi23) + L45*cos(phi234);
z = L1 + L2*sin(phi2) + L3*sin(phi23) + L45*sin(phi234);
x = r*cos(theta1);
y = r*sin(theta1);

pos = [x y z]
