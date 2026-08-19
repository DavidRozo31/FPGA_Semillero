% Metodo geometrico -- construccion matriz por matriz, brazo de 6 GDL real
% (misma metodologia y mismo estilo que Metodo_Geometrico_RPY.m, que era
% para el brazo viejo de 5 articulaciones + roll de gripper)
%
% Tabla DH nueva (corregida por el profesor -- sistemas 3 y 5 devueltos
% a los sistemas anteriores, "regla 4"):
%
%   i | theta_i     | d_i   | alpha_i | a_i
%   1 | theta1      | L1    | pi/2    | 0
%   2 | theta2      | 0     | 0       | L2
%   3 | theta3+pi/2 | 0     | pi/2    | 0
%   4 | theta4+pi/2 | L3+L4 | pi/2    | 0
%   5 | theta5      | 0     | -pi/2   | 0
%   6 | theta6      | L5+L6 | 0       | 0
%
% Ya validada en Peter Corke (Metodo_DH_Peter_6R.m): en q=0 el brazo
% queda extendido, x=42cm y=0 z=6.5cm, yaw=90 pitch=0 roll=90.
%
% Para cambiar de caso: editar theta1..theta6 mas abajo y volver a correr.
%
% ===================================================================
%  CASOS YA PROBADOS EN MODELSIM (VHDL, Paso 7 -- integracion final)
% ===================================================================
%
%  Caso A: theta1=0  theta2=0  theta3=0  theta4=0  theta5=0  theta6=0
%    esperado: pos=[0.4200 0 0.0650] | yaw=-90 pitch=0 roll=-90
%
%  Caso B: theta1=0  theta2=0  theta3=0  theta4=0  theta5=deg2rad(90)  theta6=deg2rad(90)
%    esperado: pos=[0.2470 0.1730 0.0650] | SINGULARIDAD -> yaw=0 roll=180 pitch=90
%
%  Caso C: theta1=deg2rad(30)  theta2=deg2rad(20)  theta3=deg2rad(-15)
%          theta4=deg2rad(45)  theta5=deg2rad(60)  theta6=deg2rad(-70)
%    esperado: pos~=[0.2215 0.2502 0.2269] | yaw~-42.50 pitch~-34.56 roll~-37.47
% ===================================================================

% Longitudes (cm)
L1 = 6.5;
L2 = 10.7;
L3 = 9.5;
L4 = 4.5;
L5 = 7;
L6 = 10.3;
Ld4 = L3 + L4;   % la fila 4 absorbe la distancia que el sistema 3 ya no carga (a3=0)
Ld6 = L5 + L6;   % la fila 6 absorbe la distancia que el sistema 5 ya no carga (a5=0)


% ===================== Articulacion 1 =====================
theta1 = 0;
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


% ===================== Articulacion 3 (sistema 3 = sistema 2, con el desfase +pi/2 de la tabla) =====================
theta3 = 0;
alpha3 = pi/2;

R3 = [cos(theta3+pi/2)  -sin(theta3+pi/2)*cos(alpha3)   sin(theta3+pi/2)*sin(alpha3);
      sin(theta3+pi/2)   cos(theta3+pi/2)*cos(alpha3)  -cos(theta3+pi/2)*sin(alpha3);
             0                    sin(alpha3)                   cos(alpha3)          ]


% ===================== Articulacion 4 (con el desfase +pi/2 de la tabla DH) =====================
theta4 = 0;
alpha4 = pi/2;

R4 = [cos(theta4+pi/2)  -sin(theta4+pi/2)*cos(alpha4)   sin(theta4+pi/2)*sin(alpha4);
      sin(theta4+pi/2)   cos(theta4+pi/2)*cos(alpha4)  -cos(theta4+pi/2)*sin(alpha4);
             0                    sin(alpha4)                   cos(alpha4)          ]


% ===================== Articulacion 5 (sistema 5 = sistema 4, alpha negativo) =====================
theta5 = 0;
alpha5 = -pi/2;

R5 = [cos(theta5)  -sin(theta5)*cos(alpha5)   sin(theta5)*sin(alpha5);
      sin(theta5)   cos(theta5)*cos(alpha5)  -cos(theta5)*sin(alpha5);
          0                sin(alpha5)               cos(alpha5)     ]


% ===================== Articulacion 6 -- roll final del efector (alpha6=0, Rz puro) =====================
theta6 = 0;

R6 = [cos(theta6)  -sin(theta6)   0;
      sin(theta6)   cos(theta6)   0;
          0              0        1]


% ===================== Multiplicacion en cadena, UNA matriz a la vez =====================
R02 = R1 * R2

R03 = R02 * R3

R04 = R03 * R4

R05 = R04 * R5

R06 = R05 * R6


% ===================== Yaw, Pitch, Roll -- paso a paso (formulas del profesor) =====================
R11 = R06(1,1);
R21 = R06(2,1);
R31 = R06(3,1);
R32 = R06(3,2);
R33 = R06(3,3);

% R13,R23 hacen falta para la posicion (ver seccion de abajo) -- no se
% usan para yaw/pitch/roll, son columna 3 de R0_6 (direccion de Z6).
R13 = R06(1,3);
R23 = R06(2,3);

% rho = sqrt(R11^2+R21^2) es tambien la magnitud de (R33,R32) en la
% singularidad -- mismo criterio de deteccion que el brazo de 5R.
rho = sqrt(R11^2 + R21^2);

RHO_THRESH = 1e-4;   % umbral de "rho ~ 0" -- ajustable

if rho < RHO_THRESH
    % ESTANDARIZACION (misma convencion acordada con el profesor para el
    % brazo de 5R, se mantiene igual aqui): en pitch=+-90 grados, yaw y
    % roll individuales no estan definidos, solo yaw+roll lo esta. Para
    % que TODAS las plataformas (FPGA, MATLAB, STM32) den el mismo numero:
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
% toolbox, sin estandarizar -- puede diferir de yaw/roll en la singularidad)
r_rpy_check = rad2deg(tr2rpy(R06, 'zyx'))


% ===================== Verificacion del profesor: reconstruir R06 =====================
% Con yaw,pitch,roll ya extraidos, arma Rz(yaw)*Ry(pitch)*Rx(roll) y
% compara contra R06. Fuera de la singularidad el error debe dar ~0.
yr = deg2rad(yaw);
pr = deg2rad(pitch);
rr = deg2rad(roll);

Rz_yaw   = [cos(yr) -sin(yr) 0; sin(yr) cos(yr) 0; 0 0 1];
Ry_pitch = [cos(pr) 0 sin(pr); 0 1 0; -sin(pr) 0 cos(pr)];
Rx_roll  = [1 0 0; 0 cos(rr) -sin(rr); 0 sin(rr) cos(rr)];

R_reconstruida = Rz_yaw * Ry_pitch * Rx_roll

disp('Comparacion  [ R06  |  R_reconstruida ]:')
comparacion = [R06, NaN(3,1), R_reconstruida]

diferencia = R_reconstruida - R06

error_reconstruccion = norm(diferencia)


% ===================== Posicion (metodo geometrico RECURSIVO, del profesor) =====================
% Igual idea que el ejemplo del profesor (Inversa2R.pdf): la posicion se
% acumula eslabon por eslabon, NO con un atajo trigonometrico global.
%
% Para cada eslabon i:
%   p_i = [ a_i*cos(theta_i') , a_i*sin(theta_i') , d_i ]   <- desplazamiento
%         LOCAL de ese eslabon (sale directo de su fila en la tabla DH,
%         theta_i' ya incluye el desfase fijo de esa fila si tiene)
%   O_i = O_(i-1) + R_(i-1)^0 * p_i   <- se rota ese desplazamiento local
%         al "idioma" de la base (con la rotacion acumulada HASTA el
%         eslabon anterior, que ya se calculo arriba: R1, R02, R03, R04, R05)
%         y se suma a donde ya estaba el eslabon anterior
%
% O0 = [0;0;0], R_0^0 = identidad (la base no esta rotada respecto a si misma)

O0 = [0;0;0];

% --- eslabon 1: a1=0, d1=L1, theta1' = theta1 ---
p1 = [0; 0; L1];
O1 = O0 + eye(3)*p1

% --- eslabon 2: a2=L2, d2=0, theta2' = theta2 ---
p2 = [L2*cos(theta2); L2*sin(theta2); 0];
O2 = O1 + R1*p2

% --- eslabon 3: a3=0, d3=0 (sistema 3 = sistema 2, "regla 4") ---
p3 = [0; 0; 0];
O3 = O2 + R02*p3

% --- eslabon 4: a4=0, d4=Ld4, theta4' = theta4+pi/2 ---
p4 = [0; 0; Ld4];
O4 = O3 + R03*p4

% --- eslabon 5: a5=0, d5=0 (sistema 5 = sistema 4, "regla 4") ---
p5 = [0; 0; 0];
O5 = O4 + R04*p5

% --- eslabon 6: a6=0, d6=Ld6, theta6' = theta6 ---
p6 = [0; 0; Ld6];
O6 = O5 + R05*p6

pos = O6'
