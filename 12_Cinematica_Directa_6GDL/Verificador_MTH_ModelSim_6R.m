% Verificador_MTH_ModelSim_6R.m
%
% Reconstruye la MTH (matriz de transformacion homogenea 4x4) a partir de
% los valores RAW en Q2.13 que bota ModelSim (x_out,y_out,z_out,yaw_out,
% pitch_out,roll_out), y la compara contra la MTH "ideal" calculada con los
% mismos angulos de entrada pero SIN el redondeo del CORDIC -- para ver
% cuanto se aleja el resultado real de la FPGA del resultado matematico
% exacto.
%
% Como usar:
%   1. Corre el testbench en ModelSim para un caso.
%   2. Copia los 6 numeros que imprime (x,y,z,yaw,pitch,roll, todos en
%      crudo, tal cual salen -- son enteros con signo) en la SECCION 1.
%   3. Copia los theta1..theta6 (en grados) que usaste para ESE caso en
%      la SECCION 2.
%   4. Corre el script. Compara MTH_modelsim contra MTH_ideal, y mira
%      "error_norma" al final.

clear; clc;

%% ===== 1. Valores RAW que bota ModelSim (Q2.13) -- EDITA ESTO =====
x_raw     = 3441;
y_raw     = 0;
z_raw     = 532;
yaw_raw   = -12868;
pitch_raw = 0;
roll_raw  = -12868;

%% ===== 2. Angulos de entrada usados en ese caso (grados) -- EDITA ESTO =====
theta1_deg = 0;
theta2_deg = 0;
theta3_deg = 0;
theta4_deg = 0;
theta5_deg = 0;
theta6_deg = 0;

%% ===== 3. Q2.13 -> unidades reales =====
Q13 = 8192;   % raw = round(valor_real * 8192)

x = x_raw/Q13;   y = y_raw/Q13;   z = z_raw/Q13;         % metros
yaw   = rad2deg(yaw_raw/Q13);
pitch = rad2deg(pitch_raw/Q13);
roll  = rad2deg(roll_raw/Q13);

fprintf('Del ModelSim (con redondeo de CORDIC):\n');
fprintf('  x=%.4f  y=%.4f  z=%.4f  m\n', x, y, z);
fprintf('  yaw=%.2f  pitch=%.2f  roll=%.2f  grados\n\n', yaw, pitch, roll);

%% ===== 4. MTH reconstruida con los valores de ModelSim =====
yr = deg2rad(yaw); pr = deg2rad(pitch); rr = deg2rad(roll);

Rz_yaw   = [cos(yr) -sin(yr) 0; sin(yr) cos(yr) 0; 0 0 1];
Ry_pitch = [cos(pr) 0 sin(pr); 0 1 0; -sin(pr) 0 cos(pr)];
Rx_roll  = [1 0 0; 0 cos(rr) -sin(rr); 0 sin(rr) cos(rr)];
R_modelsim = Rz_yaw * Ry_pitch * Rx_roll;

MTH_modelsim = [R_modelsim, [x;y;z]; 0 0 0 1]

%% ===== 5. MTH "ideal" -- mismos angulos de entrada, sin CORDIC =====
% mismas longitudes y formulas que Metodo_Geometrico_RPY_6R.m
L1=0.065; L2=0.107; L3=0.095; L4=0.045; L5=0.07; L6=0.103;
Ld4 = L3+L4; Ld6 = L5+L6;

t1=deg2rad(theta1_deg); t2=deg2rad(theta2_deg); t3=deg2rad(theta3_deg);
t4=deg2rad(theta4_deg); t5=deg2rad(theta5_deg); t6=deg2rad(theta6_deg);

dh = @(th,al) [cos(th) -sin(th)*cos(al)  sin(th)*sin(al);
               sin(th)  cos(th)*cos(al) -cos(th)*sin(al);
               0        sin(al)          cos(al)        ];

R1 = dh(t1,        pi/2);
R2 = dh(t2,        0);
R3 = dh(t3+pi/2,   pi/2);
R4 = dh(t4+pi/2,   pi/2);
R5 = dh(t5,       -pi/2);
R6 = dh(t6,        0);

R02=R1*R2; R03=R02*R3; R04=R03*R4; R05=R04*R5; R06=R05*R6;

% Posicion: metodo recursivo (igual que Metodo_Geometrico_RPY_6R.m)
O0 = [0;0;0];
O1 = O0 + eye(3)*[0;0;L1];
O2 = O1 + R1  *[L2*cos(t2); L2*sin(t2); 0];
O3 = O2 + R02 *[0;0;0];
O4 = O3 + R03 *[0;0;Ld4];
O5 = O4 + R04 *[0;0;0];
O6 = O5 + R05 *[0;0;Ld6];

MTH_ideal = [R06, O6; 0 0 0 1]

%% ===== 6. Comparacion =====
diferencia = MTH_modelsim - MTH_ideal
error_norma = norm(diferencia);

fprintf('\nError total (norma de la diferencia): %.6f\n', error_norma);
fprintf('Margen normal esperado del CORDIC de 12 iteraciones: ~0.02-0.1 grados de angulo,\n');
fprintf('unas pocas unidades Q2.13 de posicion -- si el error sale mucho mayor que eso,\n');
fprintf('probablemente copiaste mal un valor o los theta1..6 no son los del caso correcto.\n');
