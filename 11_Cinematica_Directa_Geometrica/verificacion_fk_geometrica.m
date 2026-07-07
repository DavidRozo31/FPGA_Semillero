% =========================================================================
% Verificacion Cinematica Directa - Metodo Geometrico
% Brazo robotico 5 GDL + gripper - Proyecto FK_6R_Geometrico
% Universidad Militar Nueva Granada
%
% Calcula la posicion (x,y,z) y orientacion (yaw,pitch,roll) del efector
% final, con las mismas formulas que estan programadas en VHDL:
%   - Posicion:    fk_geom_core.vhd
%   - Orientacion: mat_r05_elems.vhd + formulas de angulos de Euler ZYX
% =========================================================================

clear; clc;

%% ---- Longitudes del brazo (metros) ----
L1 = 0.05;
L2 = 0.107;
L3 = 0.13;
L4 = 0.07;
L5 = 0.11;
L45 = L4 + L5;   % L4 y L5 se suman porque theta5 (roll del gripper) no cambia la posicion

%% ==================================================================
%  ANGULOS DE ENTRADA (cambiar aqui, en grados, y volver a correr)
%  ==================================================================
theta1 = 0;     % rotacion de la base
theta2 = 45;    % articulacion 2
theta3 = 0;     % articulacion 3
theta4 = 0;     % articulacion 4
theta5 = 0;     % roll del gripper

%% ---- Conversion a radianes ----
th1 = deg2rad(theta1);
th2 = deg2rad(theta2);
th3 = deg2rad(theta3);
th4 = deg2rad(theta4);
th5 = deg2rad(theta5);

%% ---- Angulos acumulados (igual que angle_sum_gen.vhd) ----
% Como los eslabones 2,3,4 son coplanares, cada uno hereda la
% inclinacion de los anteriores.
phi2   = th2;
phi23  = th2 + th3;
phi234 = th2 + th3 + th4;

%% ---- Posicion del efector (igual que fk_geom_core.vhd) ----
r = L2*cos(phi2) + L3*cos(phi23) + L45*cos(phi234);
z = L1 + L2*sin(phi2) + L3*sin(phi23) + L45*sin(phi234);
x = r*cos(th1);
y = r*sin(th1);

%% ---- Elementos de la matriz de rotacion R0_5 (igual que mat_r05_elems.vhd) ----
R11 = -cos(th1)*sin(phi234);
R21 = -sin(th1)*sin(phi234);
R31 =  cos(phi234);
R32 =  sin(phi234)*sin(th5);
R33 =  sin(phi234)*cos(th5);

%% ---- Angulos de Euler ZYX (formulas de la clase del profesor) ----
yaw   = atan2(R21, R11);
pitch = atan2(-R31, sqrt(R11^2 + R21^2));
roll  = atan2(R32, R33);

%% ---- Resultados ----
fprintf('Angulos de entrada (grados): theta1=%g theta2=%g theta3=%g theta4=%g theta5=%g\n\n', ...
    theta1, theta2, theta3, theta4, theta5);

fprintf('Posicion del efector final:\n');
fprintf('  x = %.4f m\n', x);
fprintf('  y = %.4f m\n', y);
fprintf('  z = %.4f m\n', z);

fprintf('\nOrientacion del efector final:\n');
fprintf('  yaw   = %.2f grados\n', rad2deg(yaw));
fprintf('  pitch = %.2f grados\n', rad2deg(pitch));
fprintf('  roll  = %.2f grados\n', rad2deg(roll));

% Aviso de singularidad: si el brazo queda extendido o plegado en linea
% recta (phi234 = 0 o 180 grados), yaw y roll quedan indefinidos.
if abs(sin(phi234)) < 0.05
    fprintf('\nAviso: phi234 = theta2+theta3+theta4 = %.1f grados.\n', rad2deg(phi234));
    fprintf('El brazo esta extendido/plegado en linea recta (singularidad):\n');
    fprintf('yaw y roll no estan bien definidos aqui.\n');
end
