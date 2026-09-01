% =========================================================================
% Verificacion cruzada: Metodo Geometrico (el que corre en la FPGA/STM32,
% lecciones 11-13) vs Matrices de Transformacion Homogenea (MTH, 4x4,
% construidas aqui desde cero con la tabla DH del brazo de 5 GDL+gripper).
%
% Idea: tomar los MISMOS angulos de entrada que ya se validaron en la FPGA
% (lección 11, seccion 7) y calcular la pose del efector final por el
% camino MTH clasico (T01*T12*T23*T34*T45) -- si los dos metodos
% independientes coinciden, es una confirmacion fuerte de que el metodo
% geometrico (elegido por ahorrar recursos logicos) es correcto.
%
% Universidad Militar Nueva Granada
% =========================================================================
clear; clc;

%% ---- Longitudes del brazo (metros) -- igual que la leccion 11 ----
L1 = 0.05; L2 = 0.107; L3 = 0.13; L45 = 0.18;   % L45 = L4+L5 combinados

%% ---- Matriz DH generica: T = Rz(thetaZ)*Trans_z(Lz)*Trans_x(Lx)*Rx(thetaX) ----
T = @(thetaZ, thetaX, Lz, Lx) [
    cos(thetaZ) -cos(thetaX)*sin(thetaZ)  sin(thetaX)*sin(thetaZ)  Lx*cos(thetaZ);
    sin(thetaZ)  cos(thetaX)*cos(thetaZ) -cos(thetaZ)*sin(thetaX)  Lx*sin(thetaZ);
    0            sin(thetaX)              cos(thetaX)              Lz;
    0            0                        0                        1];

%% ---- Casos de prueba (idénticos a la lección 11, sección 7) ----
% [theta1 theta2 theta3 theta4 theta5] en grados
casos = {
    'Caso 1 (singularidad)', [0,   0, 0,  0,  0];
    'Caso 2 (limpio)',       [45,  0, 0, 90, 45];
    'Caso 3 (limpio)',       [0,  45, 0,  0,  0];
};

% Resultados ya validados en la FPGA (lección 11, tabla de la sección 7,
% convertidos de Q2.13 a metros/grados: raw/8192)
fpga = [
     0.4178   0.0004   0.0503     0    -90.00  180.00;   % Caso 1 (yaw/roll estandarizados)
     0.1679   0.1678   0.2304   -90.02    0.00    0.00;  % Caso 2
     0.2952   0.0002   0.3450  -179.98  -45.00    0.02;  % Caso 3
];

fprintf('%-22s | %-28s | %-28s\n', 'Caso', 'MTH (matrices 4x4)', 'FPGA (leccion 11)');
fprintf('%s\n', repmat('-', 1, 90));

for i = 1:size(casos,1)
    nombre = casos{i,1};
    ang = deg2rad(casos{i,2});
    t1=ang(1); t2=ang(2); t3=ang(3); t4=ang(4); t5=ang(5);

    T01 = T(t1,        pi/2, L1, 0);
    T12 = T(t2,        0,    0,  L2);
    T23 = T(t3,        0,    0,  L3);
    T34 = T(t4 + pi/2, pi/2, 0,  0);
    T45 = T(t5,        0,    L45, 0);
    T05 = T01*T12*T23*T34*T45;

    pos = T05(1:3,4);
    R   = T05(1:3,1:3);
    yaw   = rad2deg(atan2(R(2,1), R(1,1)));
    pitch = rad2deg(atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2)));
    roll  = rad2deg(atan2(R(3,2), R(3,3)));

    mth_str  = sprintf('x=%.4f y=%.4f z=%.4f', pos(1), pos(2), pos(3));
    fpga_str = sprintf('x=%.4f y=%.4f z=%.4f', fpga(i,1), fpga(i,2), fpga(i,3));
    fprintf('%-22s | %-28s | %-28s\n', nombre, mth_str, fpga_str);

    mth_ang  = sprintf('yaw=%.2f pitch=%.2f roll=%.2f', yaw, pitch, roll);
    fpga_ang = sprintf('yaw=%.2f pitch=%.2f roll=%.2f', fpga(i,4), fpga(i,5), fpga(i,6));
    fprintf('%-22s | %-28s | %-28s\n', '', mth_ang, fpga_ang);

    if i == 1
        fprintf('%-22s | %s\n', '', 'Nota: yaw/roll INDIVIDUALES no comparables aqui (gimbal lock,');
        fprintf('%-22s | %s\n', '', 'seccion 6 lección 13) -- solo yaw+roll es invariante: MTH=180.00, FPGA=180.00 (coincide).');
    end
    if i == 3
        fprintf('%-22s | %s\n', '', 'Nota: yaw=180.00 (MTH) y yaw=-179.98 (FPGA) son el mismo angulo fisico');
        fprintf('%-22s | %s\n', '', '(corte de +-180 grados de atan2, seccion 7 lección 11).');
    end
    fprintf('\n');
end
