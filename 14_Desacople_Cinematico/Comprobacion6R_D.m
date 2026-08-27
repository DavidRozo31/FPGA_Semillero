%% Script de Verificacion en MATLAB para los Casos del Testbench VHDL
clear all
close all
clc

%% ==================== PARAMETROS DEL ROBOT ====================
l1 = 6;     % cm (convertidos a m en la operacion o mantenidos en la misma unidad que p)
l2 = 10.7;
l3 = 9.5;
l4 = 4.5;
l5 = 7;
l6 = 10.3;

% Nota: Si l1 está en cm (6 cm = 0.06 m), asegúrate de que las posiciones 
% y longitudes guarden la misma unidad. En este script asumiremos cm:
l_total_wrist = l5 + l6; 

%% ==================== DEFINICION DE LOS 4 CASOS DE PRUEBA ====================
% Formato de cada caso: [rz, ry, rx (en grados), px, py, pz]
casos = [
    % Caso 1 - Orientacion identidad
    0.0,   0.0,   0.0,   30.0,   0.0,   45.0;
    % Caso 2 - Rotacion 90 deg en Z
    90.0,  0.0,   0.0,   20.0,  25.0,   35.0;
    % Caso 3 - Rotacion ZYX combinada
    30.0,  45.0,  22.5,  15.0,  10.0,   40.0;
    % Caso 4 - Miremos el ejemplo de Fabian
    -128.2, -11.5, 26.1, 8.790, 18.655, 38.781
];

fprintf('===================================================================\n');
fprintf('         VERIFICACION NUMERICA DE CASOS DE PRUEBA (MATLAB)         \n');
fprintf('===================================================================\n\n');

for i = 1:size(casos, 1)
    fprintf('-------------------- CASO %d --------------------\n', i);
    
    % Extraer angulos Euler (convertir a radianes)
    rz = deg2rad(casos(i, 1));
    ry = deg2rad(casos(i, 2));
    rx = deg2rad(casos(i, 3));
    
    % Extraer posicion deseada del TCP (convertida de m a cm si en el TB usaste 0.3 -> 30cm)
    % Nota: En tu TB usaste 0.3 que equivaldria a 30 cm si la escala es en cm.
    px = casos(i, 4);
    py = casos(i, 5);
    pz = casos(i, 6);
    
    %% 1. Construccion de la Matriz de Rotacion R0_6 (Euler ZYX)
    Rz_mat = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
    Ry_mat = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    Rx_mat = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    
    R = Rz_mat * Ry_mat * Rx_mat;
    
    %% 2. Calculo del Centro de la Muneca (Wrist Center)
    rz_vec = R(1:3, 3);
    PosWrist = [px; py; pz] - l_total_wrist * rz_vec;
    
    Px = PosWrist(1);
    Py = PosWrist(2);
    Pz = PosWrist(3);
    
    %% 3. Cinematica Inversa (Theta 1, 2, 3)
    b = sqrt(Px^2 + Py^2);
    c = Pz - l1;
    e = sqrt(b^2 + c^2);
    
    theta1 = atan2(Py, Px);
    
    cos_theta3 = (e^2 - l2^2 - (l3+l4)^2) / (2 * l2 * (l3+l4));
    cos_theta3 = max(-1, min(1, cos_theta3)); % Satura por seguridad numerica
    sen_theta3 = sqrt(1 - cos_theta3^2);
    theta3 = atan2(sen_theta3, cos_theta3);
    
    alpha_ang = atan2(c, b);
    phi_ang = atan2((l3+l4)*sen_theta3, l2 + (l3+l4)*cos_theta3);
    theta2 = alpha_ang - phi_ang;
    
    if theta2 <= -pi
        theta2 = (2*pi) + theta2;
    end
    
    %% 4. Desacople Numerico (Theta 4, 5, 6)
    % Construccion de R03 con los tres primeros angulos
    R01 = RotarZ_f(theta1) * round(RotarX_f(pi/2));
    R12 = RotarZ_f(theta2);
    R23 = RotarZ_f(theta3) * round(RotarX_f(pi/2) * RotarY_f(pi/2));
    R03 = R01 * R12 * R23;
    
    R03i = R03'; % Transpuesta
    R36A = R03i * R;
    
    theta4 = atan2(R36A(1,3), -R36A(2,3));
    theta6 = atan2(R36A(3,2), -R36A(3,1));
    theta5 = atan2(sqrt(max(0, 1 - R36A(3,3)^2)), R36A(3,3));
    
    %% 5. Mostrar Resultados en Grados
    fprintf('Resultados Esperados:\n');
    fprintf('  Theta 1 = %8.4f deg\n', rad2deg(theta1));
    fprintf('  Theta 2 = %8.4f deg\n', rad2deg(theta2));
    fprintf('  Theta 3 = %8.4f deg\n', rad2deg(theta3));
    fprintf('  Theta 4 = %8.4f deg\n', rad2deg(theta4));
    fprintf('  Theta 5 = %8.4f deg\n', rad2deg(theta5));
    fprintf('  Theta 6 = %8.4f deg\n', rad2deg(theta6));
    fprintf('\n');
end

%% Funciones Locales de Rotacion Auxiliares
function R = RotarX_f(theta)
    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end
function R = RotarY_f(theta)
    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end
function R = RotarZ_f(theta)
    R = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
end