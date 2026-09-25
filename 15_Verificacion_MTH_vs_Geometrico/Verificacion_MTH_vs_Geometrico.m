% =========================================================================
% Verificacion cruzada 6R: MATLAB (MTH) vs FPGA (ModelSim)
% Formato numerico: Q3.20, valor_real = valor_entero / 2^20
%
% Para cada caso se muestran por separado:
% X, Y, Z, Roll, Pitch y Yaw.
% =========================================================================
clear; clc;

Q = 2^20;

% Longitudes del robot en metros.
L1 = 0.065;
L2 = 0.107;
d4 = 0.140;
d6 = 0.173;

% Entradas de tb.vhd en Q3.20 radianes.
% Columnas: theta1, theta2, theta3, theta4, theta5, theta6.
entrada_q320 = [
        0,       0,        0,        0,       0,        0;
        0,       0,        0,        0, 1647099,  1647099;
   549033,  366022,  -274517,   823550, 1098066, -1281077;
  2518232,  437396,  3078246,  1363432, 2000311,   763156;
  1647099, 1647099,  1647099,  1647099, 1647099,  1647099;
        0,       0, -1647099, -1647099, 1647099,        0
];

% Resultados obtenidos en ModelSim, todos en Q3.20.
% Columnas: X, Y, Z, Yaw, Pitch, Roll.
fpga_q320 = [
   440405,       1,   68159, -1647096,        4, -1647096;
   259000,  181405,   68158,        0,  1647084,  3294199;
   232277,  262378,  237898,  -777726,  -632504,  -685696;
   -69434,    1441,  -65907, -1622664,   217994,  2689033;
        0, -146803,   -1049,  3294199,       -4, -3294191;
   -69208,      -1,  -78645,        0,  1647088,  3294199
];

nombres = {'A','B','C','D','E','F'};
descripcion = {
    'theta = [0, 0, 0, 0, 0, 0] grados';
    'theta = [0, 0, 0, 0, 90, 90] grados';
    'theta = [30, 20, -15, 45, 60, -70] grados';
    'theta = [137.6, 23.9, 168.2, 74.5, 109.3, 41.7] grados';
    'theta = [90, 90, 90, 90, 90, 90] grados';
    'theta = [0, 0, -90, -90, 90, 0] grados'
};

matlab_q320 = zeros(6,6);
matlab_real = zeros(6,6);

for caso = 1:6
    theta = double(entrada_q320(caso,:))/Q;
    t1=theta(1); t2=theta(2); t3=theta(3);
    t4=theta(4); t5=theta(5); t6=theta(6);

    % Matrices DH del robot 6R.
    T01 = [cos(t1) 0 sin(t1) 0;
           sin(t1) 0 -cos(t1) 0;
           0 1 0 L1;
           0 0 0 1];

    T12 = [cos(t2) -sin(t2) 0 L2*cos(t2);
           sin(t2)  cos(t2) 0 L2*sin(t2);
           0 0 1 0;
           0 0 0 1];

    t3_DH = t3 + pi/2;
    T23 = [cos(t3_DH) 0 sin(t3_DH) 0;
           sin(t3_DH) 0 -cos(t3_DH) 0;
           0 1 0 0;
           0 0 0 1];

    t4_DH = t4 + pi/2;
    T34 = [cos(t4_DH) 0 sin(t4_DH) 0;
           sin(t4_DH) 0 -cos(t4_DH) 0;
           0 1 0 d4;
           0 0 0 1];

    T45 = [cos(t5) 0 -sin(t5) 0;
           sin(t5) 0  cos(t5) 0;
           0 -1 0 0;
           0 0 0 1];

    T56 = [cos(t6) -sin(t6) 0 0;
           sin(t6)  cos(t6) 0 0;
           0 0 1 d6;
           0 0 0 1];

    T06 = T01*T12*T23*T34*T45*T56;
    posicion = T06(1:3,4).';
    R06 = T06(1:3,1:3);

    R11=R06(1,1); R21=R06(2,1); R31=R06(3,1);
    R32=R06(3,2); R33=R06(3,3);
    rho = sqrt(R11^2 + R21^2);

    pitch = atan2(-R31,rho);
    if rho < 50/Q
        yaw = 0;
        roll = pi;
    else
        yaw = atan2(R21,R11);
        roll = atan2(R32,R33);
    end

    % Orden comun de almacenamiento: X, Y, Z, Yaw, Pitch, Roll.
    matlab_real(caso,:) = [posicion yaw pitch roll];
    matlab_q320(caso,:) = round(matlab_real(caso,:)*Q);
end

fprintf('\nVERIFICACION CRUZADA 6R: MATLAB VS FPGA - FORMATO Q3.20\n');
fprintf('Valor real = entero Q3.20 / 1048576\n');

% Orden solicitado para presentar los resultados.
orden = [1 2 3 6 5 4];
variables = {'X','Y','Z','Roll','Pitch','Yaw'};

for caso = 1:6
    fprintf('\n===============================================================\n');
    fprintf('CASO %s - %s\n',nombres{caso},descripcion{caso});
    fprintf('===============================================================\n');
    fprintf('%-7s | %12s | %12s | %14s | %14s | %9s\n', ...
        'Valor','MATLAB Q3.20','FPGA Q3.20','MATLAB real','FPGA real','Error LSB');
    fprintf('%s\n',repmat('-',1,86));

    for fila = 1:6
        columna = orden(fila);
        valor_matlab = matlab_q320(caso,columna);
        valor_fpga = fpga_q320(caso,columna);
        error_lsb = valor_fpga-valor_matlab;

        if columna <= 3
            matlab_texto = sprintf('%.6f m',matlab_real(caso,columna));
            fpga_texto = sprintf('%.6f m',fpga_q320(caso,columna)/Q);
        else
            matlab_texto = sprintf('%.6f deg',rad2deg(matlab_real(caso,columna)));
            fpga_texto = sprintf('%.6f deg',rad2deg(fpga_q320(caso,columna)/Q));
        end

        fprintf('%-7s | %12d | %12d | %14s | %14s | %+9d\n', ...
            variables{fila},valor_matlab,valor_fpga, ...
            matlab_texto,fpga_texto,error_lsb);
    end
end

fprintf('\nNota: los casos B y F tienen pitch cercano a 90 grados.\n');
fprintf('En esa singularidad el VHDL fija Yaw=0 y Roll=180 grados.\n');
