clear all
close all
clc

L1 = 6.5;
L2 = 10.7;
L3 = 9.5;
L4 = 4.5;
L5 = 7;
L6 = 10.3;


q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;

% ============================================================
%  CORRECCION DEL PROFESOR: "los sistemas 3 y 5 se deben devolver
%  a los sistemas anteriores" -- el sistema 3 coincide con el 2
%  (a3=0, no se mueve aparte) y el sistema 5 coincide con el 4
%  (a5=0). La distancia que antes era a3=L3 y a5=L5 no desaparece:
%  se acumula en el d del eslabon SIGUIENTE (d4 = L3+L4, d6 = L5+L6).
%  Sin offsets nuevos, solo se movieron las distancias de columna.
% ============================================================

% ============================================================
%  DEFINICIÓN DE ESLABONES (Peter Corke)
% ============================================================
% Eslabón 1: Revoluto | d=L1,     alpha=pi/2, a=0,  offset=0
% Eslabón 2: Revoluto | d=0,      alpha=0,    a=L2, offset=0
% Eslabón 3: Revoluto | d=0,      alpha=pi/2, a=0,  offset=pi/2
% Eslabón 4: Revoluto | d=L3+L4,  alpha=pi/2, a=0,  offset=pi/2
% Eslabón 5: Revoluto | d=0,      alpha=-pi/2, a=0,  offset=0
% Eslabón 6: Revoluto | d=L5+L6,  alpha=0,    a=0,  offset=0

R(1) = Link('revolute', 'd', L1,     'alpha',  pi/2, 'a', 0, 'offset', 0);
R(2) = Link('revolute', 'd', 0,      'alpha',  0,    'a', L2, 'offset', 0);
R(3) = Link('revolute', 'd', 0,      'alpha',  pi/2, 'a', 0, 'offset', pi/2);
R(4) = Link('revolute', 'd', L3+L4,  'alpha',  pi/2, 'a', 0, 'offset', pi/2);
R(5) = Link('revolute', 'd', 0,      'alpha',  -pi/2, 'a', 0, 'offset', 0);
R(6) = Link('revolute', 'd', L5+L6,  'alpha',  0,    'a', 0, 'offset', 0);


Robot = SerialLink(R, 'name', 'MiBrazo6DOF');
Robot.plot([q1, q2, q3, q4, q5, q6], 'scale', 0.5, 'workspace', [-40 40 -40 40 -40 40]);
zlim([-10, 40]);
Robot.teach([q1, q2, q3, q4, q5, q6], 'rpy/zyx');

% Cinemática directa con Peter Corke
MTH = Robot.fkine([q1, q2, q3, q4, q5, q6])

%% ============================================================
%%  MATRICES DH PASO A PASO
%% ============================================================

% --- Eslabón 1: θ1, d=L1, α=π/2, a=0 ---
TZ0 = [1 0 0  0;
       0 1 0  0;
       0 0 1 L1;
       0 0 0  1];

RZ0 = [cos(q1) -sin(q1) 0 0;
       sin(q1)  cos(q1) 0 0;
       0        0       1 0;
       0        0       0 1];

TX1 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RX1 = [1      0           0       0;
       0  cos(pi/2)  -sin(pi/2)   0;
       0  sin(pi/2)   cos(pi/2)   0;
       0      0           0       1];

T01 = RZ0 * TZ0 * RX1 * TX1

% --- Eslabón 2: θ2, d=0, α=0, a=L2 ---
TZ1 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RZ1 = [cos(q2) -sin(q2) 0 0;
       sin(q2)  cos(q2) 0 0;
       0        0       1 0;
       0        0       0 1];

TX2 = [1 0 0 L2;
       0 1 0  0;
       0 0 1  0;
       0 0 0  1];

RX2 = [1    0        0     0;
       0  cos(0)  -sin(0)  0;
       0  sin(0)   cos(0)  0;
       0    0        0     1];

T12 = RZ1 * TZ1 * RX2 * TX2

% --- Eslabón 3: θ3+π/2, d=0, α=π/2, a=0 (sistema 3 coincide con el 2) ---
TZ2 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RZ2 = [cos(q3+pi/2) -sin(q3+pi/2) 0 0;
       sin(q3+pi/2)  cos(q3+pi/2) 0 0;
       0             0            1 0;
       0             0            0 1];

TX3 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RX3 = [1      0           0       0;
       0  cos(pi/2)  -sin(pi/2)   0;
       0  sin(pi/2)   cos(pi/2)   0;
       0      0           0       1];

T23 = RZ2 * TZ2 * RX3 * TX3

% --- Eslabón 4: θ4+π/2, d=L3+L4, α=π/2, a=0 (absorbe la distancia de L3) ---
TZ3 = [1 0 0  0;
       0 1 0  0;
       0 0 1 L3+L4;
       0 0 0  1];

RZ3 = [cos(q4+pi/2) -sin(q4+pi/2) 0 0;
       sin(q4+pi/2)  cos(q4+pi/2) 0 0;
       0             0            1 0;
       0             0            0 1];

TX4 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RX4 = [1      0           0       0;
       0  cos(pi/2)  -sin(pi/2)   0;
       0  sin(pi/2)   cos(pi/2)   0;
       0      0           0       1];

T34 = RZ3 * TZ3 * RX4 * TX4

% --- Eslabón 5: θ5+π, d=0, α=π/2, a=0 (sistema 5 coincide con el 4, +180° para que quede bien orientado en q5=0) ---
TZ4 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RZ4 = [cos(q5+pi) -sin(q5+pi) 0 0;
       sin(q5+pi)  cos(q5+pi) 0 0;
       0           0          1 0;
       0           0          0 1];

TX5 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RX5 = [1      0           0       0;
       0  cos(pi/2)  -sin(pi/2)   0;
       0  sin(pi/2)   cos(pi/2)   0;
       0      0           0       1];

T45 = RZ4 * TZ4 * RX5 * TX5

% --- Eslabón 6: θ6, d=L5+L6, α=0, a=0 (absorbe la distancia de L5) ---
TZ5 = [1 0 0  0;
       0 1 0  0;
       0 0 1 L5+L6;
       0 0 0  1];

RZ5 = [cos(q6) -sin(q6) 0 0;
       sin(q6)  cos(q6) 0 0;
       0        0       1 0;
       0        0       0 1];

TX6 = [1 0 0 0;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

RX6 = [1    0        0     0;
       0  cos(0)  -sin(0)  0;
       0  sin(0)   cos(0)  0;
       0    0        0     1];

T56 = RZ5 * TZ5 * RX6 * TX6

% Transformación total
T06 = T01 * T12 * T23 * T34 * T45 * T56

% Verificación con ángulos de Euler (ZYX)
m = T06(1:3, 1:3);
r = rad2deg(tr2rpy(m, 'zyx'))
