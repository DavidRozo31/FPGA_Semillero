%% =========================================================
%  Testbench MATLAB - Cinemática Inversa 2R en Q2.13
%  Verifica cada etapa del pipeline FPGA paso a paso
%  Formato: decimal real  +  entero Q2.13 (escala = 8192)
% ==========================================================

clc; clear;

% =========================================================
%  PARAMETROS DE ENTRADA - modificar aqui
% =========================================================
Px = 0.10;
Py = 0.08;
L1 = 0.08;
L2 = 0.07;

% =========================================================
%  Constantes
% =========================================================
SCALE = 8192;   % 2^13
One   = 1.0;
Two   = 2.0;

% Funcion de conversion
toQ = @(x) round(x * SCALE);

% =========================================================
%  Encabezado
% =========================================================
fprintf('==========================================================\n');
fprintf('  TESTBENCH MATLAB - Cinemática Inversa 2R  Q2.13\n');
fprintf('==========================================================\n');
fprintf('  ENTRADAS:\n');
fprintf('    Px = %.4f  ->  Q2.13 = %d\n', Px, toQ(Px));
fprintf('    Py = %.4f  ->  Q2.13 = %d\n', Py, toQ(Py));
fprintf('    L1 = %.4f  ->  Q2.13 = %d\n', L1, toQ(L1));
fprintf('    L2 = %.4f  ->  Q2.13 = %d\n', L2, toQ(L2));
fprintf('    1  = %.4f  ->  Q2.13 = %d\n', One, toQ(One));
fprintf('    2  = %.4f  ->  Q2.13 = %d\n', Two, toQ(Two));
fprintf('==========================================================\n\n');

% =========================================================
%  PASO 1: Px^2  y  Py^2
% =========================================================
Px2 = Px * Px;
Py2 = Py * Py;
fprintf('--- PASO 1: Px^2  y  Py^2 ---\n');
fprintf('  Px2 = Px*Px = %.6f  ->  Q2.13 = %d\n', Px2, toQ(Px2));
fprintf('  Py2 = Py*Py = %.6f  ->  Q2.13 = %d\n', Py2, toQ(Py2));
fprintf('\n');

% =========================================================
%  PASO 2: L1^2  y  L2^2
% =========================================================
L1sq = L1 * L1;
L2sq = L2 * L2;
fprintf('--- PASO 2: L1^2  y  L2^2 ---\n');
fprintf('  L1sq = L1*L1 = %.6f  ->  Q2.13 = %d\n', L1sq, toQ(L1sq));
fprintf('  L2sq = L2*L2 = %.6f  ->  Q2.13 = %d\n', L2sq, toQ(L2sq));
fprintf('\n');

% =========================================================
%  PASO 3: b2 = Px^2 + Py^2
% =========================================================
b2 = Px2 + Py2;
fprintf('--- PASO 3: b2 = Px^2 + Py^2 ---\n');
fprintf('  b2 = %.6f  ->  Q2.13 = %d\n', b2, toQ(b2));
fprintf('\n');

% =========================================================
%  PASO 4: Lsum = L1^2 + L2^2
% =========================================================
Lsum = L1sq + L2sq;
fprintf('--- PASO 4: Lsum = L1^2 + L2^2 ---\n');
fprintf('  Lsum = %.6f  ->  Q2.13 = %d\n', Lsum, toQ(Lsum));
fprintf('\n');

% =========================================================
%  PASO 5: Two*L1  y  den = 2*L1*L2
% =========================================================
TwoL1 = Two * L1;
den   = TwoL1 * L2;
fprintf('--- PASO 5: Two*L1  y  den = 2*L1*L2 ---\n');
fprintf('  TwoL1 = 2*L1 = %.6f  ->  Q2.13 = %d\n', TwoL1, toQ(TwoL1));
fprintf('  den   = 2*L1*L2 = %.6f  ->  Q2.13 = %d\n', den, toQ(den));
fprintf('\n');

% =========================================================
%  PASO 6: numerador = b2 - Lsum
% =========================================================
numerador = b2 - Lsum;
fprintf('--- PASO 6: numerador = b2 - Lsum ---\n');
fprintf('  numerador = %.6f  ->  Q2.13 = %d\n', numerador, toQ(numerador));
fprintf('\n');

% =========================================================
%  PASO 7: cos(theta2) = numerador / den
% =========================================================
cos_t2 = numerador / den;
fprintf('--- PASO 7: cos(theta2) = numerador / den ---\n');
fprintf('  cos_t2 = %.6f  ->  Q2.13 = %d\n', cos_t2, toQ(cos_t2));
fprintf('\n');

% Verificacion de rango
if abs(cos_t2) > 1
    warning('  *** cos_t2 fuera de rango [-1,1]: punto (Px,Py) inalcanzable ***\n');
end

% =========================================================
%  PASO 8: cos_t2^2
% =========================================================
cos2 = cos_t2 * cos_t2;
fprintf('--- PASO 8: cos_t2^2 ---\n');
fprintf('  cos2 = %.6f  ->  Q2.13 = %d\n', cos2, toQ(cos2));
fprintf('\n');

% =========================================================
%  PASO 9: 1 - cos_t2^2
% =========================================================
one_mcos2 = One - cos2;
fprintf('--- PASO 9: 1 - cos_t2^2 ---\n');
fprintf('  1-cos2 = %.6f  ->  Q2.13 = %d\n', one_mcos2, toQ(one_mcos2));
fprintf('\n');

% =========================================================
%  PASO 10: sin(theta2) = sqrt(1 - cos_t2^2)
% =========================================================
sin_t2 = sqrt(one_mcos2);
fprintf('--- PASO 10: sin(theta2) = sqrt(1 - cos_t2^2) ---\n');
fprintf('  sin_t2 = %.6f  ->  Q2.13 = %d\n', sin_t2, toQ(sin_t2));
fprintf('\n');

% =========================================================
%  PASO 11: theta2 = atan2(sin_t2, cos_t2)
%  [CORDIC: x_in=cos_t2, y_in=sin_t2]
% =========================================================
theta2 = atan2(sin_t2, cos_t2);
fprintf('--- PASO 11: theta2 = atan2(sin_t2, cos_t2) ---\n');
fprintf('  theta2 = %.6f rad  ->  Q2.13 = %d\n', theta2, toQ(theta2));
fprintf('  theta2 = %.4f deg\n', rad2deg(theta2));
fprintf('\n');

% =========================================================
%  PASO 12: L2*cos_t2  y  L2*sin_t2
% =========================================================
L2cos = L2 * cos_t2;
L2sin = L2 * sin_t2;
fprintf('--- PASO 12: L2*cos_t2  y  L2*sin_t2 ---\n');
fprintf('  L2cos = L2*cos_t2 = %.6f  ->  Q2.13 = %d\n', L2cos, toQ(L2cos));
fprintf('  L2sin = L2*sin_t2 = %.6f  ->  Q2.13 = %d\n', L2sin, toQ(L2sin));
fprintf('\n');

% =========================================================
%  PASO 13: k1 = L1 + L2*cos_t2
% =========================================================
k1 = L1 + L2cos;
fprintf('--- PASO 13: k1 = L1 + L2*cos_t2 ---\n');
fprintf('  k1 = %.6f  ->  Q2.13 = %d\n', k1, toQ(k1));
fprintf('\n');

% =========================================================
%  PASO 14: angle_base = atan2(Py, Px)
%  [CORDIC: x_in=Py, y_in=Px -> atan2(Px,Py) segun BDF]
%  Nota: el BDF conecta x_in=Py, y_in=Px
%  atan2(y,x) con y=Px, x=Py = atan2(Px,Py)
%  Para el robot 2R la formula correcta es atan2(Py,Px),
%  pero el CORDIC calcula atan2(y_in, x_in):
%    con x_in=Py, y_in=Px  ->  atan2(Px, Py)
%  Si esto da resultado incorrecto, revisar conexion en BDF.
% =========================================================
angle_base_bdf  = atan2(Px, Py);   % lo que calcula el BDF actual
angle_base_real = atan2(Py, Px);   % lo que deberia ser matematicamente
fprintf('--- PASO 14: angle_base ---\n');
fprintf('  BDF  calcula atan2(Px, Py) = %.6f rad  ->  Q2.13 = %d\n', angle_base_bdf,  toQ(angle_base_bdf));
fprintf('  Real deberia ser atan2(Py, Px) = %.6f rad  ->  Q2.13 = %d\n', angle_base_real, toQ(angle_base_real));
if abs(angle_base_bdf - angle_base_real) > 1e-6
    fprintf('  *** ADVERTENCIA: conexion x_in/y_in en b2v_inst30 esta invertida ***\n');
else
    fprintf('  OK: coinciden (caso simetrico Px=Py)\n');
end
fprintf('\n');

% =========================================================
%  PASO 15: angle_corr = atan2(L2*sin_t2, k1)
%  [CORDIC: x_in=k1, y_in=L2sin]
% =========================================================
angle_corr = atan2(L2sin, k1);
fprintf('--- PASO 15: angle_corr = atan2(L2*sin, k1) ---\n');
fprintf('  angle_corr = %.6f rad  ->  Q2.13 = %d\n', angle_corr, toQ(angle_corr));
fprintf('  angle_corr = %.4f deg\n', rad2deg(angle_corr));
fprintf('\n');

% =========================================================
%  PASO 16: theta1 = angle_base - angle_corr
%  [fp_adder: op=1 -> a_in - b_in]
%  BDF correcto: a_in=angle_base(WIRE_44), b_in=angle_corr(WIRE_43)
% =========================================================
theta1 = angle_base_real - angle_corr;
fprintf('--- PASO 16: theta1 = angle_base - angle_corr ---\n');
fprintf('  theta1 = %.6f rad  ->  Q2.13 = %d\n', theta1, toQ(theta1));
fprintf('  theta1 = %.4f deg\n', rad2deg(theta1));
fprintf('\n');

% =========================================================
%  VERIFICACION FINAL - forward kinematics
% =========================================================
Px_check = L1*cos(theta1) + L2*cos(theta1 + theta2);
Py_check = L1*sin(theta1) + L2*sin(theta1 + theta2);
err_x = abs(Px_check - Px);
err_y = abs(Py_check - Py);

fprintf('==========================================================\n');
fprintf('  VERIFICACION FINAL (FK desde angulos calculados)\n');
fprintf('==========================================================\n');
fprintf('  Px original = %.6f  |  Px reconstruido = %.6f  |  error = %.2e\n', Px, Px_check, err_x);
fprintf('  Py original = %.6f  |  Py reconstruido = %.6f  |  error = %.2e\n', Py, Py_check, err_y);
if err_x < 1e-4 && err_y < 1e-4
    fprintf('  RESULTADO: OK - los angulos son correctos\n');
else
    fprintf('  RESULTADO: ERROR - revisar pipeline\n');
end

fprintf('\n');
fprintf('==========================================================\n');
fprintf('  RESUMEN Q2.13 - para comparar con ModelSim\n');
fprintf('==========================================================\n');
fprintf('  %-20s %8s   %8s\n', 'Variable', 'Real', 'Q2.13');
fprintf('  %-20s %8.4f   %8d\n', 'Px2',        Px2,         toQ(Px2));
fprintf('  %-20s %8.4f   %8d\n', 'Py2',        Py2,         toQ(Py2));
fprintf('  %-20s %8.4f   %8d\n', 'L1sq',       L1sq,        toQ(L1sq));
fprintf('  %-20s %8.4f   %8d\n', 'L2sq',       L2sq,        toQ(L2sq));
fprintf('  %-20s %8.4f   %8d\n', 'b2',         b2,          toQ(b2));
fprintf('  %-20s %8.4f   %8d\n', 'Lsum',       Lsum,        toQ(Lsum));
fprintf('  %-20s %8.4f   %8d\n', 'TwoL1',      TwoL1,       toQ(TwoL1));
fprintf('  %-20s %8.4f   %8d\n', 'den',        den,         toQ(den));
fprintf('  %-20s %8.4f   %8d\n', 'numerador',  numerador,   toQ(numerador));
fprintf('  %-20s %8.4f   %8d\n', 'cos_t2',     cos_t2,      toQ(cos_t2));
fprintf('  %-20s %8.4f   %8d\n', 'cos2',       cos2,        toQ(cos2));
fprintf('  %-20s %8.4f   %8d\n', '1-cos2',     one_mcos2,   toQ(one_mcos2));
fprintf('  %-20s %8.4f   %8d\n', 'sin_t2',     sin_t2,      toQ(sin_t2));
fprintf('  %-20s %8.4f   %8d\n', 'theta2 (rad)', theta2,    toQ(theta2));
fprintf('  %-20s %8.4f   %8d\n', 'L2cos',      L2cos,       toQ(L2cos));
fprintf('  %-20s %8.4f   %8d\n', 'L2sin',      L2sin,       toQ(L2sin));
fprintf('  %-20s %8.4f   %8d\n', 'k1',         k1,          toQ(k1));
fprintf('  %-20s %8.4f   %8d\n', 'angle_base', angle_base_real, toQ(angle_base_real));
fprintf('  %-20s %8.4f   %8d\n', 'angle_corr', angle_corr,  toQ(angle_corr));
fprintf('  %-20s %8.4f   %8d\n', 'theta1 (rad)', theta1,    toQ(theta1));
fprintf('==========================================================\n');

% =========================================================
%  DUTY CYCLE PARA SERVOS (ik_pwm_encoder)
% =========================================================
SCALE_NUM = 1943;
SCALE_DEN = 1000;
DUTY_MIN  = 50000;
DUTY_MAX  = 100000;

% Función de conversión ángulo → duty cycle
toDuty = @(theta_q13) min(DUTY_MAX, max(DUTY_MIN, ...
         DUTY_MIN + floor(max(0, theta_q13) * SCALE_NUM / SCALE_DEN)));

duty1 = toDuty(toQ(theta1));
duty2 = toDuty(toQ(theta2));

fprintf('==========================================================\n');
fprintf('  DUTY CYCLE - Salida ik_pwm_encoder\n');
fprintf('==========================================================\n');
fprintf('  %-20s %8s   %8s   %8s\n', 'Variable', 'Rad', 'Q2.13', 'Duty');
fprintf('  %-20s %8.4f   %8d   %8d\n', 'theta1', theta1, toQ(theta1), duty1);
fprintf('  %-20s %8.4f   %8d   %8d\n', 'theta2', theta2, toQ(theta2), duty2);
fprintf('\n');
fprintf('  Servo1 pulso = %.4f ms  (%d cuentas / 1000000 × 20ms)\n', ...
        duty1/1000000*20, duty1);
fprintf('  Servo2 pulso = %.4f ms  (%d cuentas / 1000000 × 20ms)\n', ...
        duty2/1000000*20, duty2);
fprintf('==========================================================\n');


% =========================================================
%  VISUALIZACION CON PETER CORKE ROBOTICS TOOLBOX
% =========================================================

% Verificar que el toolbox esté disponible
if ~exist('SerialLink', 'class') && ~exist('rigidBodyTree', 'class')
    warning('Peter Corke Toolbox no encontrado. Usando visualización básica.');
else

% =========================================================
%  Definir el robot 2R con parámetros DH
%  Convencion Denavit-Hartenberg:
%  [theta, d, a, alpha]
%  theta → variable de articulación (se deja en 0, se asigna luego)
%  d     → desplazamiento a lo largo de Z
%  a     → longitud del eslabón
%  alpha → rotación alrededor de X
% =========================================================

% Eslabón 1: longitud L1, en el plano XY → alpha=0
L_1 = Link('d', 0, 'a', L1, 'alpha', 0, 'revolute');

% Eslabón 2: longitud L2, en el plano XY → alpha=0
L_2 = Link('d', 0, 'a', L2, 'alpha', 0, 'revolute');

% Crear robot
robot = SerialLink([L_1 L_2], 'name', 'Robot 2R FPGA');

% =========================================================
%  Configuracion de articulaciones
%  q = [theta1, theta2] en radianes
% =========================================================
q = [theta1, theta2];

fprintf('==========================================================\n');
fprintf('  PETER CORKE TOOLBOX\n');
fprintf('==========================================================\n');
fprintf('  q = [%.4f, %.4f] rad\n', theta1, theta2);
fprintf('  q = [%.4f, %.4f] deg\n', rad2deg(theta1), rad2deg(theta2));

% Cinemática directa con el toolbox para verificar
T = robot.fkine(q);
fprintf('  FK Toolbox → posición extremo:\n');
fprintf('    x = %.6f m  (objetivo: %.6f)\n', T.t(1), Px);
fprintf('    y = %.6f m  (objetivo: %.6f)\n', T.t(2), Py);
fprintf('==========================================================\n');

% =========================================================
%  Figura 1: visualización 3D del robot
% =========================================================
figure('Name', 'Robot 2R - Peter Corke Toolbox', ...
       'NumberTitle', 'off');

robot.plot(q, ...
    'workspace', [-0.2 0.2 -0.2 0.2 -0.05 0.05], ...
    'jointdiam', 1.5, ...
    'jvec', ...
    'shadow', ...
    'tile1color', [0.85 0.85 0.85], ...
    'tile2color', [0.75 0.75 0.75]);

title(sprintf('Robot 2R  |  θ1=%.1f°  θ2=%.1f°  |  Objetivo:(%.3f, %.3f)m', ...
      rad2deg(theta1), rad2deg(theta2), Px, Py));

% Marcar el punto objetivo en la figura
hold on;
plot3(Px, Py, 0, 'rx', 'MarkerSize', 14, 'LineWidth', 3);
text(Px+0.005, Py+0.005, 0, ...
     sprintf('Objetivo\n(%.3f, %.3f)', Px, Py), ...
     'FontSize', 9, 'Color', 'red');
hold off;

% =========================================================
%  Figura 2: trayectoria entre posición home y objetivo
%  Interpola N pasos entre q_home=[0,0] y q=[theta1,theta2]
% =========================================================
figure('Name', 'Robot 2R - Trayectoria', ...
       'NumberTitle', 'off');

N_pasos = 50;
q_home  = [0, 0];
q_traj  = jtraj(q_home, q, N_pasos);  % interpolación suave

robot.plot(q_traj, ...
    'workspace', [-0.2 0.2 -0.2 0.2 -0.05 0.05], ...
    'jointdiam', 1.5, ...
    'trail', 'r-', ...
    'shadow');

title(sprintf('Trayectoria: home [0°,0°] → [%.1f°, %.1f°]', ...
      rad2deg(theta1), rad2deg(theta2)));

% =========================================================
%  Figura 3: gráfica 2D del espacio de trabajo + posición
% =========================================================
figure('Name', 'Robot 2R - Espacio de trabajo', ...
       'NumberTitle', 'off', ...
       'Color', [0.15 0.15 0.15]);

hold on;
axis equal;
grid on;
set(gca, 'Color',     [0.1  0.1  0.1 ], ...
         'XColor',    [0.8  0.8  0.8 ], ...
         'YColor',    [0.8  0.8  0.8 ], ...
         'GridColor', [0.3  0.3  0.3 ]);

% Espacio de trabajo: barrido de todas las configuraciones posibles
theta1_range = linspace(-pi, pi, 200);
theta2_range = linspace(-pi, pi, 200);
[T1, T2] = meshgrid(theta1_range, theta2_range);
Xws = L1*cos(T1) + L2*cos(T1+T2);
Yws = L1*sin(T1) + L2*sin(T1+T2);
plot(Xws(:), Yws(:), '.', 'Color', [0.2 0.3 0.4], 'MarkerSize', 1);

% Circulos de alcance
theta_circ = linspace(0, 2*pi, 300);
plot((L1+L2)*cos(theta_circ), (L1+L2)*sin(theta_circ), ...
     '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
plot(abs(L1-L2)*cos(theta_circ), abs(L1-L2)*sin(theta_circ), ...
     '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

% Eslabon 1
x1_plot = L1*cos(theta1);
y1_plot = L1*sin(theta1);
plot([0 x1_plot], [0 y1_plot], '-', ...
     'Color', [0.2 0.6 1.0], 'LineWidth', 4, ...
     'DisplayName', sprintf('L1=%.2fm', L1));

% Eslabon 2
plot([x1_plot Px], [y1_plot Py], '-', ...
     'Color', [0.2 1.0 0.5], 'LineWidth', 4, ...
     'DisplayName', sprintf('L2=%.2fm', L2));

% Articulaciones
plot(0,       0,  'o', 'MarkerSize', 12, ...
     'MarkerFaceColor', [1.0 0.8 0.0], 'MarkerEdgeColor', 'white', ...
     'LineWidth', 1.5, 'DisplayName', 'Base');
plot(x1_plot, y1_plot, 'o', 'MarkerSize', 10, ...
     'MarkerFaceColor', [0.2 0.6 1.0], 'MarkerEdgeColor', 'white', ...
     'LineWidth', 1.5, 'DisplayName', 'Articulación 1');
plot(Px, Py, 'x', 'MarkerSize', 16, ...
     'Color', [1.0 0.3 0.3], 'LineWidth', 3, ...
     'DisplayName', sprintf('Objetivo(%.3f,%.3f)', Px, Py));

% Info box
info_str = sprintf(['L1=%.3fm  L2=%.3fm\n' ...
                    'θ1=%.2f°  θ2=%.2f°\n' ...
                    'Duty1=%d  Duty2=%d'], ...
                    L1, L2, ...
                    rad2deg(theta1), rad2deg(theta2), ...
                    duty1, duty2);
text(-(L1+L2)*0.95, (L1+L2)*0.80, info_str, ...
     'Color', [0.9 0.9 0.9], 'FontSize', 8.5, ...
     'BackgroundColor', [0.2 0.2 0.2], ...
     'EdgeColor', [0.5 0.5 0.5], ...
     'FontName', 'Courier New');

title('Espacio de trabajo + posición actual', ...
      'Color', 'white', 'FontSize', 11);
xlabel('X (m)', 'Color', [0.8 0.8 0.8]);
ylabel('Y (m)', 'Color', [0.8 0.8 0.8]);
legend('show', 'Location', 'southeast', ...
       'TextColor', 'white', ...
       'Color', [0.2 0.2 0.2], ...
       'EdgeColor', [0.5 0.5 0.5]);

hold off;

end  % fin del if toolbox disponible