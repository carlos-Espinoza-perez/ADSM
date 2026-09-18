% =========================================================================
% UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
% Facultad de Ingeniería · Ingeniería Mecatrónica
% Proyecto: Tobillo Protésico Activo Escalado
% Autor: Carlos Espinoza · Revisor: MSc. Kevin Josué Flores Carvajal
%
% MODELO DINÁMICO LINEALIZADO ALREDEDOR DEL PUNTO DE OPERACIÓN (NEUTRO)
% Cumple la observación del docente del 18 de septiembre de 2026:
% "La función de transferencia de segundo orden se describe formalmente
%  como un modelo linealizado alrededor de un punto de operación, dado que
%  la transmisión por husillo, el reductor y la fricción introducen
%  dinámicas fuertemente no lineales."
% =========================================================================

clear; clc; close all;

fprintf('===================================================================\n');
fprintf('   MODELACIÓN LINEALIZADA Y ANÁLISIS EN LAZO CERRADO - TOBILLO ULSA\n');
fprintf('===================================================================\n\n');

%% 1. PARÁMETROS MECATRONICOS NOMINALES
% Actuador GA25-370 12V (medido / ficha)
V_nom   = 12.0;               % Voltaje nominal del motor (V)
rpm_nom = 500.0;              % Velocidad nominal de salida sin carga (rpm)
w_nom   = rpm_nom * (2*pi/60);% rad/s (aprox. 52.36 rad/s)
tau_m   = 0.10;               % Constante de tiempo electromecánica (s)

% Ganancia de velocidad del motorreductor: rad/s por cada Voltio
Km = w_nom / V_nom;           % (rad/s)/V = 4.363

% Transmisión mecánica: Husillo M8 + Palanca
p_husillo = 1.25e-3;          % Paso del husillo: 1.25 mm/rev (m/rev)
r_palanca = 12.0e-3;          % Brazo de palanca sagital: 12 mm (m)
theta_0   = 0.0;              % Punto de operación: posición neutra (0 rad)

%% 2. LINEALIZACIÓN DE LA CINEMÁTICA ALREDEDOR DE theta = 0
% Cinemática no lineal: y(theta) = -r * sin(theta)
% Linealización en Taylor primer orden:
% dy/dtheta |(theta=0) = -r * cos(0) = -r
% Relación inversa: dtheta/dy |(y=0) = -1 / r

% Sensibilidad cinemática angular por radián de giro del husillo:
% Un giro completo del husillo (2*pi rad) avanza p_husillo metros.
% Delta_y = (theta_m / (2*pi)) * p_husillo
% Delta_theta_art = Delta_y / r_palanca = theta_m * (p_husillo / (2*pi * r_palanca))
S_cin_rad = p_husillo / (2 * pi * r_palanca); % rad articular / rad motor
S_cin_deg = S_cin_rad * (180 / pi);           % deg articular / rad motor
S_rev_deg = S_cin_deg * (2 * pi);             % deg articular / rev motor = 5.968 deg/rev

fprintf('-> Sensibilidad cinemática linealizada: %.4f deg/rev motor\n', S_rev_deg);
fprintf('-> Ganancia estática motor: %.3f (rad/s)/V\n', Km);

%% 3. FUNCIÓN DE TRANSFERENCIA EN LAZO ABIERTO
% Entrada: Voltaje de control V(s) [V]
% Salida:  Ángulo articular Theta(s) [grados]
% G(s) = Theta(s) / V(s) = (Km * S_cin_deg) / (s * (tau_m * s + 1))

K_planta = Km * S_cin_deg;    % (deg/s)/V = 4.363 * 0.950 = 4.145 (deg/s)/V
s = tf('s');
G_planta = K_planta / (s * (tau_m * s + 1));

fprintf('-> Función de transferencia linealizada G(s):\n');
G_planta

%% 4. CONTROLADOR PROPORCIONAL Y ANÁLISIS EN LAZO CERRADO
% Ganancia del lazo proporcional implementada en firmware v6:
Kp = 4.0; % V/deg

% Lazo cerrado directo
T_lazo = feedback(Kp * G_planta, 1);

fprintf('-> Función de transferencia en lazo cerrado T(s):\n');
T_lazo

% Polos y amortiguamiento
[wn, zeta, polos] = damp(T_lazo);
fprintf('\n-> Parámetros de lazo cerrado:\n');
fprintf('   Frecuencia natural (wn) : %.2f rad/s\n', wn(1));
fprintf('   Factor de amortiguamiento: %.3f\n', zeta(1));
fprintf('   Polos dominantes        : %.2f + %.2fj\n', real(polos(1)), imag(polos(1)));

%% 5. SIMULACIÓN DE RESPUESTA AL ESCALÓN (OBJETIVO 300 ms)
% Evaluamos la transición típica entre clases de terreno: salto de 5°
t_sim = 0:0.001:0.6; % 600 ms de simulación
theta_ref = 5.0;     % Escalón de 5 grados

[y_step, t_out] = step(theta_ref * T_lazo, t_sim);

% Cálculo métrico
info = stepinfo(T_lazo);
t_asentamiento = info.SettlingTime;
sobrepaso      = info.Overshoot;

fprintf('\n-> Desempeño dinámico:\n');
fprintf('   Tiempo de asentamiento (ts al 2%%): %.3f s (%.1f ms)\n', t_asentamiento, t_asentamiento*1000);
fprintf('   Sobrepaso porcentual (Mp)       : %.2f %%\n', sobrepaso);
if t_asentamiento <= 0.300
    fprintf('   ESTADO: CUMPLE EL CRITERIO DE DISENO (< 300 ms)\n');
else
    fprintf('   ESTADO: SUPERA LOS 300 ms (ajustar Kp)\n');
end

%% 6. GRAFICACIÓN
figure('Color', 'white', 'Position', [100, 100, 800, 500]);
subplot(2,1,1);
plot(t_out*1000, y_step, 'b-', 'LineWidth', 1.8); hold on;
yline(theta_ref, 'k--', 'Referencia 5°', 'LineWidth', 1.2);
xline(300, 'r:', 'Límite 300 ms', 'LineWidth', 1.4);
grid on;
title('Respuesta al Escalón del Modelo Linealizado (Salto de 5°)', 'FontWeight', 'bold');
xlabel('Tiempo (ms)'); ylabel('Ángulo Articular (°)');
xlim([0, 500]); ylim([0, 6.0]);

subplot(2,1,2);
% Comparación de no linealidad: cinemática exacta vs lineal
theta_sweep = -15:0.5:15;
y_exact = -r_palanca * sind(theta_sweep) * 1000; % mm
y_linear = -r_palanca * deg2rad(theta_sweep) * 1000; % mm
error_lin = abs(y_exact - y_linear);

plot(theta_sweep, error_lin, 'r-', 'LineWidth', 1.6);
grid on;
title('Error de la Linealización Cinemática en el Rango ±15°', 'FontWeight', 'bold');
xlabel('Ángulo Articular (°)'); ylabel('Error Absoluto (mm)');
xlim([-15, 15]);

fprintf('\nCálculo finalizado con éxito.\n');
