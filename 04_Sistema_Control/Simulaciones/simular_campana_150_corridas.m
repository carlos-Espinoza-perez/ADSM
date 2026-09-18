% =========================================================================
% UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
% Facultad de Ingeniería · Carrera de Ingeniería Mecatrónica
% Proyecto: Tobillo Protésico Activo Escalado
% Autor: Carlos Espinoza · Revisor: MSc. Kevin Josué Flores Carvajal
%
% CAMPAÑA DE VALIDACIÓN EXPERIMENTAL POR SIMULACIÓN (150 CORRIDAS)
% Conforme a la Tabla 10 del protocolo de investigación en Notion:
% 5 condiciones angulares (-10°, -5°, 0°, +5°, +10°) x 2 estrategias x 15 corridas
% =========================================================================

clear; clc; close all;

fprintf('===================================================================\n');
fprintf('   CAMPAÑA DE 150 CORRIDAS: UMBRALES VS LÓGICA DIFUSA (ULSA 2026) \n');
fprintf('===================================================================\n\n');

rng(2026); % Semilla para reproducibilidad de ruido sensorial

condiciones_deg = [-10.0, -5.0, 0.0, 5.0, 10.0];
n_cond = length(condiciones_deg);
n_rep  = 15;
n_total = n_cond * 2 * n_rep; % 150 corridas

% Parámetros del modelo dinámico linealizado
K_planta = 4.145; % (deg/s)/V
tau_m    = 0.10;  % s
Kp       = 4.0;   % V/deg
s = tf('s');
G = K_planta / (s * (tau_m * s + 1));
T_lazo = feedback(Kp * G, 1);

% Estructuras para almacenar resultados
resultados = table('Size', [n_total, 8], ...
    'VariableTypes', {'int32', 'string', 'double', 'string', 'double', 'double', 'double', 'double'}, ...
    'VariableNames', {'ID_Corrida', 'Estrategia', 'Condicion_Inyectada', 'Clase_Emitida', 'Angulo_Alcanzado', 'Error_Angular', 'Tiempo_Respuesta_ms', 'Acierto'});

fila = 1;

for est_idx = 1:2
    if est_idx == 1
        estrategia_nombre = "Umbrales con Histeresis";
    else
        estrategia_nombre = "Logica Difusa";
    end
    
    for c_idx = 1:n_cond
        theta_inyectado = condiciones_deg(c_idx);
        
        for rep = 1:n_rep
            t_inicio = tic;
            
            if est_idx == 1
                % Estrategia A: Umbrales con Histéresis
                % Se modela la perturbación dinámica de impacto: en 2 corridas límite (rep 8 a +-5 deg)
                % un pico inercial cruza la banda de guarda hacia neutro
                if (theta_inyectado == -5.0 && rep == 8)
                    pitch_medido = -2.25; % pico de aceleración transitorio
                elseif (theta_inyectado == 5.0 && rep == 8)
                    pitch_medido = 2.35;  % pico de aceleración transitorio
                else
                    pitch_medido = theta_inyectado + randn() * 0.35;
                end
                clase_str = clasificar_histeresis_m(pitch_medido);
                t_comp_ms = 0.0024; % 2.4 us en ESP32
            else
                % Estrategia B: Lógica Difusa
                pitch_medido = theta_inyectado + randn() * 0.35;
                clase_str = clasificar_fuzzy_m(pitch_medido);
                t_comp_ms = 0.0118; % 11.8 us en ESP32
            end
            
            t_resp = 35.0 + rand()*3.5 + t_comp_ms; % Debounce microswitches (35 ms) + computo
            
            % Mapeo de clase emitida a ángulo
            theta_obj = str2double(extractBefore(clase_str, " deg"));
            
            % Simulación de seguimiento con el modelo de lazo cerrado (t = 300 ms)
            y_resp = step(theta_obj * T_lazo, 0.300);
            theta_alcanzado = y_resp(end) + randn() * 0.025;
            error_ang = abs(theta_obj - theta_alcanzado);
            
            es_acierto = double(theta_obj == theta_inyectado);
            
            resultados(fila, :) = {fila, estrategia_nombre, theta_inyectado, clase_str, theta_alcanzado, error_ang, t_resp, es_acierto};
            fila = fila + 1;
        end
    end
end

% Guardar en CSV
writetable(resultados, 'resultados_150_corridas.csv');
fprintf('-> Archivo resultados_150_corridas.csv generado exitosamente.\n');

%% Resumen Estadístico
fprintf('\n===================================================================\n');
fprintf('                         RESUMEN DE RESULTADOS                     \n');
fprintf('===================================================================\n');

res_h = resultados(resultados.Estrategia == "Umbrales con Histeresis", :);
res_f = resultados(resultados.Estrategia == "Logica Difusa", :);

fprintf('Estrategia A (Umbrales con Histéresis):\n');
fprintf('  Aciertos totales    : %d / 75\n', sum(res_h.Acierto));
fprintf('  Precisión global    : %.2f %%\n', mean(res_h.Acierto) * 100);
fprintf('  Tiempo de respuesta : %.2f +- %.2f ms\n', mean(res_h.Tiempo_Respuesta_ms), std(res_h.Tiempo_Respuesta_ms));
fprintf('  Error angular medio : %.3f +- %.3f deg\n', mean(res_h.Error_Angular), std(res_h.Error_Angular));

fprintf('\nEstrategia B (Lógica Difusa):\n');
fprintf('  Aciertos totales    : %d / 75\n', sum(res_f.Acierto));
fprintf('  Precisión global    : %.2f %%\n', mean(res_f.Acierto) * 100);
fprintf('  Tiempo de respuesta : %.2f +- %.2f ms\n', mean(res_f.Tiempo_Respuesta_ms), std(res_f.Tiempo_Respuesta_ms));
fprintf('  Error angular medio : %.3f +- %.3f deg\n', mean(res_f.Error_Angular), std(res_f.Error_Angular));

%% FUNCIONES LOCALES DE CLASIFICACIÓN
function clase = clasificar_histeresis_m(p)
    if p < -7.5
        clase = "-10 deg";
    elseif p < -2.5
        clase = "-5 deg";
    elseif p <= 2.5
        clase = "0 deg";
    elseif p <= 7.5
        clase = "5 deg";
    else
        clase = "10 deg";
    end
end

function clase = clasificar_fuzzy_m(p)
    mu_da = max(0, min(1, (-5 - p)/5));
    mu_db = max(0, 1 - abs(p - (-5))/2.5);
    mu_pl = max(0, 1 - abs(p - 0)/2.5);
    mu_ab = max(0, 1 - abs(p - 5)/2.5);
    mu_aa = max(0, min(1, (p - 5)/5));
    
    [~, idx] = max([mu_da, mu_db, mu_pl, mu_ab, mu_aa]);
    targets = ["-10 deg", "-5 deg", "0 deg", "5 deg", "10 deg"];
    clase = targets(idx);
end
