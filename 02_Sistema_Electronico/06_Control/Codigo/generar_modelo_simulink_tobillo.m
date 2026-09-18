function generar_modelo_simulink_tobillo()
% =========================================================================
% UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · LEÓN, NICARAGUA
% Facultad de Ingeniería · Carrera de Ingeniería Mecatrónica
% Proyecto: Tobillo Protésico Activo Escalado
%
% MODELO BÁSICO Y DIRECTO DE CONTROL EN SIMULINK (SIN COMPLICACIONES)
% Compatible con MATLAB Desktop y MATLAB Online
% =========================================================================

clc;
disp('===================================================================');
disp('   GENERANDO MODELO SIMPLE DE CONTROL EN SIMULINK - TOBILLO ULSA   ');
disp('===================================================================');

mdl = 'Tobillo_Protesico_Control_v6';

% 1. Cargar librerías base
if ~bdIsLoaded('simulink')
    load_system('simulink');
end

% 2. Cerrar si ya estaba abierto
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

% 3. Crear sistema limpio
new_system(mdl);

% Configuración estándar del solver (Automático y continuo)
set_param(mdl, 'Solver', 'ode45');
set_param(mdl, 'StopTime', '10.0');
set_param(mdl, 'ScreenColor', 'white');

disp('-> Creando los 6 bloques básicos...');

% 1. Referencia de Ángulo Deseado (Seno de marcha: +-10 grados a 0.5 Hz)
safe_add_block('simulink/Sources/Sine Wave', [mdl, '/Referencia_Angulo'], ...
    'Amplitude', '10.0', 'Frequency', '2*pi*0.5', 'SampleTime', '0', ...
    'Position', [60, 95, 130, 145]);

% 2. Sumador de Error: e = Ref - Real
safe_add_block('simulink/Math Operations/Sum', [mdl, '/Sumador_Error'], ...
    'Inputs', '+-', ...
    'Position', [200, 105, 230, 135]);

% 3. Controlador Proporcional Kp
safe_add_block('simulink/Math Operations/Gain', [mdl, '/Controlador_Kp'], ...
    'Gain', '4.0', ...
    'Position', [290, 100, 350, 140]);

% 4. Saturación de Voltaje (+-12V del Driver)
safe_add_block('simulink/Discontinuities/Saturation', [mdl, '/Saturacion_12V'], ...
    'UpperLimit', '12.0', 'LowerLimit', '-12.0', ...
    'Position', [410, 100, 450, 140]);

% 5. Planta del Motor (Velocidad angular del husillo)
safe_add_block('simulink/Continuous/Transfer Fcn', [mdl, '/Motor_Husillo'], ...
    'Numerator', '[15.0]', 'Denominator', '[0.1 1]', ...
    'Position', [510, 95, 600, 145]);

% 6. Integrador (Velocidad -> Posición Angular del Tobillo)
safe_add_block('simulink/Continuous/Integrator', [mdl, '/Integrador_Angulo'], ...
    'InitialCondition', '0.0', ...
    'Position', [660, 105, 700, 135]);

% 7. Scope para ver las curvas (Entrada 1: Deseado, Entrada 2: Real)
safe_add_block('simulink/Sinks/Scope', [mdl, '/Scope_Seguimiento'], ...
    'NumInputPorts', '2', ...
    'Position', [780, 90, 820, 150]);

disp('-> Conectando líneas...');
% Línea de Referencia al Sumador y al Scope (para comparar)
safe_add_line(mdl, 'Referencia_Angulo/1', 'Sumador_Error/1');
safe_add_line(mdl, 'Referencia_Angulo/1', 'Scope_Seguimiento/1');

% Cadena de Control hacia el Motor
safe_add_line(mdl, 'Sumador_Error/1', 'Controlador_Kp/1');
safe_add_line(mdl, 'Controlador_Kp/1', 'Saturacion_12V/1');
safe_add_line(mdl, 'Saturacion_12V/1', 'Motor_Husillo/1');
safe_add_line(mdl, 'Motor_Husillo/1', 'Integrador_Angulo/1');

% Salida real al Scope
safe_add_line(mdl, 'Integrador_Angulo/1', 'Scope_Seguimiento/2');

% Retroalimentación directa al sumador (por debajo, totalmente limpia)
safe_add_line(mdl, 'Integrador_Angulo/1', 'Sumador_Error/2');

% Guardar y centrar
save_system(mdl);
open_system(mdl);

try
    set_param(mdl, 'ZoomFactor', 'FitSystem');
catch
end

disp('===================================================================');
disp('   MODELO SIMPLE LISTO Y FUNCIONAL                                 ');
disp('   Solo 6 bloques clásicos de control en lazo cerrado.             ');
disp('   Haz clic en "Run" y abre el Scope para ver las curvas.          ');
disp('===================================================================');

end

% --- Funciones Auxiliares ---
function safe_add_block(src, dst, varargin)
    try
        add_block(src, dst, varargin{:});
        disp(['   [OK] ', dst]);
    catch ME
        disp(['   [AVISO] ', dst, ' -> ', ME.message]);
    end
end

function safe_add_line(m, p1, p2)
    try
        add_line(m, p1, p2, 'autorouting', 'on');
    catch ME
        disp(['   [AVISO LÍNEA] ', p1, ' -> ', p2, ': ', ME.message]);
    end
end
