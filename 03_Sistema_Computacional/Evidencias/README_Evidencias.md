# Registro de Evidencias y Bitácora de Depuración

Este directorio recopila las evidencias de pruebas experimentales, fallas detectadas en banco y su proceso de corrección durante el desarrollo del sistema computacional y de control del tobillo protésico activo.

## Bitácora de Pruebas y Depuración

### 1. Falla de Error Estacionario y Acumulación Angular
- **Archivo**: `Fallas_y_Depuracion/01_falla_error_estacionario_desfase_6grados.png`
- **Problema detectado**: Al ejecutar secuencias continuas de marcha en modo simulación de banco, la articulación no retornaba al neutro ($0^\circ$) al concluir el ciclo, acumulando un desfase progresivo de hasta $+6^\circ$.
- **Causa raíz**: Integración de error en el lazo de posición e histéresis acumulativa entre pasos sucesivos sin forzar el reset de posición neutra al final del apoyo terminal/balanceo.
- **Solución implementada**: Modificación en la máquina de estados del firmware (v6) para forzar explícitamente el setpoint a $0.0^\circ$ con temporizador de asentamiento y reinicio de variables integrales al finalizar cada ciclo de caminata.

### 2. Telemetría y Seguimiento por Subfases
- **Archivo**: `Fallas_y_Depuracion/02_depuracion_telemetria_subfases_marcha.png`
- **Problema analizado**: Monitoreo de discrepancias entre el ángulo comandado (`sp`) y el medido (`act`) durante la transición de apoyo medio a apoyo terminal.
- **Causa raíz**: Retardo temporal de la transmisión mecánica (husillo-tuerca M8 y holgura del acople) requería ajuste en la modulación PWM para mantener velocidad de respuesta dinámica.
- **Solución implementada**: Calibración de ganancias de velocidad y ajuste de la amplitud del ciclo de paso a $0.80$ para evitar saturación del driver TB6612FNG.

### 3. Validación de Curva de Seguimiento Angular
- **Archivo**: `Fallas_y_Depuracion/03_validacion_curva_seguimiento_angular.png`
- **Registro**: Curva experimental de trayectoria sagital en el visualizador gráfico del dashboard, contrastando el setpoint con la realimentación por encoder en tiempo real.

### 4. Depuración y Reorganización de la Interfaz HMI (Node-RED)
- **Archivos**:
  - `Fallas_y_Depuracion/04_ajuste_panel_calibracion_y_control.png`
  - `Fallas_y_Depuracion/05_optimizacion_distribucion_botones_diagnostico.png`
  - `Fallas_y_Depuracion/06_flujo_nodered_inicial_previo_ordenamiento.png`
- **Mejora**: Reestructuración del flujo de Node-RED para desacoplar la capa de comunicación MQTT de la capa visual de botones y gráficas, alineando las dimensiones de los paneles para operación táctil o de banco en PC.
