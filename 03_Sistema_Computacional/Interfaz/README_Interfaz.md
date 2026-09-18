# Interfaz de Monitoreo y Control (Node-RED)

La interfaz de supervisión, telemetría y diagnóstico del prototipo de tobillo protésico está implementada en Node-RED utilizando la suite `@flowfuse/node-red-dashboard` (Dashboard 2.0) y un broker MQTT local (puerto 1883).

## Archivos Incluidos

- **`flujo_nodered_tobillo.json`**: Exportación completa de los 139 nodos que componen la lógica de comunicación, enrutamiento JSON y generación de vistas.
- **`captura_flujo_nodered.png`**: Captura panorámica del lienzo de trabajo en el editor de Node-RED (desde la recepción/salida de comandos hasta la telemetría, gráficas y control manual).
- **`captura_dashboard_telemetria.png`**: Captura de página completa de la vista de monitoreo en tiempo real (`/dashboard/tobillo`).
- **`captura_dashboard_control_calibracion.png`**: Captura de página completa del panel de control, calibración NVS y consola de diagnóstico interactiva (`/dashboard/control`).

## Módulos y Páginas del Dashboard

### 1. Telemetría y Monitoreo (`/dashboard/tobillo`)
- **Conectividad y Estado**: Estado del enlace MQTT con el ESP32, clasificador activo (Histéresis o Lógica Difusa), estado de memoria NVS y detección de bloqueos mecánicos.
- **Marcha y Paso Simulado**: Sliders interactivos para modificar en vivo la inclinación de terreno objetivo, duración de ciclo (ms) y amplitud de paso; botones de ejecución individual, marcha continua o parada de emergencia.
- **Gráficas Temporales en Vivo**:
  1. Trayectoria de ángulo: setpoint vs. ángulo real articular.
  2. Esfuerzo de control: modulación PWM hacia el driver TB6612FNG.
  3. Relación multicomponente: ángulo, velocidad angular (°/s) y pulsos de encoder.
- **Sensores y Articulación**: Diales analógicos de ángulo articular, inclinación tibial (MPU-6050), estimación de terreno y verificación con encoder magnético AS5600.

### 2. Control y Calibración (`/dashboard/control`)
- **Consola Serial y MQTT**: Terminal con log en vivo de las 8 subfases de marcha, errores de seguimiento y tiempos de asentamiento.
- **Diagnóstico Rápido**: Botones directos para solicitar estado general, volcado de tabla NVS, prueba de ruido y diagnóstico AS5600.
- **Control de Movimiento Manual**: Movimientos paso a paso (Jog $\pm 2^\circ$, $\pm 1^\circ$), consignas rápidas ($-15^\circ$, $0^\circ$, $+15^\circ$) y velocidad de posicionamiento.
- **Calibración de Posición**: Ajuste y fijación de los 5 puntos sagitales ($-30^\circ$, $-15^\circ$, $0^\circ$, $+15^\circ$, $+30^\circ$) con persistencia directa en la memoria flash del microcontrolador.
- **Parámetros NVS**: Ajuste de PWM mínimo de arranque y ganancia proporcional $K_p$.
