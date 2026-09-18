# Prototipo Escalado de Tobillo Protésico Activo

Repositorio técnico correspondiente al análisis, diseño e implementación del prototipo mecatrónico de tobillo protésico activo de 1 Grado de Libertad (GDL) sagital, orientado al ajuste angular automático para asistencia a la marcha.

---

## Arquitectura General del Sistema

El sistema integra diseño mecánico paramétrico, acondicionamiento y distribución electrónica de potencia, y control embebido en tiempo real para el seguimiento de trayectorias angulares:

```
                      +-----------------------------+
                      |   Supervisión & Telemetría  |
                      |    (Node-RED / MQTT / WiFi) |
                      +--------------+--------------+
                                     |
                                     v
+------------------+     +--------------------+     +-------------------+
|     Sensores     |     |   Microcontrolador |     |      Actuación    |
| - AS5600 (I2C)   | --> |   ESP32-WROOM-32D  | --> | - Driver TB6612   |
| - MPU6050 (I2C)  |     |   Control PID 1 GDL|     | - Motor GA25-370  |
| - Celdas HX711   |     +--------------------+     | - Husillo M8      |
+------------------+                                +-------------------+
```

---

## Estructura de Subsistemas

El repositorio está organizado en tres subsistemas de ingeniería:

### 1. `01_Sistema_Mecanico`
Contiene la definición geométrica, analítica y dimensional del mecanismo de transmisión y soporte estructural:
- **`Calculos/`**: Memoria de cálculo cinemático, dimensionamiento de esfuerzos y verificación de pares de torsión.
- **`Modelos_CAD/`**: Archivos paramétricos de diseño en Autodesk Fusion 360 (`.f3d`), modelos exportados para manufactura (`.step`, `.stl`) y ensambles de referencia en SolidWorks.
- **`Planos/`**: Planos técnicos dimensionales normalizados en formato PDF, incluyendo vistas ortogonales, cortes y detalles de tolerancias para fabricación.
- **`Simulaciones/`**: Reporte técnico de barrido cinemático, análisis de desplazamientos y comprobación geométrica del rango angular ($\pm 15^\circ$).
- **`Evidencias/`**: Vistas de despiece, ensambles y capturas de verificación de ensamble.

### 2. `02_Sistema_Electronico`
Comprende el acondicionamiento eléctrico, potencia y esquemas de conexión:
- **`01_Arquitectura/`**: Diagramas de bloques del sistema electrónico y flujo de señales.
- **`02_Esquematico_Electrico/`**: Esquemas de conexionado general, reguladores y protecciones.
- **`03_Conexionado/`**: Diagrama de cableado punto a punto y tabla de conexionado.
- **`04_Alimentacion/`**: Dimensionamiento de consumo energético, baterías y distribución de líneas de 12V, 5V y 3.3V.
- **`05_Componentes/`**: Lista de materiales (BOM) y hojas de datos (*datasheets*) de componentes críticos.
- **`06_Control/`**: Asignación de pines (GPIO), modelos matemáticos en MATLAB/Simulink y rutinas de calibración.
- **`07_Pruebas/`**: Estructura para protocolos de prueba de continuidad, consumo y niveles lógicos.

### 3. `03_Sistema_Computacional`
Implementa el control en lazo cerrado, adquisición sensorial y telemetría:
- **`Firmware/`**: Código fuente modular para ESP32 (`codigo_final.ino`), incorporando el algoritmo de control PID, filtros digitales y pruebas unitarias de periféricos.
- **`Configuracion/`**: Parámetros de calibración angular persistentes (NVS) y mapa de tópicos MQTT.
- **`Interfaz/`**: Flujo para Node-RED y paneles de control telemétrico para monitoreo en tiempo real.
- **`Evidencias/`**: Registros de curvas de seguimiento angular, depuración de transitorios y calibración en banco.

---

## Especificaciones Técnicas

| Parámetro | Valor Nominal | Unidad |
|---|---|---|
| Grados de Libertad (GDL) | 1 (Sagital: flexión dorsal / plantar) | - |
| Rango Angular de Trabajo | $\pm 15.0$ | Grados ($^\circ$) |
| Mecanismo de Transmisión | Husillo métrico M8 con acople directo | Paso $1.25\text{ mm/rev}$ |
| Actuador Principal | GA25-370 (12 VDC, caja reductora) | Par nominal $\ge 0.15\text{ N}\cdot\text{m}$ |
| Sensor de Posición Angular | AS5600 magnético absoluto (I2C) | Resolución 12 bits ($0.088^\circ$) |
| Unidad de Procesamiento | ESP32 Dual Core @ 240 MHz | 520 KB SRAM |
| Protocolo de Telemetría | MQTT sobre red Wi-Fi (2.4 GHz) | JSON payload |
