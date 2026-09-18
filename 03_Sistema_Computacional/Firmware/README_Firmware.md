# Firmware del Prototipo de Tobillo Protésico

El firmware del sistema corre sobre un microcontrolador ESP32 DevKit v1. Administra la lectura de sensores de marcha, el control de lazo cerrado del motor de ajuste sagital, la máquina de estados del ciclo de marcha y la comunicación bidireccional por MQTT y Serial.

## Estructura de Códigos

1. **`codigo_final/codigo_final.ino`**: Firmware integral del sistema.
   - Control de posición por encoder incremental y mapeo no lineal en memoria NVS.
   - Verificación cruzada de ángulo con sensor magnético absoluto AS5600.
   - Detección de fases de marcha mediante microswitches (talón/punta) e inclinación tibial (MPU-6050).
   - Servidor cautivo WiFiManager para asignación de red y broker MQTT en campo.
   - Publicación y suscripción a tópicos de telemetría y comandos remotos.

2. **`Pruebas_Unitarias/`**: Códigos individuales de validación de hardware en banco.
   - `i2c_scanner.ino`: Verificación de direcciones I2C en bus compartido (MPU6050 en `0x68`, AS5600 en `0x36`).
   - `prueba_motor.ino`: Comprobación de sentido de giro y activación de driver TB6612FNG.
   - `prueba_velocidad_motor.ino`: Barrido de PWM para determinar zona muerta (arranque mínimo) y velocidad máxima con conteo de encoder.
   - `prueba_hx711_talon.ino`: Lectura cruda de celda de carga para fase de contacto de talón.
   - `prueba_todos_sensores.ino`: Comprobación conjunta de lectura simultánea de IMU, encoder magnético y celdas sin bloqueos en bus.

## Mapeo de Pines (GPIO)

| Periférico / Señal | Pin ESP32 | Descripción |
|---|---|---|
| I2C SDA | GPIO 21 | Bus de datos (MPU-6050, AS5600) |
| I2C SCL | GPIO 22 | Bus de reloj (MPU-6050, AS5600) |
| Motor PWM | GPIO 25 | Señal de modulación de velocidad a TB6612FNG |
| Motor IN1 | GPIO 26 | Control de dirección A |
| Motor IN2 | GPIO 27 | Control de dirección B |
| Encoder Canal A | GPIO 34 | Entrada de pulsos (requiere pull-up externo de 10 kΩ) |
| Encoder Canal B | GPIO 35 | Entrada de cuadratura (requiere pull-up externo de 10 kΩ) |
| Switch Talón | GPIO 32 | Entrada digital con debounce por software |
| Switch Punta | GPIO 33 | Entrada digital con debounce por software |
| HX711 Talón (DT / SCK) | GPIO 16 / 17 | Interfaz de celda de carga de apoyo posterior |
| HX711 Punta (DT / SCK) | GPIO 4 / 13 | Interfaz de celda de carga de despegue anterior |
| Botón Config Wi-Fi | GPIO 0 | Pulsador para forzar portal WiFiManager (3 s sostenido) |

## Librerías Requeridas en Arduino IDE

- `WiFiManager` (tzapu)
- `PubSubClient` (Nick O'Leary)
- `ArduinoJson` (Benoît Blanchon)
- `Adafruit MPU6050` y `Adafruit Unified Sensor`
- `AS5600` (Rob Tillaart)
- `HX711` (Bogdan Necula)
