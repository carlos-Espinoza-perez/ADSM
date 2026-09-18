# Firmware del Prototipo de Tobillo Protésico Activo

El firmware del sistema se ejecuta sobre un microcontrolador **ESP32-WROOM-32 DevKit v1**. Administra la adquisición sensorial de marcha, el control de lazo cerrado del motorreductor sagital, la máquina de estados de las fases de apoyo/balanceo y la comunicación bidireccional mediante telemetría MQTT y consola serie.

---

## Estructura de Códigos

1. **`codigo_final/codigo_final.ino`**: Firmware integral operativo del prototipo.
   - Control de posición angular por odometría de pulsos de encoder con tabla de 5 puntos e interpolación lineal almacenada en memoria NVS.
   - Verificación cruzada contra el encoder magnético absoluto AS5600 montado en el eje articular (detección de *Slip Alarm*).
   - Detección de contacto de apoyo mediante microswitches de talón y punta con filtro de antirrebote (*debounce*) de 35 ms.
   - Estimación de inclinación mediante MPU-6050 por lectura directa de registros I2C a 400 kHz.
   - Servidor cautivo WiFiManager para aprovisionamiento dinámico de red y configuración del broker MQTT en campo.
   - Publicación de telemetría y suscripción a consignas remotas.

2. **`Pruebas_Unitarias/`**: Códigos individuales de validación de hardware en banco de laboratorio.
   - `i2c_scanner.ino`: Verificación de direcciones en bus I2C compartido (MPU-6050 en `0x68`, AS5600 en `0x36`).
   - `prueba_motor.ino`: Comprobación de sentido de giro y polarización del driver TB6612FNG con canales en paralelo.
   - `prueba_velocidad_motor.ino`: Barrido de PWM para determinar zona muerta mecánica (arranque mínimo a PWM ≈ 65) y conteo de encoder.
   - `prueba_todos_sensores.ino`: Comprobación conjunta de lectura simultánea en bus I2C y entradas digitales.
   - `prueba_hx711_talon.ino`: Código instrumental previo de caracterización de fuerza en banco de ensayos.

---

## Mapeo Oficial de Pines (ESP32-WROOM-32)

Mapeo rigurosamente estandarizado y verificado contra `codigo_final.ino`, el plano esquemático `Esquematico_Principal.pdf` y la tabla maestra de GPIO:

| Pin ESP32 | Nombre Firmware | Tipo I/O | Dispositivo Destino | Nivel Lógico | Polarización / Función |
|:---:|:---:|:---:|:---:|:---:|:---|
| **GPIO 0** | `BOTON_RESET` | Entrada Digital | Pulsador BOOT en placa | 3.3 V | Pull-up interna. Reset de WiFi y NVS al pulsar > 3 s |
| **GPIO 14** | `PIN_STBY` | Salida Digital | TB6612FNG (Pin STBY) | 3.3 V | Conexión directa. HIGH = Habilitado, LOW = Standby |
| **GPIO 21** | `SDA_PIN` | I2C Bidireccional | Bus I2C compartido | 3.3 V | Pull-up en módulos. SDA para MPU-6050 (`0x68`) y AS5600 (`0x36`) |
| **GPIO 22** | `SCL_PIN` | I2C Salida Reloj | Bus I2C compartido | 3.3 V | Pull-up en módulos. SCL a 400 kHz |
| **GPIO 25** | `PIN_AIN1` | Salida Digital | TB6612FNG (AIN1 + BIN1) | 3.3 V | Conexión directa. Dirección A (canales A y B en paralelo) |
| **GPIO 26** | `PIN_AIN2` | Salida Digital | TB6612FNG (AIN2 + BIN2) | 3.3 V | Conexión directa. Dirección B (canales A y B en paralelo) |
| **GPIO 27** | `PIN_PWM` | Salida PWM | TB6612FNG (PWMA + PWMB) | 3.3 V | Modulación LEDC a 20 kHz, resolución 8 bits (duty 0-255) |
| **GPIO 32** | `PIN_TALON` | Entrada Digital | Microswitch Talón | 3.3 V | Pull-up interna. Detección de contacto de talón (cierre a GND) |
| **GPIO 33** | `PIN_PUNTA` | Entrada Digital | Microswitch Punta | 3.3 V | Pull-up interna. Detección de apoyo de antepié (cierre a GND) |
| **GPIO 34** | `PIN_ENC_A` | Entrada Interrupción| Encoder GA25-370 Canal A | 3.3 V | **Pull-up externa de 10 kΩ a 3.3V** (pin de entrada pura) |
| **GPIO 35** | `PIN_ENC_B` | Entrada Digital | Encoder GA25-370 Canal B | 3.3 V | **Pull-up externa de 10 kΩ a 3.3V** (pin de entrada pura) |

> **Nota sobre pines de banco instrumental previo:**  
> Los pines GPIO 16, 17, 4 y 13 se emplearon en etapas preliminares de laboratorio para lectura de módulos HX711 en ensayos de fuerza estática. En la arquitectura operativa final del prototipo, el sensado de apoyo se ejecuta exclusivamente mediante los microswitches en GPIO 32 y GPIO 33.

---

## Librerías Requeridas en Arduino IDE

- `WiFiManager` (por tzapu, v2.0.16 o superior)
- `PubSubClient` (por Nick O'Leary, v2.8 o superior)
- `ArduinoJson` (por Benoît Blanchon, v6.21 o superior)
- `Preferences` (Librería nativa del ESP32 Core de Espressif)
- `Wire` (Librería nativa para comunicación I2C por hardware)

*Nota de diseño:* Los sensores MPU-6050 y AS5600 se leen directamente mediante accesos a registros por `Wire.h`, prescindiendo de librerías de terceros con bucles bloqueantes para minimizar la latencia de interrupción en el lazo de control.
