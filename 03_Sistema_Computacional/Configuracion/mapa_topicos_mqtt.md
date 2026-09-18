# Protocolo de Comunicación y Mapa de Tópicos MQTT

La comunicación entre el microcontrolador ESP32 y el panel de supervisión (Node-RED) se realiza bajo el protocolo MQTT (puerto estándar 1883).

---

## Parámetros de Red

- **Client ID ESP32**: `tobilloESP32`
- **Puerto**: `1883`
- **Reconexión automática**: Cada 5000 ms ante pérdida de enlace.
- **Tamaño de buffer MQTT**: 2560 bytes.
- **Configuración de IP de Broker**: Asignable dinámicamente mediante el portal cautivo WiFiManager (pulsador BOOT en GPIO 0 por > 3 s).

---

## Tabla Oficial de Tópicos (Firmware v6)

Mapeo de tópicos MQTT sincronizado con las constantes `#define TOPIC_*` de `codigo_final.ino`:

| Tópico MQTT | Dirección | Tipo | Descripción | Formato / Ejemplo |
|---|:---:|:---:|---|---|
| `tobillo/estado` | ESP32 → Node-RED | LWT / Estado | Estado de conexión del dispositivo y mensaje Last Will and Testament. | `"conectado"` / `"desconectado"` |
| `tobillo/comando` | Node-RED → ESP32 | Control | Consignas operativas, reinicio, selección de modo o calibración NVS. | `"caminar:15.0"`, `"calibrar"`, `"histeresis"`, `"fuzzy"` |
| `tobillo/terreno` | Bidireccional | Datos | Inyección de ángulo de inclinación de terreno o reporte de clase estimada por el clasificador activo. | Valor numérico en grados (ej: `5.0`, `-10.0`) |
| `tobillo/telemetria` | ESP32 → Node-RED | Telemetría | Paquete JSON en tiempo real con ángulo real, setpoint, PWM aplicado y estado de switches. | `{"ang": 4.8, "sp": 5.0, "pwm": 120, "swT": 1, "swP": 0}` |
| `tobillo/ciclo` | ESP32 → Node-RED | Marcha | Subfase actual del ciclo de marcha y porcentaje de progresión del paso. | `{"fase": "APOYO_PLANO", "pct": 35.0}` |
| `tobillo/paso` | ESP32 → Node-RED | Eventos | Contador de pasos completados e indicador de cambio de estado de apoyo. | `{"paso": 42, "cadencia_spm": 54.0}` |
| `tobillo/log` | ESP32 → Node-RED | Diagnóstico | Espejo de la consola serial para monitoreo remoto y depuración de eventos críticos. | Texto estructurado / Alarmas (ej: `Slip_Alarm`) |
