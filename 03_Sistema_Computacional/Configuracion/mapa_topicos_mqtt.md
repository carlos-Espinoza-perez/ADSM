# Protocolo de Comunicación y Mapa de Tópicos MQTT

La comunicación entre el microcontrolador ESP32 y el panel de supervisión (Node-RED) se realiza bajo el protocolo MQTT (puerto 1883).

## Parámetros de Red

- **Client ID ESP32**: `tobilloESP32`
- **Puerto**: `1883`
- **Reconexión automática**: Cada 5000 ms ante pérdida de enlace.
- **Tamaño de buffer MQTT**: 2560 bytes.
- **Configuración de IP de Broker**: Asignable dinámicamente mediante el portal cautivo WiFiManager (GPIO 0).

## Tabla de Tópicos

| Tópico | Dirección | Tipo | Descripción | Formato / Ejemplo |
|---|---|---|---|---|
| `tobillo/estado` | ESP32 → Node-RED | Telemetría / LWT | Estado de conexión del dispositivo. Incluye mensaje LWT en caso de desconexión abrupta. | `"conectado"` / `"desconectado"` |
| `tobillo/comando` | Node-RED → ESP32 | Control | Consignas operativas, reinicio, selección de modo o calibración. | `"caminar:15.0"`, `"calibrar"`, `"?"` |
| `tobillo/terreno` | Bidireccional | Datos | Inyección de ángulo de inclinación de terreno o reporte del terreno estimado por IMU. | Valor numérico en grados (ej: `15.0`) |
| `tobillo/log` | ESP32 → Node-RED | Diagnóstico | Espejo de la consola serial para monitoreo remoto de subfases y variables de control. | Texto estructurado / JSON con porcentaje de marcha y setpoint |
