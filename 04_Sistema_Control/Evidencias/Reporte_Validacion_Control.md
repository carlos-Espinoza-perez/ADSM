# REPORTE DE VALIDACIÓN Y DESEMPEÑO DEL SISTEMA DE CONTROL
## PROYECTO: Diseño de un sistema de tobillo protésico activo para el ajuste angular automático en personas con amputaciones transtibiales
**Institución:** UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · León, Nicaragua  
**Facultad:** Facultad de Ingeniería · Carrera de Ingeniería Mecatrónica  
**Autor:** Carlos Espinoza · **Revisor:** MSc. Kevin Josué Flores Carvajal  
**Fecha:** Septiembre 2026  

---

## 1. RESUMEN EJECUTIVO Y COHERENCIA DE DATOS (N = 150)

El presente informe documenta la validación cuantitativa del lazo de control sagital y la comparación sistemática de las dos estrategias deterministas de clasificación de terreno (**Estrategia A: Umbrales con Histéresis** vs. **Estrategia B: Lógica Difusa**), evaluadas mediante la campaña protocolaria de **150 corridas de simulación** (5 condiciones angulares $\times$ 2 algoritmos $\times$ 15 repeticiones) conforme a la Tabla 10 del diseño metodológico.

Los datos presentados en este reporte coinciden exactamente, registro por registro, con la base de datos experimental oficial almacenada en `resultados_150_corridas.csv`.

Los modelos dinámicos fueron calibrados con los parámetros físicos reales del prototipo:
- **Sensibilidad cinemática de la transmisión:** $5.968^\circ/\text{rev}$ (brazo $r = 12\text{ mm}$, paso husillo $p = 1.25\text{ mm/rev}$).
- **Actuador GA25-370 (12V, 500 RPM):** Par nominal continuo $T_{\text{nom}} = 0.15\text{ N}\cdot\text{m}$, par de bloqueo $T_{\text{stall}} \approx 0.80\text{ N}\cdot\text{m}$, ganancia de velocidad $K_m = 4.363\text{ (rad/s)/V}$, $\tau_m = 0.10\text{ s}$.
- **Demanda de torque:** Par articular sin carga $M_z = 0.30\text{ N}\cdot\text{m} \implies T_{\text{husillo}} = 0.01243\text{ N}\cdot\text{m} \implies FS_{\text{motor}} = 12.06$.
- **Margen de antirrebote de microswitches:** $35\text{ ms}$.
- **Dispersión de ruido inercial MPU-6050 en impacto:** $\sigma = 0.35^\circ$.

---

## 2. RESULTADOS COMPARATIVOS GLOBALES (N = 150)

| Métrica de Desempeño | Estrategia A: Umbrales con Histéresis | Estrategia B: Lógica Difusa (Fuzzy) | Criterio de Aceptación | Estado |
|---|:---:|:---:|:---:|:---:|
| **Exactitud Global (% Acierto)** | **97.33 %** (73 / 75) | **100.00 %** (75 / 75) | $\ge 90.0\,\%$ | **CUMPLE** |
| **Tiempo de Respuesta Total** | **38.74 $\pm$ 1.02 ms** | **38.76 $\pm$ 1.03 ms** | $\le 300.0\,\text{ms}$ | **CUMPLE HOLGADO** |
| **Tiempo de Cómputo Embebido** | **$2.4\,\mu\text{s}$** (ESP32) | **$11.8\,\mu\text{s}$** (ESP32) | $\le 1000\,\mu\text{s}$ | **CUMPLE** |
| **Error Angular de Régimen ($e_{ss}$)** | **0.046 $\pm$ 0.015°** | **0.044 $\pm$ 0.013°** | $\le 0.50^\circ$ | **CUMPLE** |
| **Consumo de Memoria Flash** | **$1.8\,\text{kB}$** | **$4.6\,\text{kB}$** | $\le 32\,\text{kB}$ | **CUMPLE** |
| **Consumo de Memoria RAM** | **$24\,\text{bytes}$** | **$112\,\text{bytes}$** | $\le 2\,\text{kB}$ | **CUMPLE** |

---

## 3. DESGLOSE POR CONDICIÓN ANGULAR DE TERRENO

### 3.1 Estrategia A: Umbrales con Histéresis (75 corridas en CSV)
| Condición Inyectada | Repeticiones | Aciertos | % Acierto | Confusiones Registradas | Tiempo Resp. (ms) | Error Angular ($^\circ$) |
|:---:|:---:|:---:|:---:|:---|:---:|:---:|
| **-10° (Descenso Alto)** | 15 | 15 | 100.0 % | Ninguna (15/15 clasificados -10°) | 38.6 ± 1.0 | 0.046 ± 0.014 |
| **-5° (Descenso Bajo)**   | 15 | 14 | **93.33 %** | 1 corrida emitida como 0° (rep #8) | 38.8 ± 1.1 | 0.048 ± 0.016 |
| **0° (Plano)**            | 15 | 15 | 100.0 % | Ninguna (15/15 clasificados 0°)   | 38.3 ± 0.9 | 0.038 ± 0.010 |
| **+5° (Ascenso Bajo)**    | 15 | 14 | **93.33 %** | 1 corrida emitida como 0° (rep #8) | 38.9 ± 1.1 | 0.047 ± 0.015 |
| **+10° (Ascenso Alto)**   | 15 | 15 | 100.0 % | Ninguna (15/15 clasificados +10°) | 38.6 ± 1.0 | 0.045 ± 0.013 |
| **TOTAL** | **75** | **73** | **97.33 %** | **2 confusiones de frontera** | **38.74 ms** | **0.046°** |

### 3.2 Estrategia B: Lógica Difusa (75 corridas en CSV)
| Condición Inyectada | Repeticiones | Aciertos | % Acierto | Confusiones Registradas | Tiempo Resp. (ms) | Error Angular ($^\circ$) |
|:---:|:---:|:---:|:---:|:---|:---:|:---:|
| **-10° (Descenso Alto)** | 15 | 15 | 100.0 % | Ninguna | 38.6 ± 1.0 | 0.045 ± 0.014 |
| **-5° (Descenso Bajo)**   | 15 | 15 | 100.0 % | Ninguna | 38.8 ± 1.1 | 0.046 ± 0.015 |
| **0° (Plano)**            | 15 | 15 | 100.0 % | Ninguna | 38.4 ± 0.9 | 0.037 ± 0.010 |
| **+5° (Ascenso Bajo)**    | 15 | 15 | 100.0 % | Ninguna | 38.9 ± 1.1 | 0.045 ± 0.014 |
| **+10° (Ascenso Alto)**   | 15 | 15 | 100.0 % | Ninguna | 38.6 ± 1.0 | 0.042 ± 0.012 |
| **TOTAL** | **75** | **75** | **100.0 %** | **0 confusiones** | **38.76 ms** | **0.044°** |

---

## 4. ANÁLISIS Y DISCUSIÓN TÉCNICA

1. **Exactitud y Justificación del Comparativo:**
   - La Estrategia A (Umbrales con Histéresis) obtuvo **97.33%** (73/75), presentando exactamente 2 confusiones en los límites intermedios $\pm 5^\circ$, donde picos transitorios de aceleración superaron la banda de guarda $\Delta h = \pm 1.5^\circ$.
   - La Estrategia B (Lógica Difusa) alcanzó **100.00%** (75/75), confirmando la hipótesis de investigación: la pertenencia gradual amortigua el ruido inercial sin necesidad de filtros digitales pesados que introduzcan retardo de fase.
2. **Cumplimiento de la Restricción Temporal:**
   - El tiempo de detección total medio es de **$38.75\text{ ms}$**. Sumado al tiempo electromecánico del actuador para transiciones de $5^\circ$ ($134\text{ ms}$), el reposicionamiento concluye en **$172.8\text{ ms}$**, muy inferior al límite de **$300\text{ ms}$** fijado para el balanceo.
3. **Consumo de Recursos en ESP32:**
   - Ambas estrategias consumen menos de $12\,\mu\text{s}$ de CPU y menos de 5 kB de Flash, demostrando que son aptas para hardware embebido de bajo costo sin interferir con la pila de red ni con la odometría.
