# MEMORIA TÉCNICA DE ESTRATEGIAS DE CONTROL Y CLASIFICACIÓN DE TERRENO
## PROYECTO: Diseño de un sistema de tobillo protésico activo para el ajuste angular automático en personas con amputaciones transtibiales
**Institución:** UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA) · León, Nicaragua  
**Facultad:** Facultad de Ingeniería · Carrera de Ingeniería Mecatrónica  
**Autor:** Carlos Espinoza · **Revisor:** MSc. Kevin Josué Flores Carvajal  
**Fecha:** Septiembre 2026  

---

## 1. INTRODUCCIÓN Y ALCANCE METODOLÓGICO

El presente documento formaliza la arquitectura de control en lazo cerrado, la máquina de estados de marcha y la comparación cuantitativa de dos estrategias deterministas de clasificación de terreno para un prototipo escalado (1:2) de tobillo protésico activo.

Conforme a las directrices vigentes del docente revisor (18 de septiembre de 2026), el proyecto se centra en:
1. La modelación dinámica y el control en tiempo real de la articulación sagital.
2. La comparación del desempeño de dos algoritmos de clasificación de terreno embebidos: **Umbrales con Histéresis** vs. **Lógica Difusa (Fuzzy Logic)**.
3. La validación experimental mediante una campaña de 150 corridas de simulación con datos paramétricos calibrados contra hardware físico, complementada con la demostración funcional en vivo del prototipo ensamblado sobre bastidor.

---

## 2. CICLO OPERATIVO EN DOS ESTADOS Y CINEMÁTICA SAGITAL

### 2.1 Desacoplamiento Apoyo / Balanceo
En la biomecánica humana, el ciclo de marcha se divide en fase de apoyo (*stance*, ~60% del ciclo) y fase de balanceo (*swing*, ~40% del ciclo). En el prototipo de tobillo activo, la actuación electromecánica se desacopla rigurosamente entre ambos regímenes:

1. **Estado Cargado (Fase de Apoyo):**
   - El sistema sostiene el momento estructural máximo de volcamiento sagital ($M_z = 12.74\text{ N}\cdot\text{m}$, correspondiente a la carga de diseño de $196\text{ N}$ a $65\text{ mm}$ del eje).
   - **Sostenimiento Pasivo por Autobloqueo Mecánico:** La transmisión mediante husillo métrico M8 (ángulo de hélice $\lambda = 3.17^\circ$) y tuerca antibacklash presenta un ángulo de fricción efectivo $\varphi = 9.83^\circ$ para un coeficiente de rozamiento $\mu = 0.15$. Dado que $\lambda < \varphi$, el mecanismo es irreversible (no retroimpulsable). El motor GA25-370 permanece apagado ($PWM = 0$) durante el apoyo, eliminando el consumo de energía y el sobrecalentamiento.
2. **Estado Descargado (Fase de Balanceo):**
   - El pie pierde contacto con el suelo (detectado por la apertura simultánea de los microswitches de talón y punta).
   - El actuador se habilita para reposicionar la articulación al ángulo diana determinado por el clasificador de terreno.
   - **Par de reposicionamiento:** Masa del conjunto móvil $m \approx 150\text{ g}$, centro de masa a $35\text{ mm}$, momento gravitatorio $\approx 0.05\text{ N}\cdot\text{m}$. Considerando masa e inercia con factor de seguridad $FS = 5$, el par articular sagital de reposicionamiento descargado es $M_z = 0.30\text{ N}\cdot\text{m}$ ($300\text{ N}\cdot\text{mm}$). A través de la transmisión por husillo M8 ($r = 12\text{ mm}$, $p = 1.25\text{ mm/rev}$, rendimiento $\eta = 0.40$), la fuerza axial es $F_{\text{axial}} = 25.0\text{ N}$ y el par torsor requerido en el eje del motor es de solo $T_{\text{husillo}} = 0.01243\text{ N}\cdot\text{m}$ ($1.243\text{ N}\cdot\text{cm}$). El motorreductor GA25-370 12V 500 RPM presenta un **par nominal continuo $T_{\text{nom}} = 0.15\text{ N}\cdot\text{m}$** ($15\text{ N}\cdot\text{cm}$) y un **par de bloqueo (*stall*) $T_{\text{stall}} \approx 0.80\text{ N}\cdot\text{m}$** ($80\text{ N}\cdot\text{cm}$). Por consiguiente, el actuador opera con un factor de seguridad nominal continuo $FS_{\text{motor}} = \frac{T_{\text{nom}}}{T_{\text{husillo}}} = \frac{0.15}{0.01243} = 12.06$ (y un margen de bloqueo $FS_{\text{stall}} = 64.3$), garantizando reposicionamiento rápido sin fatiga térmica.
   - **Restricción Temporal:** El tiempo máximo admisible de reposicionamiento es de **$300\text{ ms}$** para el recorrido total de $30^\circ$ ($100^\circ/\text{s}$). Para transiciones entre clases adyacentes de $5^\circ$, el tiempo real de giro es de solo $\approx 134\text{ ms}$, garantizando un margen superior al 50%.

### 2.2 Relaciones Cinemáticas Exactas
Con un brazo de palanca cinemático $r = 12.00\text{ mm}$ y un husillo de avance $p = 1.25\text{ mm/rev}$:
$$\Delta y = -r \cdot \sin(\theta)$$
$$\theta = -\arcsin\left(\frac{\Delta y}{r}\right)$$
$$N_{\text{rev}} = \frac{r \cdot \sin(\theta)}{p} = 9.6 \cdot \sin(\theta)$$

- Carrera vertical útil de la tuerca para $\pm 15^\circ$: $\Delta y_{\text{total}} = 6.2117\text{ mm}$.
- Revoluciones requeridas: $4.969\text{ vueltas}$.
- Sensibilidad en neutro: $5.968^\circ/\text{rev}$ ($4.775^\circ/\text{mm}$).

---

## 3. COMPARACIÓN DE LAS DOS ESTRATEGIAS DE CLASIFICACIÓN

Ambos algoritmos reciben exactamente las mismas tres entradas sensoriales:
1. `pitch_tibial` ($\theta_{\text{tib}}$): Ángulo de inclinación medido por el MPU6050 y filtrado a $100\text{ Hz}$.
2. `sw_talon`: Estado binario del microswitch de talón ($1 = \text{contacto}$).
3. `sw_punta`: Estado binario del microswitch de punta ($1 = \text{contacto}$).

El universo de discurso se compone de 5 clases discretas de terreno:
- **Clase 1 (Descenso Alto):** $-10^\circ$
- **Clase 2 (Descenso Bajo):** $-5^\circ$
- **Clase 3 (Plano / Neutro):** $0^\circ$
- **Clase 4 (Ascenso Bajo):** $+5^\circ$
- **Clase 5 (Ascenso Alto):** $+10^\circ$

```
          Descenso Alto    Descenso Bajo        Plano        Ascenso Bajo     Ascenso Alto
             [-10°]            [-5°]             [0°]            [+5°]           [+10°]
  <------------|-----------------|----------------|----------------|----------------|------------>
             -10°              -5°               0°              +5°             +10°       θ_tibial
```

### 3.1 Estrategia A: Umbrales con Histéresis
La Estrategia A utiliza fronteras nítidas condicionadas por una banda de histéresis de $\Delta h = \pm 1.5^\circ$ centrada en los puntos medios entre clases ($\pm 2.5^\circ$ y $\pm 7.5^\circ$):

- Si el estado actual es **Plano ($0^\circ$)**:
  - Transiciona a **Ascenso Bajo ($+5^\circ$)** solo si $\theta_{\text{tib}} > +2.5^\circ + 1.5^\circ = +4.0^\circ$.
  - Transiciona a **Descenso Bajo ($-5^\circ$)** solo si $\theta_{\text{tib}} < -2.5^\circ - 1.5^\circ = -4.0^\circ$.
- Para retornar a **Plano ($0^\circ$)**:
  - Desde Ascenso Bajo: requiere $\theta_{\text{tib}} < +2.5^\circ - 1.5^\circ = +1.0^\circ$.
  - Desde Descenso Bajo: requiere $\theta_{\text{tib}} > -2.5^\circ + 1.5^\circ = -1.0^\circ$.

**Ventajas:** Algoritmo auditable, ejecución en $< 2\,\mu\text{s}$ en ESP32, nulo consumo de memoria RAM.  
**Limitación:** Comportamiento rígido en presencia de perturbaciones dinámicas sostenidas cerca del umbral.

### 3.2 Estrategia B: Lógica Difusa (Fuzzy Logic)
La Estrategia B emplea un Sistema de Inferencia Difusa (Mamdani/Sugeno orden 0) con 5 funciones de pertenencia triangulares y trapezoidales sobre $\theta_{\text{tib}}$:

1. $\mu_{\text{DA}}(\theta)$: Trapezoidal en $[-\infty, -10, -7.5, -5]$
2. $\mu_{\text{DB}}(\theta)$: Triangular en $[-7.5, -5, -2.5]$
3. $\mu_{\text{PL}}(\theta)$: Triangular en $[-2.5, 0, +2.5]$
4. $\mu_{\text{AB}}(\theta)$: Triangular en $[+2.5, +5, +7.5]$
5. $\mu_{\text{AA}}(\theta)$: Trapezoidal en $[+5, +7.5, +10, +\infty]$

**Reglas de Inferencia:**
1. Si `pitch` es DA y `sw_contacto` es ACTIVO $\to$ Terreno = $-10^\circ$.
2. Si `pitch` es DB y `sw_contacto` es ACTIVO $\to$ Terreno = $-5^\circ$.
3. Si `pitch` es PL y `sw_contacto` es ACTIVO $\to$ Terreno = $0^\circ$.
4. Si `pitch` es AB y `sw_contacto` es ACTIVO $\to$ Terreno = $+5^\circ$.
5. Si `pitch` es AA y `sw_contacto` es ACTIVO $\to$ Terreno = $+10^\circ$.

**Defuzzificación:** Método de la media de centros (Centroide / Sugeno Singleton) que produce una consigna continua $\theta_{\text{ref}}$, cuantizada a la clase discreta más próxima para el análisis comparativo formal.

**Ventajas:** Transiciones suaves, robustez frente al ruido de acelerómetro en impacto, pondera pertenencias compartidas en pendientes intermedias.

---

## 4. ARQUITECTURA DE CONTROL DE POSICIÓN Y REDUNDANCIA

### 4.1 Lazo Cerrado con Compensación de Zona Muerta
El lazo de control de posición angular se ejecuta en el ESP32 a una frecuencia de $100\text{ Hz}$ ($T_s = 10\text{ ms}$):
1. **Error de posición:** $e[k] = \theta_{\text{ref}}[k] - \theta_{\text{art}}[k]$.
2. **Controlador Proporcional:** $u_{\text{p}}[k] = K_p \cdot e[k]$, con ganancia $K_p = 4.0\text{ V}/^\circ$.
3. **Compensación de Zona Muerta (Deadband):** El puente H TB6612FNG y la fricción estática de la varilla M8 impiden el giro con ciclos de trabajo inferiores a $PWM \approx 65$ (sobre 255 a $12\text{ V}$, equivalente a $\approx 3.06\text{ V}$). La ley de control incorpora un salto directo:
   $$u[k] = \text{sgn}(e[k]) \cdot \left( V_{\text{dead}} + |u_{\text{p}}[k]| \right) \quad \text{para } |e[k]| > e_{\text{tol}}$$
   con $e_{\text{tol}} = 0.20^\circ$ (evitando chattering inducido por el juego de la rosca).
4. **Saturación:** $u_{\text{sat}}[k] = \text{clip}(u[k], -12.0, +12.0)\text{ V}$.

### 4.2 Verificación Cruzada y Alarma de Deslizamiento (Slip Alarm)
- **Realimentación Primaria (Odometría):** Se obtiene del contador de pulsos del encoder de cuadratura integrado en el motor (25.7 pulsos por grado articular). Permite una respuesta dinámica inmediata sin carga sobre el bus I2C.
- **Sensor Absoluto AS5600:** Montado sobre el eje del tobillo en cara lateral de A3. Mide directamente el ángulo real articulado $\theta_{\text{AS}}$ a 12 bits de resolución ($0.088^\circ$).
- **Detección de Deslizamiento (Slip Alarm):**
  $$|\theta_{\text{odo}} - \theta_{\text{AS}}| > \Delta\theta_{\text{max}} \quad (\Delta\theta_{\text{max}} = 2.0^\circ)$$
  Si el acople B7 o el prisionero del motor patina, o si se supera el juego admisible de tuerca antibacklash ($0.95^\circ$), el sistema dispara `Slip_Alarm`, cortando el pin `STBY` del TB6612FNG para proteger la integridad estructural.

---

## 5. CONCLUSIONES DE DISEÑO
La combinación de una transmisión mecánica autoblocante para absorber el choque de carga con un control proporcional asistido por compensación de fricción permite cumplir sobradamente el requerimiento de $300\text{ ms}$ en balanceo con un costo de manufactura mínimo, dejando la comparación centrada en la precisión y latencia de los dos clasificadores.
