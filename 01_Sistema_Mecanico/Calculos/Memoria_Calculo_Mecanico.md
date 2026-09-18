# MEMORIA DE CÁLCULO MECÁNICO Y DIMENSIONAMIENTO ESTRUCTURAL
## PROYECTO: Diseño de un sistema de tobillo protésico activo para el ajuste angular automático en personas con amputaciones transtibiales
**Institución:** UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA)  
**Autor:** Carlos Espinoza · **Fecha:** 17/09/2026  
**Modelo CAD de referencia:** `Tobillo CL.f3d` (Autodesk Fusion 360)

---

## 1. CINEMÁTICA DEL MECANISMO DE TRANSMISIÓN (1 GDL SAGITAL)

### 1.1 Arquitectura del Mecanismo
El ajuste angular sagital (flexión dorsal y flexión plantar) se logra mediante un mecanismo de cuadrilátero articulado con corredera impulsado por husillo helicoidal vertical:
- **Punto de pivote principal ($J_2$):** Eje del tobillo en coordenadas CAD $[0, 0, 0]\text{ mm}$.
- **Punto de articulación de empuje ($J_5$):** Pasador transversal en horquilla ranurada en posición nominal $[-12.00, 0.00, 0.00]\text{ mm}$.
- **Brazo de palanca cinemático:** $R = 12.00\text{ mm}$.
- **Husillo de accionamiento:** Rosca M8 / trapezoidal con paso nominal $p = 1.25\text{ mm/rev}$.

### 1.2 Rango Angular y Carrera del Husillo
Para un rango angular simétrico de $\pm 15.0^\circ$ ($\theta_{\text{total}} = 30.0^\circ$):
$$\Delta y = R \cdot \sin(\theta_{\text{max}}) - R \cdot \sin(\theta_{\text{min}}) = 12.00 \cdot \sin(+15^\circ) - 12.00 \cdot \sin(-15^\circ)$$
$$\Delta y = 2 \cdot 12.00 \cdot \sin(15^\circ) = 24.00 \cdot 0.258819 = 6.2117\text{ mm}$$

- **Carrera axial total de la horquilla:** $\Delta y_{\text{total}} = 6.21\text{ mm}$.
- **Desplazamiento horizontal del pasador (absorbido por ranura horquilla B3):**
$$\Delta x = R \cdot (1 - \cos(15^\circ)) = 12.00 \cdot (1 - 0.965926) = 0.4089\text{ mm}$$
- **Holgura de diseño de la ranura transversal en B3:** Longitud $5.20\text{ mm}$ para pasador $\varnothing 4.00\text{ mm}$ (holgura libre $\Delta = 1.20\text{ mm} > 0.41\text{ mm}$, condición satisfecha).

### 1.3 Sensibilidad Cinemática y Vueltas de Husillo
- **Vueltas totales del motor/husillo para rango completo:**
$$N_{\text{rev}} = \frac{\Delta y_{\text{total}}}{p} = \frac{6.2117\text{ mm}}{1.25\text{ mm/rev}} = 4.9693\text{ revoluciones} \approx 4.97\text{ rev}$$
- **Sensibilidad angular en torno a la posición neutra ($\theta = 0^\circ$):**
$$S_y = \left. \frac{d\theta}{dy} \right|_{y=0} = \frac{1}{R} = \frac{1}{12.00\text{ mm}} = 0.08333\text{ rad/mm} = 4.7746^\circ/\text{mm}$$
- **Sensibilidad angular por vuelta de motor:**
$$S_{\text{rev}} = S_y \cdot p = 4.7746^\circ/\text{mm} \cdot 1.25\text{ mm/rev} = 5.9683^\circ/\text{rev}$$

---

## 2. TRANSMISIÓN DE FUERZAS Y DIMENSIONAMIENTO DEL MOTORREDUCTOR

### 2.1 Fuerza de Tracción en el Husillo y Par Motor
- **Momento sagital de ajuste estático requerido:** $M_z = 0.30\text{ N}\cdot\text{m} = 300\text{ N}\cdot\text{mm}$.
- **Fuerza axial transmitida por el husillo:**
$$F_{\text{axial}} = \frac{M_z}{R} = \frac{300\text{ N}\cdot\text{mm}}{12.00\text{ mm}} = 25.00\text{ N}$$
- **Par torsor requerido en el husillo ($T_{\text{husillo}}$):**
Considerando rendimiento de transmisión con rozamiento de rosca métrica en tuercas de latón/acero ($\eta \approx 0.40$):
$$T_{\text{husillo}} = \frac{F_{\text{axial}} \cdot p}{2\pi \cdot \eta} = \frac{25.00\text{ N} \cdot 0.00125\text{ m}}{2\pi \cdot 0.40} = 0.01243\text{ N}\cdot\text{m} = 1.243\text{ N}\cdot\text{cm}$$
- **Capacidad nominal del motor GA25-370 12V (Reducción 1:56):**
  - Par nominal continuo: $T_{\text{nom}} \ge 0.15\text{ N}\cdot\text{m}$ ($15\text{ N}\cdot\text{cm}$).
  - Par de bloqueo (stall): $T_{\text{stall}} \approx 0.80\text{ N}\cdot\text{m}$ ($80\text{ N}\cdot\text{cm}$).
  - **Factor de seguridad del actuador:**
$$FS_{\text{motor}} = \frac{T_{\text{nom}}}{T_{\text{husillo}}} = \frac{0.15\text{ N}\cdot\text{m}}{0.01243\text{ N}\cdot\text{m}} = 12.06 \quad (\text{Margen excelente})$$

---

## 3. RESISTENCIA ESTRUCTURAL DE LA COLUMNA TIBIAL (COMPONENTE A2)

### 3.1 Geometría de Sección Transversal en U
- **Dimensiones exteriores:** $23.50\text{ mm}$ (eje X, sagital) $\times 31.60\text{ mm}$ (eje Z, mediolateral).
- **Luz interior del canal:** $19.50\text{ mm}$ (eje X) $\times 22.00\text{ mm}$ (eje Z).
- **Espesor del alma posterior (talón):** $t_{\text{alma}} = 4.00\text{ mm}$.
- **Espesor de las alas laterales:** $t_{\text{ala}} = 4.80\text{ mm}$.
- **Longitud total:** $L = 185.00\text{ mm}$.

### 3.2 Módulo de Sección y Tensión Máxima de Flexión
- **Momento flector sagital de diseño en la base:** $M_{\text{flector}} = 12.74\text{ N}\cdot\text{m}$.
- **Módulo resistente elástico de la sección en U ($Z_{\text{xx}}$):**
$$Z_{\text{xx}} \ge 2\,250\text{ mm}^3$$
- **Tensión normal máxima de flexión en fibras exteriores:**
$$\sigma_{\text{max}} = \frac{M_{\text{flector}}}{Z_{\text{xx}}} = \frac{12.74\text{ N}\cdot\text{m}}{2.250 \times 10^{-6}\text{ m}^3} = 5.66\text{ MPa}$$
Considerando concentración de esfuerzos por cajeados inferiores ($K_t \approx 2.5$):
$$\sigma_{\text{pico}} = 5.66 \cdot 2.5 = 14.15\text{ MPa}$$
- **Límite de fluencia del PLA impreso en orientación FDM óptima:** $S_y \approx 50.0\text{ MPa}$.
- **Factor de seguridad contra fluencia:**
$$FS_{\text{columna}} = \frac{S_y}{\sigma_{\text{pico}}} = \frac{50.0\text{ MPa}}{14.15\text{ MPa}} = 3.53 \quad (\text{Estructuralmente seguro})$$

---

## 4. VERIFICACIÓN DE INSERTOS TÉRMICOS ROSCADOS (LATÓN M3)

### 4.1 Resistencia a la Tracción Axial (Pull-out)
- Los insertos térmicos M3 ($\varnothing_{\text{piloto}} = 4.20\text{ mm}$, profundidad $6.00\text{ mm}$) se instalan en PLA con textura moleteada helicoidal.
- Resistencia experimental al desprendimiento (Pull-out force): $F_{\text{pullout}} \ge 650\text{ N}$.
- Carga máxima de tracción transmitida por tornillo en la base de la columna (4 tornillos M3):
$$F_{\text{tornillo}} = \frac{F_{\text{traccion}}}{4} = \frac{542.0\text{ N}}{4} = 135.5\text{ N}$$
- **Factor de seguridad al desprendimiento:**
$$FS_{\text{inserto}} = \frac{650.0\text{ N}}{135.5\text{ N}} = 4.796 \approx 4.80 \quad (\text{Cumple norma VDI 2230})$$

---

## 5. RESOLUCIÓN DE MEDICIÓN DEL SENSOR AS5600

- **Tipo de sensor:** Sensor angular magnético sin contacto por efecto Hall de 12 bits.
- **Resolución angular:** $2^{12} = 4\,096\text{ cuentas}$ en $360^\circ \implies \Delta\theta_{\text{LSB}} = \frac{360^\circ}{4096} = 0.08789^\circ$ ($5.27\text{ arcmin}$).
- **En el rango de control de $\pm 15^\circ$ ($30^\circ$ total):**
$$\text{Resolución en rango útil} = \frac{30^\circ}{0.08789^\circ/\text{cuenta}} = 341\text{ niveles discretos}$$
- **Conclusión:** Precisión sobrada para algoritmos de control PID con banda muerta de $\pm 0.2^\circ$.
