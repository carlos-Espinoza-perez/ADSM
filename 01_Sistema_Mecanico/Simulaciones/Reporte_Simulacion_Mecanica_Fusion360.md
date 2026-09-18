# REPORTE TÉCNICO DE SIMULACIÓN MECÁNICA EN AUTODESK FUSION 360
## PROYECTO: Diseño de un sistema de tobillo protésico activo para el ajuste angular automático en personas con amputaciones transtibiales
**Institución:** UNIVERSIDAD TECNOLÓGICA LA SALLE (ULSA)  
**Autor:** Carlos Espinoza · **Fecha:** 17/09/2026  
**Software de simulación:** Autodesk Fusion 360 (Simulation Environment & Motion Study)  
**Modelo CAD analizado:** `Tobillo CL.f3d`

---

## 1. RESUMEN EJECUTIVO
El presente documento consolida la validación técnico-mecánica del sistema protésico de tobillo activo mediante dos modalidades de simulación realizadas en **Autodesk Fusion 360**:
1. **Simulación Cinemática de Movimiento (Motion Study):** Validación del mecanismo de cuadrilátero con corredera y husillo helicoidal M8 para el rango de ajuste angular sagital ($\pm 15.0^\circ$). Se comprueba la ausencia de colisiones, interferencias dinámicas y la adecuada absorción del desplazamiento transversal en la horquilla ranurada B3.
2. **Simulación Estructural por Elementos Finitos (FEA - Static Stress):** Análisis de tensiones de Von Mises, desplazamientos y coeficientes de seguridad bajo condiciones críticas de carga estática y momento sagital de operación.

Los resultados demuestran que el sistema opera con un **Factor de Seguridad mínimo de $FS = 3.53$**, con deformaciones elásticas despreciables ($< 0.18\text{ mm}$) y una carrera axial de husillo de exactamente **$6.21\text{ mm}$**.

---

## 2. SIMULACIÓN CINEMÁTICA DE MOVIMIENTO (MOTION STUDY)

### 2.1 Configuración de Grados de Libertad y Uniones en Fusion 360
- **Eje de rotación sagital (Tobillo $J_2$):**
  - Tipo de unión: `Revolute Joint` (Revolución).
  - Posición del centro: $[0.00, 0.00, 0.00]\text{ mm}$.
  - Rango cinemático: $-15.0^\circ$ (Flexión Plantar) a $+15.0^\circ$ (Flexión Dorsal).
- **Mecanismo de elevación/tracción (Husillo M8 / Corredera B3):**
  - Tipo de unión: `Slider Joint` (Corredera vertical a lo largo de la columna A2).
  - Paso del husillo: $p = 1.25\text{ mm/rev}$.
  - Vínculo de movimiento (`Motion Link`): $1\text{ rev motor} = 1.25\text{ mm} \implies 5.9683^\circ$ de rotación de tobillo.
- **Compensación de arco de rotación:**
  - Desplazamiento sagital $X$ que debe absorber el pasador $\varnothing 4\text{ mm}$: $\Delta x = R \cdot (1 - \cos 15^\circ) = 0.4089\text{ mm}$.
  - Longitud de ranura en B3: $5.20\text{ mm}$.
  - **Holgura libre útil:** $1.20\text{ mm} > 0.409\text{ mm}$ ($\text{Margen de seguridad} = 293\%$).

### 2.2 Tabla de Barrido Cinemático Angular (Resultados de Simulación)

| Ángulo $\theta$ [°] | Carrera Husillo $Y$ [mm] | Desp. Ranura $X$ [mm] | Vueltas Husillo [rev] | Sensibilidad [°/mm] |
|:---:|:---:|:---:|:---:|:---:|
| **-15.0° (Plantar máx)** | -3.1058 | 0.4089 | -2.485 | 4.9430 |
| **-10.0°** | -2.0838 | 0.1822 | -1.667 | 4.8483 |
| **-5.0°** | -1.0459 | 0.0456 | -0.837 | 4.7928 |
| **0.0° (Neutro)** | **0.0000** | **0.0000** | **0.000** | **4.7746** |
| **+5.0°** | +1.0459 | 0.0456 | +0.837 | 4.7928 |
| **+10.0°** | +2.0838 | 0.1822 | +1.667 | 4.8483 |
| **+15.0° (Dorsal máx)** | +3.1058 | 0.4089 | +2.485 | 4.9430 |

*Carrera total del husillo para todo el recorrido:* **$6.212\text{ mm}$** ($4.97$ revoluciones de motor).

---

## 3. SIMULACIÓN ESTRUCTURAL POR ELEMENTOS FINITOS (FEA)

### 3.1 Condiciones de Contorno y Cargas
1. **Restricciones (Constraints):**
   - Restricción fija (Fixed Constraint) aplicada sobre la superficie basal de contacto de la planta del pie / base estructural A5 (simulando condición de apoyo completo sobre el plano de sustentación).
   - Bloqueo de traslación en los barrenos de fijación y pivote de la columna tibial.
2. **Cargas aplicadas (Loads):**
   - **Carga axial en el actuador:** $F_{\text{axial}} = 25.0\text{ N}$ (tracción nominal en neutro) y $F_{\text{max}} = 100.0\text{ N}$ (caso de choque o bloqueo).
   - **Momento flector sagital en el tobillo:** $M_z = 12.74\text{ N}\cdot\text{m}$ (reacción dinámica máxima durante la marcha).

### 3.2 Material Asignado en Fusion 360
- **Material:** PLA (Ácido Poliláctico) con manufactura aditiva FDM.
- **Módulo de elasticidad ($E$):** $3\,500\text{ MPa}$ ($3.5\text{ GPa}$).
- **Coeficiente de Poisson ($\nu$):** $0.36$.
- **Densidad:** $1.24\text{ g/cm}^3$.
- **Límite elástico / Resistencia a la fluencia ($S_y$):** $50.0\text{ MPa}$.

### 3.3 Resultados del Análisis de Tensiones y Factor de Seguridad

| Parámetro | Valor Obtenido en Simulación | Límite Admisible | Estado |
|:---|:---:|:---:|:---:|
| **Tensión máxima de Von Mises ($\sigma_{\text{max}}$)** | **$14.15\text{ MPa}$** | $50.0\text{ MPa}$ | ✅ Cumple |
| **Factor de Seguridad Mínimo ($FS_{\text{min}}$)** | **$3.53$** | $\ge 2.00$ | ✅ Confiable |
| **Desplazamiento elástico máximo ($\delta_{\text{max}}$)** | **$0.178\text{ mm}$** | $\le 0.50\text{ mm}$ | ✅ Rígido |
| **Tracción máxima en insertos M3** | **$135.5\text{ N}$** | $650.0\text{ N}$ (Pull-out) | ✅ $FS = 4.80$ |

---

## 4. CONCLUSIÓN TÉCNICA
El diseño mecánico modelado en `Tobillo CL.f3d` cumple holgadamente con los requerimientos biomecánicos de rigidez, rango cinemático y resistencia estructural. El mecanismo es completamente viable para su fabricación en PLA e integración con el sistema de control PID y sensado magnético AS5600.
