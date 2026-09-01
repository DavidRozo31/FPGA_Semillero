# Verificación Cruzada: Método Geométrico (FPGA) vs Matrices Homogéneas (MTH)

## Resumen

Las lecciones 11-13 implementan la cinemática directa del brazo con el **método geométrico**
(fórmulas cerradas, elegido para ahorrar recursos lógicos — ver
[lección 11](../11_Cinematica_Directa_Geometrica/README.md)). Esta lección responde una
pregunta distinta: *¿el método geométrico da el mismo resultado que el método clásico y
riguroso de matrices de transformación homogénea (MTH) 4×4?*

Se toman los 3 casos de prueba ya validados en la FPGA ([lección 11, sección 7](../11_Cinematica_Directa_Geometrica/README.md#7-testbench-casos-de-prueba-y-la-singularidad))
y se recalculan desde cero en MATLAB con la cadena `T01·T12·T23·T34·T45` (MTH), sin reusar
ninguna fórmula del método geométrico — es una verificación **independiente**, no una
comparación circular.

Script: [`Verificacion_MTH_vs_Geometrico.m`](Verificacion_MTH_vs_Geometrico.m).

---

## Resultado: coinciden en los 3 casos

| Caso | θ1..θ5 (°) | MTH — x, y, z (m) | FPGA — x, y, z (m) | MTH — yaw, pitch, roll (°) | FPGA — yaw, pitch, roll (°) |
|---|---|---|---|---|---|
| 1 — singularidad | `0,0,0,0,0` | 0.4170, 0.0000, 0.0500 | 0.4178, 0.0004, 0.0503 | 45.00, **-90.00**, 135.00 | 0.00\*, **-90.00**, 180.00\* |
| 2 — limpio | `45,0,0,90,45` | 0.1676, 0.1676, 0.2300 | 0.1679, 0.1678, 0.2304 | -90.00, -0.00, -0.00 | -90.02, 0.00, 0.00 |
| 3 — limpio | `0,45,0,0,0` | 0.2949, 0.0000, 0.3449 | 0.2952, 0.0002, 0.3450 | **180.00**, -45.00, 0.00 | **-179.98**, -45.00, 0.02 |

Posición: coincide en las 2 fuentes, dentro del margen normal del CORDIC de la FPGA
(±0.02–0.1%, [lección 11, sección 7](../11_Cinematica_Directa_Geometrica/README.md#7-testbench-casos-de-prueba-y-la-singularidad)).
`pitch`: coincide exacto en los 3 casos.

**Las dos diferencias marcadas (\*) no son errores — son los mismos dos fenómenos ya
documentados, ahora confirmados por un método de cálculo completamente independiente:**

- **Caso 1 — gimbal lock.** Con `pitch=±90°` exacto, `yaw` y `roll` individuales no están
  definidos (es la razón por la que existen los cuaterniones) — cada método "reparte" el
  ángulo distinto. Solo la **suma** `yaw+roll` es invariante:
  MTH → `45+135=180`; FPGA (estandarizado por convención del profesor) → `0+180=180`.
  **Coinciden.**
- **Caso 3 — el corte de ±180°.** `yaw=180.00°` (MTH) y `yaw=-179.98°` (FPGA) son el
  **mismo ángulo físico** — es el punto de discontinuidad de `atan2`, no un desacuerdo real.

---

## Por qué importa

El método geométrico de la FPGA usa fórmulas cerradas y reducidas (obtenidas simplificando la
cadena DH en papel, con `sympy`/CSE) para gastar el mínimo de multiplicadores — eso es lo que
permitió bajar de 55,391 LEs (matrices homogéneas completas en tiempo real, Fitter Failed) a
1,754 LEs. Esta verificación confirma que esa reducción algebraica **no introdujo ningún error**
— construir la matriz completa 4×4 desde cero, sin atajos, da la misma pose física en los 3
casos, con las únicas discrepancias siendo fenómenos matemáticos ya conocidos (gimbal lock,
corte de `atan2`), no bugs.

---

## Cómo correrlo

Abrir [`Verificacion_MTH_vs_Geometrico.m`](Verificacion_MTH_vs_Geometrico.m) en MATLAB y
ejecutar — no requiere ningún toolbox (solo álgebra matricial estándar). Imprime la tabla de
arriba caso por caso con las notas de gimbal lock / corte de `atan2` incluidas.
