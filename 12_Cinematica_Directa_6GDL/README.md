# Cinemática Directa 6 GDL Real por Método Geométrico — de `FK_6R_Geometrico` a `FK_7R_Geometrico`

## Descripción General

La [lección 11](../11_Cinematica_Directa_Geometrica/README.md) implementó la cinemática directa
completa (posición + orientación) de un brazo que, a pesar del nombre del proyecto
(`FK_6R_Geometrico`), en realidad tenía **5 articulaciones revolutas + un roll de gripper** — la
muñeca no aportaba grados de libertad propios en el sentido estricto.

El diseño mecánico cambió: el brazo real del semillero tiene **6 articulaciones revolutas
independientes**. Este documento cubre todo lo que cambió al pasar de un brazo a otro:

1. Una tabla DH nueva, corregida por el profesor sobre la marcha.
2. Un hallazgo matemático importante: la posición del efector **deja de ser independiente de la
   muñeca** (algo que sí era cierto en el brazo de 5R y que simplificaba mucho el diseño anterior).
3. Un rediseño completo del VHDL: un bloque de orientación nuevo, una entrada nueva en el
   diagrama de bloques, y **una crisis de recursos real** (el diseño ingenuo no cabía en la FPGA)
   con su causa raíz medida y su arreglo.

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo), igual que en el resto del semillero.
> **Plataforma:** Cyclone IV E (EP4CE6E22C8) · Quartus 18.1
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [La corrección del profesor: "regla 4"](#1-la-correccion-del-profesor-regla-4)
2. [Tabla DH nueva](#2-tabla-dh-nueva)
3. [El hallazgo: la posición ya no es independiente de la muñeca](#3-el-hallazgo-la-posicion-ya-no-es-independiente-de-la-muñeca)
4. [Orientación: R0_6 y su reducción algebraica](#4-orientacion-r0_6-y-su-reduccion-algebraica)
5. [Arquitectura de módulos y el diagrama de conexión nuevo](#5-arquitectura-de-modulos-y-el-diagrama-de-conexion-nuevo)
6. [La crisis de recursos: cuando el diseño "correcto" no cabe en el chip](#6-la-crisis-de-recursos-cuando-el-diseno-correcto-no-cabe-en-el-chip)
7. [Verificación: ModelSim y MATLAB](#7-verificacion-modelsim-y-matlab)
8. [Comparación de recursos, de punta a punta](#8-comparacion-de-recursos-de-punta-a-punta)
9. [Cómo compilar y simular](#9-como-compilar-y-simular)
10. [Comparación con STM32: hardware dedicado vs. software, y el efecto del reloj](#10-comparacion-con-stm32-hardware-dedicado-vs-software-y-el-efecto-del-reloj)

---

## 1. La corrección del profesor: "regla 4"

Al presentar la asignación de ejes DH del brazo de 6 GDL, el profesor corrigió: *"los sistemas
coordenados 3 y 5 se deben devolver a los sistemas coordinados anteriores por la regla 4"*.

Interpretación aplicada (regla clásica de asignación DH: cuando el sistema de referencia de un
eslabón tiene libertad de orientación, se elige igual al del eslabón anterior en vez de
introducir un giro nuevo): el sistema 3 se hace **coincidir** con el sistema 2, y el sistema 5
con el sistema 4 — ninguno de los dos avanza por su cuenta. La distancia que antes recorrían
esos ejes no desaparece: se acumula en el eslabón **siguiente**.

Se verificó primero en MATLAB con Peter Corke (`Metodo_DH_Peter_6R.m`, `Robot.teach`) antes de
tocar ningún VHDL — con la tabla original (sin la corrección) el brazo no se dibujaba bien en
`q=0`; con la corrección, sí.

---

## 2. Tabla DH nueva

| i | θᵢ | dᵢ | αᵢ | aᵢ |
|---|---|---|---|---|
| 1 | θ1 | L1 = 6.5 cm | π/2 | 0 |
| 2 | θ2 | 0 | 0 | L2 = 10.7 cm |
| 3 | θ3+π/2 | 0 | π/2 | 0 |
| 4 | θ4+π/2 | L3+L4 = 14.0 cm (`Ld4`) | π/2 | 0 |
| 5 | θ5 | 0 | -π/2 | 0 |
| 6 | θ6 | L5+L6 = 17.3 cm (`Ld6`) | 0 | 0 |

Cambios respecto a la tabla original (antes de la corrección): filas 3 y 5 pasan su columna `a`
de la longitud del eslabón a `0` (los sistemas 3 y 5 no se mueven, "se devuelven" al sistema
anterior); esa distancia se suma al `d` de la fila siguiente (`d4 = L3+L4`, `d6 = L5+L6`).

**Verificado con `Robot.fkine` en `q = 0`:** brazo extendido, `x = 42 cm`, `y = 0`, `z = 6.5 cm`
(`= L1`), `yaw = -90°`, `pitch = 0°`, `roll = -90°` — la suma total de eslabones horizontales
(`L2+Ld4+Ld6 = 10.7+14.0+17.3 = 42.0 cm`) coincide exacto.

---

## 3. El hallazgo: la posición ya no es independiente de la muñeca

En el brazo de 5R+gripper, `theta5` era un giro puro que no movía el efector de sitio — por eso
`fk_geom_core` (posición) y `mat_r05_elems` (orientación) podían correr **en paralelo**, dos
ramas totalmente independientes desde `cordic_seq5`.

En el brazo de 6 GDL real esto deja de ser cierto: las filas 4 y 6 de la tabla DH tienen
`d4 = Ld4 = 14 cm` y `d6 = Ld6 = 17.3 cm` — desplazamientos físicos reales, no cero. Cuando
`theta4` o `theta5` giran, mueven literalmente el punto donde termina el brazo.

**Derivación** (misma idea de siempre: hasta el sistema 4 es la misma trigonometría plana de
toda la vida; lo nuevo es el tramo de la muñeca):

```
Parte A — posicion del sistema 4 (identica a la del 5R, pero con Ld4 en vez de L45,
y SIN termino phi234 porque theta4 ya no es coplanar con theta2,theta3 en esta tabla):

  phi2  = theta2
  phi23 = theta2 + theta3
  r4 = L2*cos(phi2) + Ld4*cos(phi23)
  z4 = L1 + L2*sin(phi2) + Ld4*sin(phi23)
  x4 = r4*cos(theta1) ; y4 = r4*sin(theta1)

Parte B — aporte de la muñeca (NUEVO): el desplazamiento d6=Ld6 en DH siempre ocurre a lo
largo del eje Z del sistema anterior (Z5). Como alpha6=0, Z6=Z5 -- y la tercera columna de
CUALQUIER matriz de rotacion R0_i es, por definicion, el eje Z de ese sistema expresado en
el mundo. Como ya hay que calcular R0_6 completa para la orientacion, se reutilizan sus
elementos R13,R23,R33 (columna 3) tambien para la posicion:

  x = x4 + Ld6*R13
  y = y4 + Ld6*R23
  z = z4 + Ld6*R33
```

Esto tiene una consecuencia directa de arquitectura: **`fk_geom_core` necesita `r13,r23,r33`
de `mat_r06_elems`**, así que el bloque de orientación tiene que terminar antes de que arranque
el de posición — ya no pueden correr en paralelo como en el diseño de 5R (ver
[sección 5](#5-arquitectura-de-modulos-y-el-diagrama-de-conexion-nuevo)).

---

## 4. Orientación: R0_6 y su reducción algebraica

`R0_6 = R1·R2·R3·R4·R5·R6` (misma construcción matriz-por-matriz que
[`Metodo_Geometrico_RPY_6R.m`](Metodo_Geometrico_RPY_6R.m)) se expandió con un sistema
algebraico (`sympy`, `cse`) para encontrar la forma con menos productos compartidos, y esa
reducción se **verificó numéricamente contra la fórmula completa antes de traducir a VHDL**
(2000 casos aleatorios, error máximo ~1e-16 — redondeo de punto flotante puro, no error real):

```
R11 = c6*(c1*(c23*s5) + c5*t5)  + s6*t6b
R21 = c6*(-c5*t6 + s1*(c23*s5)) + s6*t5b
R31 = -c6*t9 - s6*(c23*c4)
R32 = -c6*(c23*c4) + s6*t9
R33 = (c5*s23) + s4*(c23*s5)
R13 = c1*(c23*c5) - s5*t5
R23 = s1*(c23*c5) + s5*t6
```

con `t5,t5b,t6,t6b,t9` construidos a partir de 9 productos directos de las entradas
(ver comentario completo al inicio de [`mat_r06_elems.vhd`](mat_r06_elems.vhd)).

**Buena noticia:** [`atan2_seq3.vhd`](../11_Cinematica_Directa_Geometrica/atan2_seq3.vhd) (la
extracción de yaw/pitch/roll vía CORDIC, y la estandarización de la singularidad) **no cambió
nada** — las fórmulas `yaw=atan2(R21,R11)`, `pitch=atan2(-R31,ρ)`, `roll=atan2(R32,R33)` y el
umbral de singularidad son genéricas de cualquier matriz de rotación ZYX, sin importar cuántas
articulaciones la generaron.

**La singularidad se mudó de sitio.** En el brazo de 5R, `q=0` (brazo extendido) era el caso de
gimbal lock. En el brazo de 6 GDL, `q=0` da `ρ=1` (perfectamente definido) — la nueva
singularidad aparece en `theta5=90°, theta6=90°` (resto en 0), verificada y usada como caso de
prueba (ver [sección 7](#7-verificacion-modelsim-y-matlab)).

---

## 5. Arquitectura de módulos y el diagrama de conexión nuevo

| Módulo | Estado | Rol |
|---|---|---|
| [`geom_pkg.vhd`](geom_pkg.vhd) | Actualizado | Constantes `L1,L2,Ld4,Ld6` en Q2.13 |
| [`angle_sum_gen.vhd`](angle_sum_gen.vhd) | Simplificado | Ya no calcula `phi234` (theta4 no es coplanar con 2,3 en esta tabla); `phi2` ya no hace falta calcularlo, es literalmente `theta2` |
| `cordic_seq6.vhd` | **Nuevo** | 6 pasadas de `cordic_sincos_16` (antes 5): theta1, phi2, phi23, theta4, theta5, theta6 |
| `mat_r06_elems.vhd` | **Nuevo** | Los 7 elementos de R0_6 (sección 4) |
| [`fk_geom_core.vhd`](fk_geom_core.vhd) | Reescrito | Posición (sección 3) — ahora depende de `mat_r06_elems` |
| `atan2_seq3.vhd` | Sin cambios | Se reutiliza tal cual de la lección 11 |

La conexión que **no existía** en el diseño de 5R: `mat_r06_elems` → `r13,r23,r33` →
`fk_geom_core`, y el `done` de `mat_r06_elems` disparando el `start` de **ambos**
`fk_geom_core` y `atan2_seq3` (antes ese fan-out salía directo del `done` de `cordic_seq5`).

![Diagrama de conexion del top-level nuevo: cordic_seq6 alimenta a mat_r06_elems, que alimenta en serie a fk_geom_core y en paralelo a atan2_seq3](images/diagrama_bdf_6r.png)

![Diagrama de asignacion de ejes DH corregido, mostrando los sistemas 3 y 5 coincidiendo con los sistemas 2 y 4](images/diagrama_brazo_6dof_corregido.png)

---

## 6. La crisis de recursos: cuando el diseño "correcto" no cabe en el chip

La primera versión de `mat_r06_elems.vhd` y `fk_geom_core.vhd` implementaba las fórmulas de la
sección 4 **tal cual se leen**: cada producto de la fórmula, una llamada a `mul_q13(...)`
directa, repartida en 3 etapas de pipeline (mismo estilo que la lección 11). Matemáticamente
perfecta — confirmada en ModelSim con los mismos 3 casos de prueba de siempre. Al compilar en
Quartus:

| | Reportado |
|---|---|
| Total logic elements | **9,118 / 6,272 (145 %)** |
| Embedded Multiplier 9-bit elements | 30 / 30 (100 %) |
| **Flow Status** | **Failed** |

No cabe en la FPGA — el mismo tipo de fracaso que tumbó al método DH original en la lección 7,
esta vez a menor escala.

### Causa raíz (medida, no supuesta)

`mat_r06_elems.vhd` (v1) tenía **31** expresiones `mul_q13(...)` escritas como líneas de código
distintas (una por producto de la fórmula), y el nuevo `fk_geom_core.vhd` tenía **9** más — 40
multiplicaciones de 16 bits en total, contra solo 15 en el diseño de 5R
(`mat_r05_elems`=7 + `fk_geom_core` viejo=8).

El dato clave para entender **por qué** eso duele tanto: Quartus construye **un árbol de
compuertas dedicado por cada expresión de multiplicación distinta que aparece en el código
fuente** — no detecta automáticamente que 31 líneas con nombres de destino diferentes, dentro
de una máquina de estados donde solo una corre a la vez, podrían compartir el mismo hardware.
Cada árbol de 16×16 bits en lógica pura cuesta cientos de elementos lógicos. Prueba de esto: el
conteo de multiplicadores **embebidos** (hardware dedicado del chip) se quedó fijo en **4/30
tanto en el diseño de 5R como en el 6R ya arreglado** — ese "4" resulta ser, en los dos casos,
exactamente la única multiplicación de `cordic_atan2_16.vhd` (la corrección de ganancia del
CORDIC, una sola línea `mul_q13` en todo ese archivo, reutilizada). Es decir: **ninguna** de las
multiplicaciones de `mat_r0X_elems`/`fk_geom_core` usó nunca hardware dedicado, ni en el diseño
bueno de 5R ni en el malo de 6R — la diferencia completa entre 1,754 LEs y 9,118 LEs es
la diferencia entre construir **15** árboles de compuertas de multiplicación en total o
construir **40**.

### El arreglo: un solo multiplicador, reutilizado en serie

Misma idea que ya usa `cordic_seq6` (un solo `cordic_sincos_16` reutilizado en 6 pasadas en vez
de 6 en paralelo), aplicada a la multiplicación: **una sola línea `mul_q13(...)` en todo el
archivo**, con los operandos seleccionados por un `case` sobre un contador de paso, reutilizada
en 31 ciclos secuenciales para `mat_r06_elems` (9 para `fk_geom_core`). Al haber un único
punto de llamada en el código fuente, Quartus solo puede construir un único árbol de
compuertas — lo comparte por construcción, no por optimización esperada.

Antes de tocar el VHDL, la secuencia completa de 31/9 pasos se verificó **bit a bit en Python**
contra los mismos valores que ya se habían confirmado en ModelSim con la versión paralela —
para no arriesgar un error de reordenamiento en un cambio tan grande.

| | Antes (v1, paralelo) | Después (v2, serial) |
|---|---|---|
| Total logic elements | 9,118 / 6,272 (**145 %**) | **3,759 / 6,272 (60 %)** |
| Total registers | 2,053 | 2,393 |
| Embedded Multiplier 9-bit elements | 30 / 30 (100 %) | 4 / 30 (13 %) |
| Latencia `mat_r06_elems` | 3 ciclos | ~32 ciclos |
| Latencia `fk_geom_core` | 3 ciclos | ~11 ciclos |
| Latencia total (medida en ModelSim) | ~2.6 µs | ~3.2 µs |
| **Flow Status** | **Failed** | Failed (ver nota de pines abajo) |

Se sacrificó velocidad (la cadena completa pasa de ~130 a ~160 ciclos, ~23 % más lenta) a
cambio de 2.4× menos elementos lógicos — el mismo criterio de "área sobre velocidad" que ha
guiado todo este proyecto desde que se abandonó el método DH.

**Nota — `Flow Status` sigue en `Failed`, y es esperado:** el `Fitter` todavía falla, pero ya
no por LEs — es por **pines** (`Total pins: 196/92, 213%`). Este es el mismo problema, ya
documentado, de la [lección 11](../11_Cinematica_Directa_Geometrica/README.md) (ahí eran 116
pines sobre 92 disponibles, con 5 entradas theta; con 6 entradas theta ahora son 196) — expone
demasiadas entradas de 16 bits en paralelo para los pines de E/S reales del `EP4CE6E22C8`
(TQFP144, 92 pines de usuario). Sigue **fuera de alcance** de esta lección, deliberadamente,
igual que se decidió con el profesor en la lección 11 — es un problema de asignación de pines
físicos (o de empaquetar las entradas en un bus serie), no de la arquitectura de cómputo.

![Waveform de ModelSim ANTES de la optimizacion de multiplicadores, con los 3 casos de prueba y ~2.6us por caso](images/Wave_Form_6R.png)

![Waveform de ModelSim DESPUES de la optimizacion, mismos 3 casos con los mismos resultados numericos y ~3.2us por caso](images/Wave_Form_6R_optimizado.png)

---

## 7. Verificación: ModelSim y MATLAB

Tres casos de prueba, corridos en el testbench de integración final (`tb.vhd` sobre
`FK_7R_Geometrico`, el top-level generado por Quartus desde el BDF):

| Caso | Entradas (grados) | Posición esperada | Orientación esperada | Nota |
|---|---|---|---|---|
| A — extendido | todo 0 | x=3441 y=0 z=532 | yaw=-12868(-90°) pitch=0 roll=-12868(-90°) | ρ=1 exacto, sin singularidad |
| B — singularidad | th5=90 th6=90, resto 0 | x=2023 y=1417 z=532 | yaw=0(fijo) pitch≈12868(90°) roll=25736(fijo,180°) | singularidad **nueva** del 6R (ya no está en q=0) |
| C — genérico | th1=30 th2=20 th3=-15 th4=45 th5=60 th6=-70 | x≈1815 y≈2050 z≈1859 | yaw≈-6076 pitch≈-4941 roll≈-5357 | margen normal de CORDIC |

Los tres coincidieron dentro del margen esperado (exacto en A/B por ser múltiplos de 90°; unas
pocas unidades Q2.13 de diferencia en C, típico del CORDIC de 12 iteraciones).

**Verificación cruzada en MATLAB:** [`Metodo_Geometrico_RPY_6R.m`](Metodo_Geometrico_RPY_6R.m)
reproduce los 3 casos de arriba (comentario al inicio del archivo con los valores exactos de
`theta1..theta6` a usar para cada uno) y reconstruye `Rz(yaw)·Ry(pitch)·Rx(roll)` contra `R0_6`
como chequeo independiente. [`Metodo_DH_Peter_6R.m`](Metodo_DH_Peter_6R.m) valida la tabla DH
completa con el toolbox de Peter Corke (`Robot.fkine`, `Robot.teach`).

---

## 8. Comparación de recursos, de punta a punta

| Método | Logic elements | % del EP4CE6 | Fitter |
|---|---|---|---|
| DH clásico, matrices 4×4 (lección 7) | 55,391 | 883 % | Failed |
| Geométrico, 5R+gripper (lección 11) | 1,754 | 27 % | OK |
| Geométrico, 6 GDL real — v1 (multiplicadores en paralelo) | 9,118 | 145 % | Failed |
| Geométrico, 6 GDL real — v2 (multiplicadores en serie) | **3,759** | **60 %** | Failed (solo por pines, ver sección 6) |

El salto de 1,754 a 3,759 LEs (2.1×) entre el brazo de 5R y el de 6 GDL real es genuino y
esperado — hay una articulación más y la orientación real necesita bastante más álgebra (sección
4) — no es un síntoma de mal diseño. El salto a 9,118 (v1) sí lo era, y quedó documentado en la
sección 6 con su causa raíz medida, no solo su síntoma.

---

## 9. Cómo compilar y simular

1. Abrir el proyecto `FK_6R_Geometrico.qpf` en Quartus 18.1.
2. Generar/actualizar símbolos de los `.vhd` de esta lección (`File > Create/Update > Create
   Symbol Files for Current File`) si se van a editar más adelante.
3. El top-level es `FK_7R_Geometrico.bdf` — ya cableado según el diagrama de la sección 5.
4. `Start Compilation` (Analysis & Synthesis + Fitter). El conteo de LEs debe salir ~60 %.
5. Generar el `.vhd` del top-level si hace falta simular: con el `.bdf` abierto,
   `File > Create/Update > Create HDL Design File for Current File` (VHDL).
6. En ModelSim, `do simular.do` desde la carpeta del proyecto — corre los 3 casos de la
   sección 7 y los imprime en el Transcript.

---

## 10. Comparación con STM32: hardware dedicado vs. software, y el efecto del reloj

El puerto completo a STM32 (dos proyectos Keil, mismo código de cinemática, 216MHz vs 16MHz sin
PLL, explicado paso a paso con el log real del UART) quedó documentado aparte en la
**[lección 13](../13_Cinematica_Directa_6GDL_STM32/README.md)**. Resumen del resultado:

| | FPGA (50 MHz) | STM32 @ 216 MHz | STM32 @ 16 MHz |
|---|---|---|---|
| Tiempo (constante / según caso) | ~3.1 µs | 33.5–48.8 µs | 452–659 µs |
| **Veces más lento que la FPGA** | 1× | **10.8×–15.7×** | **146×–212×** |

A pesar de que el STM32 a máxima velocidad tiene un reloj **4.3 veces más rápido** que la FPGA,
termina el mismo cálculo entre **10.8 y 15.7 veces más lento** — hardware dedicado en pipeline
le gana por mucho a software secuencial, sin importar cuánto reloj se le meta al software.
