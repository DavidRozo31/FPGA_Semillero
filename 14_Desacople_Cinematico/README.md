# Desacople Cinemático 6GDL — de la Pose Deseada a los Seis Ángulos Articulares

## Descripción General

Las lecciones anteriores resolvieron la **cinemática directa** del brazo de 6 GDL: dados los seis
ángulos articulares, calcular dónde queda el efector y cómo queda orientado
([lección 12](../12_Cinematica_Directa_6GDL/README.md) en FPGA,
[lección 13](../13_Cinematica_Directa_6GDL_STM32/README.md) el mismo cálculo en STM32). Esta
lección resuelve el problema **inverso**: dada una posición `(x,y,z)` y una orientación deseadas
(matriz de rotación `R0_6`), calcular los seis ángulos articulares `θ1..θ6` que la producen.

Resolver esto de frente — plantear las 6 ecuaciones no lineales acopladas y despejar — no es
práctico. Este proyecto usa el truco clásico de robótica para brazos con **muñeca esférica**
(los ejes de las últimas tres articulaciones se cortan en un mismo punto, condición de Pieper,
válida aquí porque `a4=a5=a6=0` en la tabla DH de la [lección 12](../12_Cinematica_Directa_6GDL/README.md#2-tabla-dh-nueva)):
el problema de 6 incógnitas se **desacopla** en dos problemas independientes y mucho más simples,
resueltos en cadena:

1. **Posición** (`θ1,θ2,θ3`) — se retrocede desde la pose del efector hasta el *centro de muñeca*
   y se resuelve como si fuera un brazo 3R plano+base, con la misma trigonometría de siempre
   (ley de cosenos + `atan2`, como en la [lección 5](../5_2R_Inverse_Cinematic/README.md)).
2. **Orientación** (`θ4,θ5,θ6`) — ya conocidos `θ1,θ2,θ3`, se reconstruye `R0_3`, se invierte
   (transpuesta, porque es una matriz de rotación) y se combina con la `R0_6` deseada para aislar
   `R3_6`, de donde salen los tres ángulos de la muñeca directamente por `atan2`.

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo), igual que el resto del semillero —
> paquete compartido [`ik_pkg.vhd`](ik_pkg.vhd) (reexporta [`cordic_pkg.vhd`](cordic_pkg.vhd)).
> **Plataforma:** Cyclone IV E (EP4CE6E22A7) · Quartus 18.1 · ModelSim-Altera.
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Paso 1 — Centro de muñeca (`Wrist_Center`)](#1-paso-1--centro-de-muñeca-wrist_center)
2. [Paso 2 — Posición: cinemática inversa 3R (`Cinematica_Inversa3R`)](#2-paso-2--posición-cinemática-inversa-3r-cinematica_inversa3r)
3. [Paso 3 — Reconstruir `R0_3` (`R03_Builder`)](#3-paso-3--reconstruir-r0_3-r03_builder)
4. [Paso 4 — Aislar `R3_6` (`Mat3_Transpose` + `Mat3x3_Mult`)](#4-paso-4--aislar-r3_6-mat3_transpose--mat3x3_mult)
5. [Paso 5 — Ángulos de la muñeca (`WristAngles3R`)](#5-paso-5--ángulos-de-la-muñeca-wristangles3r)
6. [Arquitectura completa: `Desacople6R_Top`](#6-arquitectura-completa-desacople6r_top)
7. [Librería aritmética compartida](#7-librería-aritmética-compartida)
8. [Verificación: testbenches + MATLAB](#8-verificación-testbenches--matlab)
9. [Cómo compilar y simular](#9-cómo-compilar-y-simular)
10. [Estructura de archivos](#10-estructura-de-archivos)

---

## 1. Paso 1 — Centro de muñeca (`Wrist_Center`)

Lo primero es "quitarle" al punto deseado el tramo final del brazo (`L5+L6`, el mismo `Ld6` de
la tabla DH de la lección 12) para quedarse con el punto donde estaría la muñeca si no tuviera
orientación propia:

```
P_muñeca = P_deseado − (L5+L6) · R0_6[:,3]
```

La tercera columna de `R0_6` (`R13,R23,R33`, las entradas `rz_x,rz_y,rz_z` del bloque) es, por
definición, el eje Z del sistema 6 expresado en el mundo — el mismo argumento que ya se usaba en
la [lección 12, sección 3](../12_Cinematica_Directa_6GDL/README.md#3-el-hallazgo-la-posición-ya-no-es-independiente-de-la-muñeca)
para la posición directa, aplicado ahora al revés.

Implementado en [`Wrist_Center.vhd`](Wrist_Center.vhd) (`L5+L6 = 0.173 m`) con tres
`fp_multiplier` + tres `fp_adder` en modo resta, uno por eje.

---

## 2. Paso 2 — Posición: cinemática inversa 3R (`Cinematica_Inversa3R`)

Con el centro de muñeca `(Px,Py,Pz)` ya aislado, resolver `θ1,θ2,θ3` es el problema clásico de un
brazo 3R plano con base rotante — ley de cosenos para `θ3`, `atan2` para `θ1` y para la
combinación `θ2 = α − φ`:

```
θ1 = atan2(Py, Px)
b  = sqrt(Px² + Py²) ;  c = Pz − L1 ;  e = sqrt(b² + c²)
cos(θ3) = (e² − L2² − (L3+L4)²) / (2·L2·(L3+L4))
θ3 = atan2( sqrt(1−cos²θ3), cos θ3 )
θ2 = atan2(c,b) − atan2( (L3+L4)·sen θ3 , L2+(L3+L4)·cos θ3 )
```

Implementado como una red estructural (sin máquina de estados: cada bloque dispara con el
`valid`/`done` del anterior) en [`Cinematica_Inversa3R.vhd`](Cinematica_Inversa3R.vhd), construida
originalmente en el editor gráfico de bloques de Quartus
([`Block2RTest.bdf`](Block2RTest.bdf)). Usa `cordic_atan2`, `fp_multiplier`, `fp_adder`,
`fp_divider` y `sqrt_q13` de la librería compartida (sección 7); `pulse_join2` sincroniza las
ramas que llegan con distinta profundidad.

> Las longitudes están en [`ik_pkg`](ik_pkg.vhd)/constantes locales del bloque en Q2.13
> (`L1`, `L2`, `L3+L4`). Al revisar los valores contra la tabla DH de la lección 12 vale la pena
> confirmar con el profesor que `L1` (0.06 m aquí, en el mismo valor que usa
> [`Comprobacion6R_D.m`](Comprobacion6R_D.m)) sigue correspondiendo a la medida real del eslabón 1
> — la lección 12 documentó `L1 = 0.065 m`.

---

## 3. Paso 3 — Reconstruir `R0_3` (`R03_Builder`)

Con `θ1,θ2,θ3` ya resueltos, se reconstruye la matriz de rotación acumulada `R0_3 = R0_1·R1_2·R2_3`
para poder "restarle" su efecto a la orientación deseada en el paso siguiente. Cada `Ri` se arma
con seno/coseno del ángulo respectivo (bloque [`Angle_SinCos.vhd`](Angle_SinCos.vhd), que envuelve
el CORDIC [`cordic_sincos_16.vhd`](cordic_sincos_16.vhd)) y las constantes fijas `0/±1` que vienen
de los `α` de la tabla DH (`π/2`, `0`, `π/2`), sin gastar multiplicadores en calcular senos/cosenos
de ángulos constantes. Las dos multiplicaciones de matrices (`R0_1·R1_2` y el resultado `·R2_3`)
reutilizan el mismo bloque [`Mat3x3_Mult.vhd`](Mat3x3_Mult.vhd) dos veces.

Implementado en [`R03_Builder.vhd`](R03_Builder.vhd).

---

## 4. Paso 4 — Aislar `R3_6` (`Mat3_Transpose` + `Mat3x3_Mult`)

Como toda matriz de rotación es ortogonal, `R0_3⁻¹ = R0_3ᵀ` — no hace falta invertir de verdad,
solo transponer ([`Mat3_Transpose.vhd`](Mat3_Transpose.vhd)). Con `R3_0` ya disponible, se
multiplica por la `R0_6` deseada (entradas `R11_d..R33_d` del top-level) reutilizando otra vez
[`Mat3x3_Mult.vhd`](Mat3x3_Mult.vhd) — solo hacen falta las columnas 1 y 2 de la primera fila y la
fila 3 completa (`c13,c23,c31,c32,c33`), que es justo lo que necesita el paso 5:

```
R3_6 = R0_3ᵀ · R0_6
```

---

## 5. Paso 5 — Ángulos de la muñeca (`WristAngles3R`)

Con `R3_6` aislada, los tres ángulos de la muñeca esférica salen directo, elemento por elemento
(mismo criterio de singularidad — `θ5` siempre positivo vía `sqrt` — que en las secciones de
orientación de las lecciones anteriores):

```
θ4 = atan2( r13, −r23 )
θ6 = atan2( r32, −r31 )
θ5 = atan2( sqrt(1 − r33²), r33 )
```

Implementado en [`WristAngles3R.vhd`](WristAngles3R.vhd). `θ4` y `θ6` tienen la misma profundidad
de pipeline (negación + `atan2`) y se combinan con un AND directo; `θ5` es una rama más larga
(multiplicación + resta + raíz + `atan2`) y necesita el `pulse_join2` real para el `done` final.

---

## 6. Arquitectura completa: `Desacople6R_Top`

[`Desacople6R_Top.vhd`](Desacople6R_Top.vhd) (generado desde el diagrama de bloques de Quartus,
`Desacople6R_Top.bdf`) encadena los cinco bloques anteriores en una tubería de datos pura: cada
etapa dispara con el pulso `done` de la anterior, sin controlador central ni máquina de estados —
mismo estilo que el resto del semillero.

```
                (x,y,z) + R0_6 deseadas
                          │
                 ┌────────┴─────────┐
                 │   Wrist_Center    │  → Px,Py,Pz (centro de muñeca)
                 └────────┬─────────┘
                          │
                 ┌────────┴─────────┐
                 │ Cinematica_Inversa3R │ → θ1,θ2,θ3
                 └────────┬─────────┘
                          │
                 ┌────────┴─────────┐
                 │    R03_Builder    │  → R0_3
                 └────────┬─────────┘
                          │
                 ┌────────┴─────────┐
                 │  Mat3_Transpose   │  → R3_0
                 └────────┬─────────┘
                          │            R0_6 deseada
                 ┌────────┴─────────┐  ┌──────────┘
                 │    Mat3x3_Mult    │◄─┘
                 └────────┬─────────┘  → R3_6
                          │
                 ┌────────┴─────────┐
                 │  WristAngles3R    │  → θ4,θ5,θ6
                 └───────────────────┘
```

> _Espacio para foto/captura real:_
> ![Diagrama de bloques (BDF) de Desacople6R_Top en Quartus, mostrando las conexiones entre Wrist_Center, Cinematica_Inversa3R, R03_Builder, Mat3_Transpose, Mat3x3_Mult y WristAngles3R](images/diagrama_bdf_desacople_top.png)

Entradas: `R11_d..R33_d` (orientación deseada `R0_6`), `x_d,y_d,z_d` (posición deseada), `Start`.
Salidas: `tetha1Final..tetha6Final` y `Desacople_Listo`.

---

## 7. Librería aritmética compartida

Los bloques anteriores no reinventan la aritmética: reutilizan, sin modificar, la misma librería
Q2.13 de las lecciones de cinemática directa ([11](../11_Cinematica_Directa_Geometrica/README.md),
[12](../12_Cinematica_Directa_6GDL/README.md)):

| Archivo | Rol |
|---|---|
| [`ik_pkg.vhd`](ik_pkg.vhd) | Constantes Q2.13 (`π`, `π/2`, `1.0`, `0.5`) y funciones auxiliares (`mul_q13`, `abs_q13`) |
| [`cordic_pkg.vhd`](cordic_pkg.vhd) | Parámetros compartidos del CORDIC |
| [`cordic_atan2.vhd`](cordic_atan2.vhd) | `atan2(y,x)` por CORDIC |
| [`cordic_sincos_16.vhd`](cordic_sincos_16.vhd) | `sin`/`cos` por CORDIC (usado dentro de `Angle_SinCos`) |
| [`fp_adder.vhd`](fp_adder.vhd) | Suma/resta Q2.13 (`op` selecciona el signo) |
| [`fp_multiplier.vhd`](fp_multiplier.vhd) | Multiplicación Q2.13 |
| [`fp_divider.vhd`](fp_divider.vhd) | División Q2.13 |
| [`sqrt_q13.vhd`](sqrt_q13.vhd) | Raíz cuadrada Q2.13 |
| [`pulse_join2.vhd`](pulse_join2.vhd) | Sincroniza dos pulsos `done` de ramas paralelas |
| [`done_latch_reg.vhd`](done_latch_reg.vhd) | Latch de flanco para retener un pulso `done` |

Cada uno trae su símbolo de bloque (`.bsf`) para el editor gráfico de Quartus.

---

## 8. Verificación: testbenches + MATLAB

Dos niveles de testbench, cada uno con su script de ModelSim:

- [`tb_Cinematica_Inversa3R.vhd`](tb_Cinematica_Inversa3R.vhd) + [`simular_3R.do`](simular_3R.do)
  — prueba solo el bloque de posición (`θ1,θ2,θ3`).
- [`tb_Desacople6R_Top.vhd`](tb_Desacople6R_Top.vhd) + [`simular6RD.do`](simular6RD.do) — prueba
  la tubería completa (`θ1..θ6`), con cuatro casos de pose deseada:

| Caso | Orientación (Z,Y,X, grados) | Posición (m) |
|---|---|---|
| 1 — Identidad | 0, 0, 0 | 0.30, 0.00, 0.45 |
| 2 — Rotación 90° en Z | 90, 0, 0 | 0.20, 0.25, 0.35 |
| 3 — ZYX combinada | 30, 45, 22.5 | 0.15, 0.10, 0.40 |
| 4 — Caso de Fabián | −128.2, −11.5, 26.1 | 0.0879, 0.18655, 0.38781 |

Los mismos cuatro casos se recalculan de forma **independiente** en
[`Comprobacion6R_D.m`](Comprobacion6R_D.m) (MATLAB puro, sin tocar el VHDL) siguiendo exactamente
los mismos cinco pasos de este documento — construcción de `R0_6` desde ángulos Euler ZYX, centro
de muñeca, IK 3R, `R0_3`/transpuesta/`R3_6`, ángulos de muñeca — para tener un resultado esperado
contra el cual comparar la salida del testbench de VHDL.

> _Espacio para foto/captura real:_
> ![Waveform de ModelSim de tb_Desacople6R_Top mostrando los 4 casos de prueba y las salidas theta1Final..theta6Final](images/waveform_desacople_6r.png)

`Desacople_Top_nativelink_simulation.rpt` registra una corrida exitosa vía NativeLink
(Quartus → ModelSim-Altera).

---

## 9. Cómo compilar y simular

1. Abrir [`Desacople_Cinematico6R.qpf`](Desacople_Cinematico6R.qpf) en Quartus Prime 18.1 (device
   Cyclone IV E `EP4CE6E22A7`, top-level `Desacople6R_Top`, ver [`Desacople_Top.qsf`](Desacople_Top.qsf)).
2. Simular con ModelSim-Altera vía NativeLink (`Tools → Run Simulation Tool → RTL Simulation`),
   que ejecuta automáticamente [`simular6RD.do`](simular6RD.do) — compila toda la librería
   compartida, el pipeline completo y corre `tb_Desacople6R_Top` con los 4 casos de la sección 8.
3. Para probar solo el bloque de posición, usar [`simular_3R.do`](simular_3R.do) con
   `tb_Cinematica_Inversa3R`.
4. Verificar los ángulos esperados corriendo [`Comprobacion6R_D.m`](Comprobacion6R_D.m) en
   MATLAB y comparando contra el `REPORT` impreso por el testbench de VHDL.

> _Espacio para foto/captura real:_
> ![Fotografía del brazo robótico del semillero con la muñeca esférica marcada (ejes 4, 5 y 6 cortándose en un mismo punto)](images/brazo_muneca_esferica.png)

---

## 10. Estructura de archivos

| Archivo | Contenido |
|---|---|
| `Desacople6R_Top.vhd` / `.bdf` | Top-level: encadena los 5 bloques del desacople |
| `Block2RTest.bdf` | Esquemático de prueba gráfico usado para armar `Cinematica_Inversa3R` |
| `Wrist_Center.vhd` / `.bsf` | Paso 1 — centro de muñeca |
| `Cinematica_Inversa3R.vhd` / `.bsf` | Paso 2 — `θ1,θ2,θ3` (IK 3R) |
| `R03_Builder.vhd` / `.bsf` | Paso 3 — reconstruye `R0_3` |
| `Angle_SinCos.vhd` | Seno/coseno por ángulo, usado por `R03_Builder` |
| `Mat3_Transpose.vhd` / `.bsf` | Paso 4a — transpone `R0_3` |
| `Mat3x3_Mult.vhd` / `.bsf` | Multiplicación 3×3, reutilizada en pasos 3 y 4b |
| `WristAngles3R.vhd` / `.bsf` | Paso 5 — `θ4,θ5,θ6` |
| `ik_pkg.vhd`, `cordic_pkg.vhd` | Paquetes compartidos Q2.13 / CORDIC |
| `cordic_atan2.vhd`, `cordic_sincos_16.vhd` | CORDIC (`atan2`, `sin`/`cos`) |
| `fp_adder.vhd`, `fp_multiplier.vhd`, `fp_divider.vhd`, `sqrt_q13.vhd` | Primitivas aritméticas Q2.13 |
| `pulse_join2.vhd`, `done_latch_reg.vhd` | Utilidades de sincronización de pulsos `done` |
| `tb_Cinematica_Inversa3R.vhd`, `simular_3R.do` | Testbench + script del bloque de posición |
| `tb_Desacople6R_Top.vhd`, `simular6RD.do` | Testbench + script de la tubería completa |
| `Comprobacion6R_D.m` | Verificación independiente en MATLAB de los 4 casos de prueba |
| `Desacople_Cinematico6R.qpf`, `Desacople_Top.qsf`, `modelsim.ini` | Proyecto Quartus / configuración ModelSim |
| `Desacople_Top_nativelink_simulation.rpt` | Log de la corrida de simulación vía NativeLink |
| `db/`, `incremental_db/` | Caché de compilación de Quartus (generada automáticamente, no editar) |

Archivos `*.bak` junto a varios `.vhd`/`.do` son copias de respaldo automáticas de Quartus de
versiones anteriores de cada bloque, conservadas como historial de las iteraciones del diseño.
