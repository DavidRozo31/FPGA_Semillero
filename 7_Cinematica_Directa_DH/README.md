# Cinemática Directa 6R por Denavit-Hartenberg — Proyecto `Fk6R_DH`

## Descripción General

Este módulo calcula la **cinemática directa** de un robot serial de 5 articulaciones
revolutas (llamado "6R" en el nombre del proyecto, contando el eslabón base) usando el
método estándar de **Denavit-Hartenberg (DH)**. Es la evolución natural de
[`6_Cinematica_Directa_Matrices_Homogeneas`](../6_Cinematica_Directa_Matrices_Homogeneas/README.md):
en vez de construir cada matriz de transformación "a mano" (rotación + traslación por
separado), aquí cada eslabón se describe con los 4 parámetros clásicos de DH y una única
fórmula genérica genera la matriz de transformación homogénea correspondiente.

Al igual que en la lección 6, este proyecto reutiliza `mat4x4_mul` (ya documentado) y el
CORDIC seno/coseno de la [lección 5](../5_2R_Inverse_Cinematic/Operadores.md). El módulo
realmente nuevo aquí es `t_dh_gen`.

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo)
> **Plataforma:** Cyclone IV E (EP4CE6E22C8) · Quartus 18.1
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Teoría: parámetros DH y la matriz de transformación](#1-teoría-parámetros-dh-y-la-matriz-de-transformación)
2. [Parámetros DH reales del robot (5 eslabones)](#2-parámetros-dh-reales-del-robot-5-eslabones)
3. [Módulo nuevo: `t_dh_gen`](#3-módulo-nuevo-t_dh_gen)
4. [Top-level generado: `Fk6R_DH` (esquemático)](#4-top-level-generado-fk6r_dh-esquemático)
5. [Sincronización por AND: una alternativa más simple](#5-sincronización-por-and-una-alternativa-más-simple)
6. [Testbench y casos de prueba](#6-testbench-y-casos-de-prueba)
7. [Cómo compilar y simular](#7-cómo-compilar-y-simular)
8. [Comparación: DH vs. matrices homogéneas](#8-comparación-dh-vs-matrices-homogéneas)

---

## 1. Teoría: parámetros DH y la matriz de transformación

La convención Denavit-Hartenberg describe la transformación entre dos eslabones consecutivos
con **4 parámetros**: `θ` (ángulo de junta, variable), `d` (desplazamiento a lo largo de Z),
`a` (longitud del eslabón, a lo largo de X) y `α` (torsión del eslabón, ángulo entre ejes Z
consecutivos). La matriz de transformación homogénea resultante es:

```
        [ cos(θ)   -cos(α)sin(θ)    sin(α)sin(θ)    a·cos(θ) ]
T(θ) =  [ sin(θ)    cos(α)cos(θ)   -sin(α)cos(θ)    a·sin(θ) ]
        [   0            sin(α)          cos(α)          d       ]
        [   0               0               0             1      ]
```

Esta es exactamente la fórmula comentada en la cabecera de `t_dh_gen.vhd`:

```vhdl
--  Matriz T resultante (DH estandar):
--    [ cos(tz)   -Cx*sin(tz)   Sx*sin(tz)   Lx*cos(tz) ]
--    [ sin(tz)    Cx*cos(tz)  -Sx*cos(tz)   Lx*sin(tz) ]
--    [    0          Sx           Cx            Lz      ]
--    [    0           0            0             1      ]
```

Donde la nomenclatura del código usa `tz` = `θ` (ángulo de junta, variable en tiempo de
ejecución) y `Cx`/`Sx` = `cos(α)`/`sin(α)` (torsión del eslabón). **Diferencia clave de
diseño:** el hardware no calcula `α` con un CORDIC — recibe directamente `cos(α)` y `sin(α)`
como entradas ya calculadas (`cx_in`, `sx_in`). Esto tiene sentido porque, en un robot serial
típico, `α` es un parámetro **fijo** de la geometría del robot (no cambia junta a junta en
tiempo real como sí lo hace `θ`), así que no vale la pena gastar un segundo CORDIC en
calcularlo cada vez: se calcula una sola vez (en MATLAB o a mano) y se carga como constante.
Ver la [lección 8](../8_MATLAB_Peter_Corke_Cinematica/README.md#verificador_dhm-y-prueba_generalidad_dhm)
para cómo se generan estos valores `cx`/`sx` desde MATLAB antes de cargarlos al testbench.

---

## 2. Parámetros DH reales del robot (5 eslabones)

Tomados directamente de `fk6r_tb.vhd` (y coinciden con `Verificador_DH.m`, lección 8):

| Eslabón | `θ` (variable) | `d` | `a` | `α` | `cx=cos(α)` | `sx=sin(α)` |
|---|---|---|---|---|---|---|
| 1 (T01) | `theta1` | `0.10 m` | `0` | `π/2` | `0` | `1` (`8192`) |
| 2 (T12) | `theta2` | `0` | `0.12 m` | `0` | `1` (`8192`) | `0` |
| 3 (T23) | `theta3` | `0` | `0.10 m` | `0` | `1` (`8192`) | `0` |
| 4 (T34) | `theta4` | `0` | `0` | `π/2` | `0` | `1` (`8192`) |
| 5 (T45) | `theta5` | `0.06 m` | `0` | `0` | `1` (`8192`) | `0` |

En Q2.13 (factor `8192`), los valores de `d` y `a` usados literalmente en el testbench son:

| Constante | Valor real | Valor Q2.13 |
|---|---|---|
| `LZ1` (d, eslabón 1) | `0.10 m` | `819` |
| `LX2` (a, eslabón 2) | `0.12 m` | `983` |
| `LX3` (a, eslabón 3) | `0.10 m` | `819` |
| `LZ5` (d, eslabón 5) | `0.06 m` | `492` |

La transformación total del efector respecto a la base es la cadena completa:

```
T05 = T01 · T12 · T23 · T34 · T45
```

---

## 3. Módulo nuevo: `t_dh_gen`

Genera **una** matriz DH a partir de 5 entradas: el ángulo variable `theta_in` y los 4
parámetros fijos del eslabón (`cx_in`, `sx_in`, `lz_in`, `lx_in`):

```vhdl
entity t_dh_gen is
    Port (
        clk, rst, start : in std_logic;
        theta_in : in  std_logic_vector(15 downto 0);  -- angulo articular (variable)
        cx_in    : in  std_logic_vector(15 downto 0);  -- cos(alpha) del eslabon (fijo)
        sx_in    : in  std_logic_vector(15 downto 0);  -- sin(alpha) del eslabon (fijo)
        lz_in    : in  std_logic_vector(15 downto 0);  -- d: desplazamiento en Z (fijo)
        lx_in    : in  std_logic_vector(15 downto 0);  -- a: longitud del eslabon (fijo)
        t_r0c0 .. t_r3c3 : out std_logic_vector(15 downto 0);
        done : out std_logic
    );
end t_dh_gen;
```

### Tres etapas internas

**Etapa 0 (en el ciclo de `start`):** se capturan los 4 parámetros fijos en latches
(`cx_latch`, `sx_latch`, `lz_latch`, `lx_latch`) para que queden estables durante todo el
cálculo, mientras el CORDIC arranca en paralelo con `theta_in`:

```vhdl
if start = '1' then
    done_reg <= '0';
    cx_latch <= signed(cx_in);
    sx_latch <= signed(sx_in);
    lz_latch <= signed(lz_in);
    lx_latch <= signed(lx_in);
end if;
```

**Etapa 1 (`cord_done`, ciclo 13):** cuando `cordic_sincos_16` termina, se capturan `cos`/`sin`
en `cos_r`/`sin_r`, y se levanta `v1` para la siguiente etapa.

**Etapa 2 (`v1='1'`, ciclo 14):** se calculan las 12 celdas no triviales de la matriz DH usando
`mul_q13` (mismo paquete `ik_pkg` que en `mat4x4_mul`, lección 6):

```vhdl
-- Fila 0: [ cos(tz)  -Cx*sin(tz)   Sx*sin(tz)   Lx*cos(tz) ]
R00 <= cos_r;
R01 <= -mul_q13(cx_latch, sin_r);
R02 <=  mul_q13(sx_latch, sin_r);
R03 <=  mul_q13(lx_latch, cos_r);
-- Fila 1: [ sin(tz)   Cx*cos(tz)  -Sx*cos(tz)   Lx*sin(tz) ]
R10 <= sin_r;
R11 <=  mul_q13(cx_latch, cos_r);
R12 <= -mul_q13(sx_latch, cos_r);
R13 <=  mul_q13(lx_latch, sin_r);
-- Fila 2: [ 0   Sx   Cx   Lz ]
R21 <= sx_latch;
R22 <= cx_latch;
R23 <= lz_latch;
done_reg <= '1';
```

Filas 2 (columna 0) y 3 completas son constantes (`0`, `0`, `0`, `1`) y se asignan
directamente por fuera del proceso, igual que en `t_matrix_gen`.

### Latencia

**15 ciclos** (13 del CORDIC + 1 de latch de `cos`/`sin` + 1 de cálculo de la matriz) — un
ciclo más que `t_matrix_gen` (14 ciclos) porque aquí el cálculo de cada celda involucra un
producto adicional (`Cx·sin`, `Sx·sin`, etc.) que se registra en una etapa separada en vez de
resolverse en la misma etapa de latch de `cos`/`sin`.

---

## 4. Top-level generado: `Fk6R_DH` (esquemático)

Igual que en la lección 6, `Fk6R_DH.vhd` **es generado automáticamente** desde el diagrama de
bloques `Fk6R_DH.bdf` — no se edita a mano, y usa nombres de señal genéricos
(`SYNTHESIZED_WIRE_n`). Es el `TOP_LEVEL_ENTITY` declarado en `Fk6R_DH.qsf`.

### Instancias del esquemático

| Instancia | Tipo | Entradas | Calcula |
|---|---|---|---|
| `b2v_inst_T01` | `t_dh_gen` | `theta_in1`, `cx_in1`, `sx_in1`, `lz_in1`, `lx_in1` | `T01` |
| `b2v_inst_T12` | `t_dh_gen` | `theta_in2`, `cx_in2`, `sx_in2`, `lz_in2`, `lx_in2` | `T12` |
| `b2v_inst_T23` | `t_dh_gen` | `theta_in3`, `cx_in3`, `sx_in3`, `lz_in3`, `lx_in3` | `T23` |
| `b2v_inst_T34` | `t_dh_gen` | `theta_in4`, `cx_in4`, `sx_in4`, `lz_in4`, `lx_in4` | `T34` |
| `b2v_inst_T45` | `t_dh_gen` | `theta_in5`, `cx_in5`, `sx_in5`, `lz_in5`, `lx_in5` | `T45` |
| `b2v_inst_mul1` | `mat4x4_mul` | `A=T01`, `B=T12` | `T01·T12` |
| `b2v_inst_mul2` | `mat4x4_mul` | `A=T01·T12`, `B=T23` | `T01·T12·T23` |
| `b2v_inst_mul3` | `mat4x4_mul` | `A=T01·T12·T23`, `B=T34` | `T01·T12·T23·T34` |
| `b2v_inst_mul4` | `mat4x4_mul` | `A=(...)·T34`, `B=T45` | `T05` (resultado final, en `c_r0c0..c_r3c3`) |

### Diagrama de bloques

```
theta1,cx1,sx1,lz1,lx1 -> [t_dh_gen T01] --\
                                              [mat4x4_mul#1] -> T01.T12 --\
theta2,cx2,sx2,lz2,lx2 -> [t_dh_gen T12] --/                                \
                                                                              [mat4x4_mul#2] -> T01.T12.T23 --\
theta3,cx3,sx3,lz3,lx3 -----------------> [t_dh_gen T23] -------------------/                                   \
                                                                                                                   [mat4x4_mul#3] -> (...)·T34 --\
theta4,cx4,sx4,lz4,lx4 -----------------------------------------------------> [t_dh_gen T34] -------------------/                                \
                                                                                                                                                    [mat4x4_mul#4] -> T05 = C
theta5,cx5,sx5,lz5,lx5 -----------------------------------------------------------------------------------------> [t_dh_gen T45] -------------------/
```

Es una cadena de multiplicación **secuencial** (no un árbol balanceado): cada producto
depende del anterior, por eso la latencia total crece linealmente con el número de
eslabones (ver sección 6).

---

## 5. Sincronización por AND: una alternativa más simple

En la lección 6, `fk2r_pipeline_core` sincronizaba los `done` de los generadores de matriz
con latches explícitos (`latch_t1`, `latch_t2`, `sync_valid.vhd`). Aquí, el esquemático usa
una compuerta **AND combinacional simple** para lanzar cada multiplicación:

```vhdl
SYNTHESIZED_WIRE_8  <= SYNTHESIZED_WIRE_0 AND SYNTHESIZED_WIRE_1;   -- done(T01) AND done(T12) -> start mul1
SYNTHESIZED_WIRE_41 <= SYNTHESIZED_WIRE_2 AND SYNTHESIZED_WIRE_3;   -- done(mul1) AND done(T23) -> start mul2
SYNTHESIZED_WIRE_74 <= SYNTHESIZED_WIRE_4 AND SYNTHESIZED_WIRE_5;   -- done(mul2) AND done(T34) -> start mul3
SYNTHESIZED_WIRE_107<= SYNTHESIZED_WIRE_6 AND SYNTHESIZED_WIRE_7;   -- done(mul3) AND done(T45) -> start mul4
```

**¿Por qué esto funciona sin necesitar latches, a diferencia de la lección 6?** Porque tanto
`t_dh_gen.done` como `mat4x4_mul.done` son señales que **se mantienen en alto indefinidamente**
una vez que se activan (ambas solo bajan cuando llega un nuevo `start`, ver secciones 3 y 4 de
la lección 6). Si dos señales de nivel sostenido llegan en ciclos distintos, un AND
combinacional simple igual detecta correctamente el momento en que **ambas** están activas —
no hace falta "recordar" que una ya llegó, porque literalmente sigue en alto. Los latches de
`fk2r_pipeline_core` son una capa de seguridad adicional, no estrictamente necesaria dado este
comportamiento de los `done`; el esquemático de `Fk6R_DH` demuestra la versión mínima que
aprovecha esa propiedad directamente.

> Cuidado: este patrón (AND de señales de nivel sostenido) solo es seguro para **un cálculo a
> la vez** por `start` — no es apto para un pipeline continuo donde se lanzan cálculos nuevos
> antes de que el anterior termine, porque un `done` viejo que no se ha limpiado con un nuevo
> `start` podría dar un falso positivo. Tanto `fk2r_pipeline_core` como `Fk6R_DH` asumen ese
> uso "un cálculo por vez", consistente con el protocolo `start`/`done` de todo el semillero.

---

## 6. Testbench y casos de prueba

`fk6r_tb.vhd` fija los parámetros de los 5 eslabones de la tabla de la sección 2 como
constantes, y corre 5 casos con `run_caso(theta1..theta5, nombre)`, reportando la matriz `T05`
completa (4 filas) al final de cada caso:

| Caso | Ángulos (`θ1..θ5`) | Descripción |
|---|---|---|
| 1 | `0,0,0,0,0` | Posición *home* del robot |
| 2 | `π/2,0,0,0,0` | Solo gira la base |
| 3 | `π/2,π/2,0,0,0` | Base + hombro |
| 4 | `0,0,0,π/2,0` | Solo la muñeca (`θ4`) |
| 5 | `π/4,π/4,π/4,π/4,π/4` | Todas las juntas a 45° |

Con timeout de **300 ciclos** por caso (más holgado que el de la lección 6, porque la
latencia real de este proyecto es mayor — ver comparación en la sección 8) y 30 ciclos de
espera entre casos para drenar el pipeline.

---

## 7. Cómo compilar y simular

### Compilar en Quartus

1. Abrir `Fk6R_DH.qpf`.
2. Top-level ya configurado (`Fk6R_DH` en `Fk6R_DH.qsf`).
3. Para cambiar la arquitectura (agregar/quitar eslabones, por ejemplo), editar
   `Fk6R_DH.bdf` en el editor de esquemáticos — nunca el `.vhd` generado.

### Simular el testbench

Mismo procedimiento que en [`2_Configuracion_Quartus_y_Simulacion`](../2_Configuracion_Quartus_y_Simulacion/README.md):
compilar todos los `.vhd` del proyecto junto con `fk6r_tb.vhd` en ModelSim/Questa y correr
`fk6r_tb` como entidad de simulación. Comparar los valores de `T05` reportados contra los
generados por `Verificador_DH.m` (lección 8) para el mismo conjunto de ángulos — deben
coincidir dentro de la tolerancia del CORDIC (±10 en Q2.13, según nota del propio script
MATLAB).

---

## 8. Comparación: DH vs. matrices homogéneas

| | [Lección 6 — Matrices homogéneas](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) | Lección 7 — Denavit-Hartenberg (esta) |
|---|---|---|
| Robot | 2R planar (3 transformaciones encadenadas) | 5 eslabones revolutos (5 transformaciones encadenadas) |
| Generador de matriz | `t_matrix_gen` (rotación Z + traslación X/Z) | `t_dh_gen` (rotación Z + traslación X/Z + torsión `α` precomputada) |
| Latencia por matriz | 14 ciclos | 15 ciclos (1 más, por el producto adicional `Cx·sin`/`Sx·sin`) |
| Multiplicaciones en cadena | 2 (`T12`, luego `T123`) | 4 (`T01·T12`, `·T23`, `·T34`, `·T45`) |
| Latencia total | ~18 ciclos | 15 + 4×2 = **23 ciclos** (460 ns a 50 MHz) |
| Sincronización de `done` | Latches explícitos (`latch_t1`, `latch_t2`) | AND combinacional directo (ver sección 5) |
| Parámetros de entrada | Específicos del robot (`theta1`, `theta2`, `L1`, `L2`, `h1`) en `fk2r_top`; genéricos (`theta_i`, `tx_i`, `tz_i`) en el esquemático | Genéricos DH (`theta_i`, `cx_i`, `sx_i`, `lz_i`, `lx_i`) — escalan a cualquier número de eslabones sin cambiar el módulo base |

La ventaja práctica de DH se ve al escalar: agregar un sexto eslabón al robot de esta lección
es instanciar un `t_dh_gen` y un `mat4x4_mul` más en el esquemático, con los mismos 5 puertos
de siempre. Hacer lo mismo con el método de matrices homogéneas "a mano" (lección 6)
requeriría decidir manualmente qué representa la traslación y la rotación de cada nuevo
eslabón, como se hizo específicamente para `T1_0`, `T2_1`, `T3_2` en `fk2r_pipeline_core`.

---

*Documentación generada para Quartus 18.1 · Familia Cyclone IV E · Reloj de sistema 50 MHz.
Ver también: [Cinemática 2R por matrices homogéneas (lección 6)](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) ·
[Verificación en MATLAB con Peter Corke (lección 8)](../8_MATLAB_Peter_Corke_Cinematica/README.md)*
