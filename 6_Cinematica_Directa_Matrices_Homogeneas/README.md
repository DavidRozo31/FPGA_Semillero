# Cinemática Directa 2R por Matrices Homogéneas — Proyecto `Fk2R_directa`

## Descripción General

Este módulo calcula la **cinemática directa** de un robot planar de 2 grados de libertad (2R)
en hardware, usando el método clásico de **matrices de transformación homogénea 4×4**
(sin pasar por parámetros Denavit-Hartenberg). Dadas las posiciones angulares de las dos
juntas y las longitudes de los eslabones, el circuito entrega la posición `(px, py)` del
efector final y su orientación, en punto fijo Q2.13.

Este proyecto reutiliza directamente los operadores aritméticos y el CORDIC seno/coseno
documentados en [`5_2R_Inverse_Cinematic/Operadores.md`](../5_2R_Inverse_Cinematic/Operadores.md)
(`fp_adder`, `fp_multiplier`, `cordic_sincos_16`, `cordic_pkg`) — **no se repiten aquí**, solo
se referencian. Esta lección se enfoca en lo nuevo: cómo se combinan esos operadores para
construir y encadenar matrices homogéneas.

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo), igual que en el resto del semillero.
> **Plataforma:** Cyclone IV E (EP4CE6E22C8) · Quartus 18.1
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Teoría: la cadena de transformaciones homogéneas](#1-teoría-la-cadena-de-transformaciones-homogéneas)
2. [Dos arquitecturas en el mismo proyecto](#2-dos-arquitecturas-en-el-mismo-proyecto)
3. [Módulo nuevo: `t_matrix_gen` — generador de una matriz T](#3-módulo-nuevo-t_matrix_gen--generador-de-una-matriz-t)
4. [Módulo nuevo: `mat4x4_mul` — multiplicador de matrices 4×4](#4-módulo-nuevo-mat4x4_mul--multiplicador-de-matrices-4×4)
5. [Módulo nuevo: `sync_valid`](#5-módulo-nuevo-sync_valid)
6. [Orquestador: `fk2r_pipeline_core`](#6-orquestador-fk2r_pipeline_core)
7. [Interfaz de alto nivel: `fk2r_top`](#7-interfaz-de-alto-nivel-fk2r_top)
8. [Top-level real de Quartus: `Fk2R_directa` (esquemático)](#8-top-level-real-de-quartus-fk2r_directa-esquemático)
9. [Testbench y casos de prueba](#9-testbench-y-casos-de-prueba)
10. [Cómo compilar y simular](#10-cómo-compilar-y-simular)
11. [Resumen visual de la arquitectura](#11-resumen-visual-de-la-arquitectura)

---

## 1. Teoría: la cadena de transformaciones homogéneas

Una matriz de transformación homogénea 4×4 describe la posición y orientación de un sistema
de referencia respecto a otro:

```
        [ R11  R12  R13  Px ]
T   =   [ R21  R22  R23  Py ]
        [ R31  R32  R33  Pz ]
        [  0    0    0   1  ]
```

Donde la submatriz 3×3 superior izquierda `R` es la rotación y la columna `[Px Py Pz]` es la
traslación. Para un robot serial, la pose del efector final respecto a la base se obtiene
**encadenando** (multiplicando) una matriz por cada eslabón:

```
T_efector_base = T1_0 · T2_1 · T3_2 · ... · Tn_(n-1)
```

Para el robot 2R de este proyecto, con 3 eslabones conceptuales (junta 1, junta 2, y el
segmento final hasta el efector), la cadena es:

```
T3_0 = T1_0 · T2_1 · T3_2
```

| Transformación | Significado físico | Parámetros |
|---|---|---|
| `T1_0` | De la base a la junta 1 | rotación `theta1` alrededor de Z, traslación `h1` en Z (altura de la base) |
| `T2_1` | De la junta 1 a la junta 2 | rotación `theta2` alrededor de Z, traslación `L1` en X (eslabón 1) |
| `T3_2` | De la junta 2 al efector final | rotación `0` (traslación pura), traslación `L2` en X (eslabón 2) |

Cada una de estas matrices individuales tiene la forma general que genera el módulo
`t_matrix_gen` (ver sección 3):

```
        [ cos(θ)  -sin(θ)   0   tx·cos(θ) ]
T(θ) =  [ sin(θ)   cos(θ)   0      0      ]
        [   0        0      1     tz      ]
        [   0        0      0      1      ]
```

> Nota: en `T1_0` y `T2_1` la traslación en X ya viene multiplicada implícitamente por la
> disposición de ejes del robot — en la práctica, `t_matrix_gen` recibe `tx_in` y `tz_in`
> como valores ya calculados por el bloque que lo instancia (ver sección 6), no aplica
> `tx·cos(θ)` internamente. Esto simplifica el hardware a costa de que quien instancia el
> módulo debe saber qué representa cada parámetro para su eslabón concreto.

Al final, de la matriz resultante `T3_0` se extrae directamente lo que interesa para control:

- **Posición del efector:** columna 3, filas 0 y 1 → `(px, py) = (T3_0[0][3], T3_0[1][3])`
- **Orientación del efector:** submatriz de rotación 2×2 superior izquierda → `(r00, r01, r10, r11)`,
  que en este robot planar equivale a `cos(θ1+θ2)` y `sin(θ1+θ2)`.

---

## 2. Dos arquitecturas en el mismo proyecto

Al explorar `Fk2R_directa/` aparecen **dos implementaciones distintas** del mismo problema.
Es importante entender cuál es cuál para no confundirse leyendo el código:

| Archivo | Qué es | ¿Es el top-level real? |
|---|---|---|
| `Fk2R_directa.vhd` | Generado automáticamente desde el esquemático `Fk2R_directa.bdf`. Robot **genérico** de 3 transformaciones encadenadas (`theta1/2/3`, `tx1/2/3`, `tz1/2/3` libres) | **Sí** — declarado como `TOP_LEVEL_ENTITY` en `Fk2R_directa.qsf` y es lo que instancia el testbench `fk2r_tb.vhd` |
| `fk2r_top.vhd` + `fk2r_pipeline_core.vhd` | Escritos a mano, interfaz específica para el robot 2R real (`theta1_in`, `theta2_in`, `l1_in`, `l2_in`, `h1_in`), con protocolo `start`/`done` y salida byte a byte | No — es una interfaz de más alto nivel, pensada para integrarse más adelante, pero **no está conectada como top-level de Quartus todavía** |

Ambas comparten los mismos bloques internos (`t_matrix_gen`, `mat4x4_mul`), así que documentar
uno documenta prácticamente el otro. Esta lección explica primero los bloques compartidos
(secciones 3-5), luego el orquestador escrito a mano `fk2r_pipeline_core` (sección 6, el más
fácil de leer porque es específico del robot 2R) y finalmente el esquemático genérico que
realmente se sintetiza (sección 8).

---

## 3. Módulo nuevo: `t_matrix_gen` — generador de una matriz T

Genera **una** matriz homogénea individual `T(θ, tx, tz)` a partir de un ángulo y dos
desplazamientos:

```vhdl
entity t_matrix_gen is
    Port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        theta_in : in  std_logic_vector(15 downto 0);
        tx_in    : in  std_logic_vector(15 downto 0);
        tz_in    : in  std_logic_vector(15 downto 0);
        t_r0c0 .. t_r3c3 : out std_logic_vector(15 downto 0);  -- 16 salidas, una por celda
        done : out std_logic
    );
end t_matrix_gen;
```

### Cómo se construye la matriz

Internamente solo hay dos piezas de trabajo real:

1. **`cordic_sincos_16`** (documentado en la lección 5) recibe `theta_in` y calcula `sin`/`cos`
   en Q2.13, con una latencia de 13 ciclos.
2. Un **pipeline de registros** (`tx_pipe`, `tz_pipe`, arreglo de 14 posiciones) retrasa
   `tx_in` y `tz_in` exactamente esos mismos 13 ciclos, para que lleguen sincronizados con
   `sin`/`cos` cuando el CORDIC termina:

```vhdl
process(clk, rst)
begin
    ...
    elsif rising_edge(clk) then
        tx_pipe(0) <= signed(tx_in);
        tz_pipe(0) <= signed(tz_in);
        for i in 1 to 13 loop
            tx_pipe(i) <= tx_pipe(i-1);
            tz_pipe(i) <= tz_pipe(i-1);
        end loop;
    end if;
end process;
```

Cuando `cordic_sincos_16` señala `done`, se capturan `cos`, `sin`, `tx_pipe(13)` y `tz_pipe(13)`
en registros de salida (`cos_reg`, `sin_reg`, `tx_reg`, `tz_reg`), y con ellos se arma la
matriz completa de forma puramente combinacional:

```vhdl
t_r0c0 <= std_logic_vector(cos_reg);        t_r0c1 <= std_logic_vector(-sin_reg);
t_r0c2 <= ZERO_Q13;                         t_r0c3 <= std_logic_vector(tx_reg);

t_r1c0 <= std_logic_vector(sin_reg);        t_r1c1 <= std_logic_vector(cos_reg);
t_r1c2 <= ZERO_Q13;                         t_r1c3 <= ZERO_Q13;

t_r2c0 <= ZERO_Q13;   t_r2c1 <= ZERO_Q13;   t_r2c2 <= ONE_Q13_SLV;   t_r2c3 <= std_logic_vector(tz_reg);
t_r3c0 <= ZERO_Q13;   t_r3c1 <= ZERO_Q13;   t_r3c2 <= ZERO_Q13;      t_r3c3 <= ONE_Q13_SLV;
```

Esto coincide exactamente con la forma general de `T(θ)` de la sección 1: fila 2 y 3 son la
identidad homogénea estándar (sin rotación fuera del plano XY), y la traslación en Z (`tz`) va
en `t_r2c3`.

### Latencia

**14 ciclos**: 13 del CORDIC + 1 ciclo de registro de salida.

---

## 4. Módulo nuevo: `mat4x4_mul` — multiplicador de matrices 4×4

Multiplica dos matrices 4×4 en Q2.13: `C = A × B`.

```vhdl
entity mat4x4_mul is
    Port (
        clk, rst, start : in std_logic;
        a_r0c0 .. a_r3c3 : in  std_logic_vector(15 downto 0);  -- 16 entradas, matriz A
        b_r0c0 .. b_r3c3 : in  std_logic_vector(15 downto 0);  -- 16 entradas, matriz B
        c_r0c0 .. c_r3c3 : out std_logic_vector(15 downto 0);  -- 16 salidas, matriz C
        done : out std_logic
    );
end mat4x4_mul;
```

### Pipeline de 2 etapas

**Etapa 1** (`v1 <= start`): en el ciclo donde `start = '1'`, se capturan las 32 entradas
(16 de A + 16 de B) en registros `signed`.

**Etapa 2** (`if v1 = '1'`): en el ciclo siguiente, se calculan las 16 celdas de `C` en
paralelo. Cada celda es el producto punto de una fila de A por una columna de B, usando la
función `mul_q13` del paquete `ik_pkg` (multiplicación Q2.13 con reescalado, ver
[Operadores.md](../5_2R_Inverse_Cinematic/Operadores.md#el-problema-del-escalado)):

```vhdl
C00 <= mul_q13(A00,B00) + mul_q13(A01,B10) + mul_q13(A02,B20) + mul_q13(A03,B30);
C01 <= mul_q13(A00,B01) + mul_q13(A01,B11) + mul_q13(A02,B21) + mul_q13(A03,B31);
-- ... (16 celdas en total, mismo patrón fila x columna)
```

> **Nota de diseño:** a diferencia de `fp_multiplier`/`fp_adder` (lección 5), aquí la
> multiplicación y la suma de cada celda se hacen en una sola línea con operadores `signed`
> directos (`+`), no instanciando `fp_multiplier`/`fp_adder` como componentes separados. Es
> una multiplicación matricial "en línea": el sintetizador infiere 4 multiplicadores y 3
> sumadores en cascada por celda (16 celdas × 4 productos = 64 multiplicaciones Q2.13 en
> paralelo en un solo ciclo de reloj).

### `done` se mantiene en alto hasta el próximo `start`

```vhdl
-- Nuevo start baja el done anterior
if start = '1' then
    done_reg <= '0';
end if;
...
if v1 = '1' then
    ... -- calcula C
    done_reg <= '1';   -- se queda en '1' indefinidamente
end if;
```

Esto es deliberado (ver comentario de cabecera del archivo): permite que un `done` que llegó
temprano se pueda combinar con un AND lógico contra otro `done` que llega después, sin perder
el pulso — patrón que se explota en `fk2r_pipeline_core` (sección 6).

### Latencia

**2 ciclos**: 1 de captura de entradas + 1 de cálculo.

---

## 5. Módulo nuevo: `sync_valid`

Sincronizador genérico de dos señales `valid` que pueden llegar en ciclos distintos:

```vhdl
entity sync_valid is
    Port (
        clk, rst : in  std_logic;
        valid_a  : in  std_logic;
        valid_b  : in  std_logic;
        valid_out : out std_logic
    );
end sync_valid;
```

Retiene cada `valid` en un latch (`latch_a`, `latch_b`) apenas llega, y cuando **ambos** están
retenidos (o llegan en el mismo ciclo), emite un pulso de 1 ciclo en `valid_out` y limpia los
latches para el siguiente uso:

```vhdl
if (latch_a = '1' or valid_a = '1') and (latch_b = '1' or valid_b = '1') then
    valid_out <= '1';
    latch_a   <= '0';
    latch_b   <= '0';
else
    valid_out <= '0';
end if;
```

> **Nota:** este módulo existe en el proyecto pero `fk2r_pipeline_core` no lo instancia como
> componente — implementa la misma lógica de sincronización "en línea" con las señales
> `latch_t1`/`latch_t2`/`sync_t12` (ver sección 6). Es decir, `sync_valid.vhd` es la versión
> reutilizable/genérica de un patrón que el pipeline usa manualmente. Vale la pena conocer
> ambas formas: la genérica (componente aparte) y la especializada (lógica inline).

---

## 6. Orquestador: `fk2r_pipeline_core`

Este es el módulo que implementa literalmente `T3_0 = T1_0 · T2_1 · T3_2` para el robot 2R
específico (recibe `theta1_in`, `theta2_in`, `l1_in`, `l2_in`, `h1_in`).

### Secuencia del pipeline

```
Paso 1 (ciclo  0)  : start -> lanzar 3x t_matrix_gen en paralelo, para T1_0, T2_1, T3_2
Paso 2 (ciclo 14)  : done de T1_0 y T2_1 -> lanzar mat4x4_mul para T12 = T1_0 * T2_1
Paso 3 (ciclo 16)  : done de T12 -> lanzar mat4x4_mul para T123 = T12 * T3_2
Paso 4 (ciclo 18)  : done de T123 -> latch del resultado final
```

**Latencia total: ~18 ciclos** (14 del `t_matrix_gen` más lento + 2×2 ciclos de las dos
multiplicaciones en cascada).

### Los tres generadores en paralelo

Los tres `t_matrix_gen` se lanzan simultáneamente con el mismo `start`, cada uno con los
parámetros de su eslabón:

| Instancia | `theta_in` | `tx_in` | `tz_in` | Representa |
|---|---|---|---|---|
| `U_T1` | `theta1_in` | `0` | `h1_in` | `T1_0`: rotación de la base + altura |
| `U_T2` | `theta2_in` | `l1_in` | `0` | `T2_1`: rotación de junta 2 + longitud eslabón 1 |
| `U_T3` | `0` (constante) | `l2_in` | `0` | `T3_2`: traslación pura, longitud eslabón 2 |

`T3_2` usa `theta = 0` porque no hay rotación entre la junta 2 y el efector final — el CORDIC
con ángulo 0 da `cos=1, sin=0`, así que `t_matrix_gen` produce una matriz de traslación pura.

### Por qué hace falta sincronizar manualmente

Aunque los tres `t_matrix_gen` arrancan juntos y tienen la misma latencia (14 ciclos), el
código sincroniza sus `done` "por seguridad" antes de lanzar la primera multiplicación:

```vhdl
-- Retener cada done
if t1_done = '1' then latch_t1 <= '1'; end if;
if t2_done = '1' then latch_t2 <= '1'; end if;
...
-- Emitir pulso cuando T1 y T2 estan listos -> inicio de mult T12
sync_t12 <= '0';
if (latch_t1='1' or t1_done='1') and (latch_t2='1' or t2_done='1') then
    sync_t12 <= '1';
    latch_t1 <= '0';
    latch_t2 <= '0';
end if;
```

`T3_2` se calcula en paralelo pero no se usa hasta el **segundo** producto (`T123 = T12 · T3_2`),
así que su resultado se guarda en un latch (`T3_latch_r*`) mientras se espera a que termine la
primera multiplicación:

```vhdl
if t3_done = '1' then
    latch_t3 <= '1';
    T3_latch_r0c0 <= T3_r0c0;  -- ... y así las 16 celdas
end if;
```

### Las dos multiplicaciones en cascada

```vhdl
U_MUL12  : mat4x4_mul port map (start => sync_t12,  a => T1_0,  b => T2_1,  c => T12, ...);
U_MUL123 : mat4x4_mul port map (start => t12_done,  a => T12,   b => T3_latch, c => T123, ...);
```

La segunda multiplicación arranca directamente con `t12_done` (el `done` de `mat4x4_mul` se
mantiene en alto, ver sección 4, así que sirve directo como `start` de la siguiente etapa).

### Extracción del resultado final

```vhdl
if t123_done = '1' then
    px_reg  <= signed(T123_r0c3);   -- posicion X: columna 3, fila 0
    py_reg  <= signed(T123_r1c3);   -- posicion Y: columna 3, fila 1
    r00_reg <= signed(T123_r0c0);   -- submatriz de rotacion 2x2
    r01_reg <= signed(T123_r0c1);
    r10_reg <= signed(T123_r1c0);
    r11_reg <= signed(T123_r1c1);
    done_reg <= '1';
end if;
```

---

## 7. Interfaz de alto nivel: `fk2r_top`

Envuelve a `fk2r_pipeline_core` con un protocolo de uso claro y una salida alternativa byte a
byte, pensada para conectarse a un microcontrolador o bus de 8 bits (como los usados en
[`9_Control_Servos_UART`](../9_Control_Servos_UART/README.md)).

### Protocolo de uso

```
1. Colocar theta1_in, theta2_in, l1_in, l2_in, h1_in
2. Dar pulso start='1' durante 1 ciclo
3. Esperar done='1' (~18 ciclos)
4. Leer px_out, py_out (posición del efector en Q2.13)
5. (Opcional) leer r00_out..r11_out (orientación)
6. O usar data_out con out_sel para leer byte a byte
```

### Formato numérico Q2.13

```
valor_real = raw_int / 8192.0
```

16 bits con signo: 2 bits enteros + 13 bits de fracción. Rango aproximado `[-4.0, +3.9999]`,
resolución `1/8192 ≈ 0.000122`. Ejemplos:

| Valor real | Cálculo | `raw_int` (Q2.13) |
|---|---|---|
| `0.5 m` | `0.5 × 8192` | `4096` |
| `0.3 m` | `0.3 × 8192` | `2458` (redondeado) |
| `π/2 rad` | `1.5708 × 8192` | `12868` |
| `-1.0` | `-1.0 × 8192` | `-8192` |

### Salida byte a byte (`out_sel`)

| `out_sel` | `data_out` |
|---|---|
| `"000"` | `px` byte bajo |
| `"001"` | `px` byte alto |
| `"010"` | `py` byte bajo |
| `"011"` | `py` byte alto |
| `"100"` | `r00` byte bajo |
| `"101"` | `r00` byte alto |
| `"110"` | `r11` byte bajo |
| `"111"` | `r11` byte alto |

### Latch de resultados al pulso `done`

`fk2r_top` no expone directamente las salidas combinacionales de `fk2r_pipeline_core`: las
captura en registros (`px_lat`, `py_lat`, ...) en el ciclo donde `done_s = '1'`, y solo ahí
levanta su propio `done` de un ciclo. Esto asegura que las salidas permanecen estables
(no cambian a mitad de una lectura externa) hasta el siguiente `start`.

---

## 8. Top-level real de Quartus: `Fk2R_directa` (esquemático)

`Fk2R_directa.vhd` **no se escribe a mano**: Quartus lo genera automáticamente a partir del
diagrama de bloques `Fk2R_directa.bdf` cada vez que se guarda el esquemático. Por eso usa
nombres de señal genéricos (`SYNTHESIZED_WIRE_n`) y **no debe editarse directamente** — cualquier
cambio se pierde la próxima vez que se regenera desde el `.bdf`. Si hay que modificar la
arquitectura, se edita el esquemático en Quartus (Block Diagram Editor), no este archivo.

A diferencia de `fk2r_top`/`fk2r_pipeline_core` (que ya conocen que es un robot 2R con
`theta1`, `theta2`, `L1`, `L2`, `h1`), el esquemático es **genérico**: recibe 3 ternas
`(theta_i, tx_i, tz_i)` independientes y encadena `t_matrix_gen → t_matrix_gen → mat4x4_mul → mat4x4_mul`
sin asumir para qué se usa cada parámetro. Esto es lo que realmente compila el archivo
`.qsf` (`TOP_LEVEL_ENTITY Fk2R_directa`) y lo que instancia el testbench `fk2r_tb.vhd`.

### Interfaz (genérica, 3 transformaciones encadenadas)

```vhdl
ENTITY Fk2R_directa IS
    PORT (
        clk_in, rst_in, start_in : IN STD_LOGIC;
        theta1_in, theta2_in, theta3_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
        tx1_in, tx2_in, tx3_in          : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
        tz1_in, tz2_in, tz3_in          : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
        done_out : OUT STD_LOGIC;
        c_r0c0 .. c_r3c3 : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)  -- matriz resultado T3_0
    );
END Fk2R_directa;
```

### Cómo mapear el robot 2R a esta interfaz genérica (según el testbench)

| Entrada genérica | Valor usado para el robot 2R | Corresponde a |
|---|---|---|
| `theta1_in`, `tx1_in`, `tz1_in` | `theta1_in`, `0`, `0` | `T1_0` (sin `h1` en el testbench: base a nivel 0) |
| `theta2_in`, `tx2_in`, `tz2_in` | `theta2_in`, `L1 = 4096` (0.5 m), `0` | `T2_1` |
| `theta3_in`, `tx3_in`, `tz3_in` | `0`, `L2 = 2458` (0.3 m), `0` | `T3_2` |

### Diagrama de bloques del esquemático

```
theta1,tx1,tz1 -> [t_matrix_gen T01] --\
                                          [mat4x4_mul #1]--> T12 --\
theta2,tx2,tz2 -> [t_matrix_gen T12] --/                             [mat4x4_mul #2] --> C (=T3_0)
                                                                    /
theta3,tx3,tz3 -> [t_matrix_gen T23] -----------------------------/
```

(Las etiquetas `T01`, `T12`, `T23` son los nombres de instancia del `.bdf`, no coinciden 1:1
con la nomenclatura `T1_0`/`T2_1`/`T3_2` de la sección 1, pero representan la misma cadena de
3 transformaciones.)

---

## 9. Testbench y casos de prueba

`fk2r_tb.vhd` instancia `Fk2R_directa` (el esquemático, sección 8) con `L1 = 4096` (0.5 m) y
`L2 = 2458` (0.3 m), y corre 4 casos con la tarea `run_caso(theta1, theta2, nombre)`:

| Caso | `theta1` | `theta2` | `px` esperado | `py` esperado |
|---|---|---|---|---|
| 1 | `0` | `0` | `~6554` (800 mm) — brazo extendido en X | `~0` |
| 2 | `π/2` (`12868`) | `0` | `~0` | `~6554` (800 mm) — brazo apuntando en Y |
| 3 | `π/4` (`6434`) | `π/4` (`6434`) | `~2896` (354 mm) | `~5357` (654 mm) |
| 4 | `π/4` (`6434`) | `-π/4` (`-6434`) | `~5357` (654 mm) | `~2896` (354 mm) |

Cada caso sigue el protocolo estándar: coloca las entradas, da un pulso de `start_in` de
1 ciclo, espera `done_out` (con timeout de 100 ciclos por seguridad) y reporta el resultado
convertido de Q2.13 a milímetros:

```vhdl
REPORT "  px (c_r0c3) = " & INTEGER'IMAGE(to_integer(signed(c_r0c3))) &
       "  ->  " & INTEGER'IMAGE(to_integer(signed(c_r0c3)) * 1000 / 8192) & " mm";
```

Entre casos espera 30 ciclos adicionales para que el pipeline se drene completamente antes
del siguiente `start_in`.

---

## 10. Cómo compilar y simular

### Compilar en Quartus

1. Abrir `Fk2R_directa.qpf` en Quartus Prime (Lite Edition, probado con 18.1).
2. El top-level (`Fk2R_directa`) ya está configurado en `Fk2R_directa.qsf` — no requiere
   cambios para una compilación estándar (`Processing → Start Compilation`).
3. Si se necesita modificar la arquitectura del top-level, editar `Fk2R_directa.bdf` (no el
   `.vhd` generado, ver sección 8).

### Simular el testbench

Igual que en [`2_Configuracion_Quartus_y_Simulacion`](../2_Configuracion_Quartus_y_Simulacion/README.md):
compilar `fk2r_tb.vhd` junto con todos los `.vhd` del proyecto en ModelSim/Questa, y correr
`fk2r_tb` como entidad de simulación (no `Fk2R_directa`, que es el DUT). Observar en la
consola de transcript los 4 reportes de `px`/`py`/`r00..r11` por caso.

---

## 11. Resumen visual de la arquitectura

```
                         fk2r_pipeline_core (especifico robot 2R)
   theta1_in ----+
   h1_in     ----+--> [t_matrix_gen] --> T1_0 --\
                                                    \
   theta2_in ----+                                   [sync_t12] --> [mat4x4_mul] --> T12 --\
   l1_in     ----+--> [t_matrix_gen] --> T2_1 ------/                                          \
                                                                                                  [mat4x4_mul] --> T123 = T3_0
   l2_in     -------> [t_matrix_gen] --> T3_2 -------------------------------[latch]-----------/
   (theta=0)                                                                                  |
                                                                                                v
                                                                          px, py, r00, r01, r10, r11, done
```

```
                         fk2r_top (wrapper)
   theta1_in, theta2_in, l1_in, l2_in, h1_in, start -->[fk2r_pipeline_core]--> px_s..r11_s, done_s
                                                              |
                                                     [latch al pulso done_s]
                                                              |
                                    px_out, py_out, r00_out..r11_out, done  (directas, 16 bits)
                                                              |
                                              [mux out_sel] --> data_out (8 bits)
```

### Librerías utilizadas

| Librería | Paquete | Uso |
|---|---|---|
| `IEEE` | `STD_LOGIC_1164` | Tipos `std_logic`, `std_logic_vector` |
| `IEEE` | `NUMERIC_STD` | Tipo `signed`, aritmética Q2.13 |
| `work` | `ik_pkg` | Constantes Q2.13 (`PI_Q13`, `ONE_Q13`...) y `mul_q13` |
| `work` | `cordic_pkg` | Tabla de ángulos CORDIC, usada indirectamente vía `cordic_sincos_16` |

---

*Documentación generada para Quartus 18.1 · Familia Cyclone IV E · Reloj de sistema 50 MHz.
Ver también: [Operadores aritméticos (lección 5)](../5_2R_Inverse_Cinematic/Operadores.md) ·
[Cinemática DH del robot 6R (lección 7)](../7_Cinematica_Directa_DH/README.md)*
