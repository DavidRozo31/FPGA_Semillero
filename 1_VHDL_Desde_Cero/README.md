<h1 align="center">⚡ VHDL — Guía de Referencia</h1>

<p align="center">
  Lenguaje de descripción de hardware para modelar circuitos digitales ejecutados en paralelo,<br>
  impulsados por eventos o interrupciones (clock, entradas digitales, etc.)
</p>

<p align="center">
  <img src="https://img.shields.io/badge/VHDL-IEEE%201076-blueviolet?style=flat-square"/>
  <img src="https://img.shields.io/badge/Librería-IEEE%20%7C%20std%20%7C%20work-blue?style=flat-square"/>
  <img src="https://img.shields.io/badge/Herramientas-ModelSim%20%7C%20GHDL%20%7C%20Vivado-green?style=flat-square"/>
</p>

---

## Tabla de contenidos

1. [Introducción](#1-introducción)
2. [Estructura general](#2-estructura-general)
3. [Tipos de datos](#3-tipos-de-datos)
   - [Paquete `standard`](#31-paquete-standard--librería-std)
   - [Paquete `std_logic_1164`](#32-paquete-std_logic_1164--librería-ieee)
   - [Paquete `numeric_std`](#33-paquete-numeric_std--librería-ieee)
   - [Paquetes `fixed_pkg` y `float_pkg`](#34-paquetes-fixed_pkg-y-float_pkg--librería-ieee)
4. [Conversiones de tipos](#4-conversiones-de-tipos)
5. [Señales, variables y constantes](#5-señales-variables-y-constantes)
6. [Atributos](#6-atributos)
7. [Sentencias concurrentes](#7-sentencias-concurrentes)
8. [Sentencias secuenciales](#8-sentencias-secuenciales)
9. [Subprogramas](#9-subprogramas)
10. [Componentes](#10-componentes)
11. [Bibliotecas](#11-bibliotecas)
12. [Ejemplos completos](#12-ejemplos-completos)
13. [Testbench](#13-testbench)

---

## 1. Introducción

VHDL = **VHSIC** (*Very High Speed Integrated Circuit*) + **HDL** (*Hardware Description Language*).

> [!IMPORTANT]
> En VHDL **no se distingue** entre minúsculas y mayúsculas.

---

## 2. Estructura general

Un código VHDL tiene dos partes fundamentales: la **Entidad** (interfaz) y la **Arquitectura** (comportamiento).

```vhdl
LIBRARY library_name;
USE library_name.package_name.package_parts;

ENTITY entity_name IS
    PORT(
        port1_name : port_mode port_type;
        port2_name : port_mode port_type;
        portN_name : port_mode port_type
    );
END entity_name;

ARCHITECTURE architecture_name OF entity_name IS
    -- Señales y constantes globales
BEGIN
    -- Sentencias concurrentes

    PROCESS(lista_sensitiva)
        -- Variables locales
    BEGIN
        -- Sentencias secuenciales
    END PROCESS;

END architecture_name;
```

> [!NOTE]
> - Las **señales** son globales a la arquitectura.
> - Las **variables** son locales a cada proceso.
> - Las **constantes** y subprogramas pueden ser globales o locales según dónde se declaren.

### Modos de puerto

| Modo | Dirección | Descripción |
|------|-----------|-------------|
| `IN` | → entrada | Solo lectura. No puede leerse dentro de la arquitectura. |
| `OUT` | ← salida | Solo escritura. No puede retroalimentarse. |
| `INOUT` | ↔ bidireccional | Lectura y escritura (buses, memorias). |
| `BUFFER` | ↑ salida con feedback | Salida que puede leerse internamente. |

---

## 3. Tipos de datos

En VHDL existen diferentes tipos de datos para almacenar bits, enteros, punto fijo y punto flotante, organizados en paquetes según la librería de origen.

### 3.1 Paquete `standard` — Librería `std`

> [!IMPORTANT]
> No requiere importar ninguna librería.

| Tipo | Valores / Rango | Descripción |
|------|----------------|-------------|
| `boolean` | `false`, `true` | Valor lógico booleano |
| `bit` | `'0'`, `'1'` | Un bit lógico |
| `bit_vector` | vector de bits | Sin representación numérica |
| `integer` | −2³² a 2³²−1 | Entero con signo (32 bits por defecto) |
| `natural` | ≥ 0 | Subtipo de `integer` |
| `positive` | > 0 | Subtipo de `integer` |
| `time` | — | Representa retardos (`ns`, `us`, `ms`) |

```vhdl
flag     : boolean := false;

a        : bit := '1';
b        : bit := '0';

data     : bit_vector(3 downto 0) := "1010";

n        : integer := 532;
m        : integer := -532;
contador : integer range 0 to 100;   -- rango parametrizado (recomendado)

nat      : natural  := 0;
pos      : positive := 1;

t1       : time := 10 ns;
```

> [!IMPORTANT]
> El rango de `integer` es $-2^{32}$ a $2^{32}-1$. Sin embargo, es recomendable parametrizar el tamaño: `range 0 to 255`.

---

### 3.2 Paquete `std_logic_1164` — Librería IEEE

> [!IMPORTANT]
> Requiere importar: `library ieee; use ieee.std_logic_1164.all;`

Define **9 estados lógicos** para modelar el comportamiento físico real:

| Estado | Nombre | Descripción |
|--------|--------|-------------|
| `'U'` | Uninitialized | Valor por defecto al no inicializar |
| `'X'` | Unknown | Conflicto entre drivers |
| `'0'` | Logic 0 | Lógica baja (fuerte) |
| `'1'` | Logic 1 | Lógica alta (fuerte) |
| `'Z'` | High impedance | Bus tristate |
| `'W'` | Weak unknown | Desconocido débil |
| `'L'` | Weak 0 | Pull-down |
| `'H'` | Weak 1 | Pull-up |
| `'-'` | Don't care | Para optimización |

```vhdl
library ieee;
use ieee.std_logic_1164.all;

-- std_logic: un bit con 9 estados posibles
a    : std_logic := '1';
b    : std_logic := 'Z';                          -- alta impedancia

-- std_logic_vector: sin representación numérica, no permite aritmética directa
bus8 : std_logic_vector(7 downto 0) := (others => '0');  -- "00000000"
bus4 : std_logic_vector(3 downto 0) := "1010";
```

---

### 3.3 Paquete `numeric_std` — Librería IEEE

> [!IMPORTANT]
> Requiere importar: `library ieee; use ieee.numeric_std.all;`

Permite definir números binarios con soporte aritmético completo: `+`, `-`, `*`, `/`, comparaciones y desplazamientos (`shift_left`, `shift_right`, `rotate_left`, `rotate_right`).

| Tipo | Rango (n bits) | Descripción |
|------|---------------|-------------|
| `unsigned` | 0 a 2ⁿ−1 | Entero sin signo |
| `signed` | −2ⁿ⁻¹ a 2ⁿ⁻¹−1 | Entero con signo (complemento a 2) |

```vhdl
library ieee;
use ieee.numeric_std.all;

-- unsigned: solo positivos
a : unsigned(7 downto 0) := "11111111";  -- 255
b : unsigned(7 downto 0) := "00001010";  -- 10

-- signed: positivos y negativos (complemento a 2, MSB = signo)
c : signed(7 downto 0) := "11111111";   -- -1
d : signed(7 downto 0) := "01111111";   -- +127
```

> [!NOTE]
> Si el MSB es `'1'` en `signed`, el valor se determina con complemento a 2.

---

### 3.4 Paquetes `fixed_pkg` y `float_pkg` — Librería IEEE

#### Punto fijo

Permite definir la cantidad de bits para la parte entera y la parte decimal. Soporta operaciones aritméticas (`+`, `-`, `*`, `/`), comparaciones y conversiones.

```vhdl
library ieee;
use ieee.fixed_pkg.all;

-- sfixed(enteros downto -fraccionales)
a : sfixed(3 downto -4) := to_sfixed(1.375, a'high, a'low);  -- 4 bits enteros + 4 fraccionales
b : ufixed(2 downto -5) := to_ufixed(2.5, b'high, b'low);    -- 3 bits enteros + 5 fraccionales
```

#### Punto flotante

Números IEEE 754: 1 bit de signo + 8 bits de exponente + 23 bits de mantisa (float32).

```vhdl
library ieee;
use ieee.float_pkg.all;

a : float32 := to_float(3.14, float32);
b : float64;
b <= to_float(2.718, float64);
```

---

## 4. Conversiones de tipos

VHDL es **fuertemente tipado**: los tipos deben coincidir o convertirse explícitamente.

> [!NOTE]
> - Usa `std_logic_vector` para **buses, registros y puertos** (y para concatenar con `&`).
> - Usa `signed` / `unsigned` para **operar aritméticamente**.

### Tabla de conversiones

| Desde | Hacia | Función / Cast |
|-------|-------|----------------|
| `std_logic_vector` | `unsigned` | `unsigned(slv)` |
| `std_logic_vector` | `signed` | `signed(slv)` |
| `unsigned` | `std_logic_vector` | `std_logic_vector(u)` |
| `signed` | `std_logic_vector` | `std_logic_vector(s)` |
| `unsigned` | `integer` | `to_integer(u)` |
| `signed` | `integer` | `to_integer(s)` |
| `integer` | `unsigned` | `to_unsigned(n, N_bits)` |
| `integer` | `signed` | `to_signed(n, N_bits)` |

```vhdl
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

signal slv : std_logic_vector(7 downto 0);
signal u   : unsigned(7 downto 0);
signal s   : signed(7 downto 0);
signal n   : integer;

-- std_logic_vector → unsigned / signed
u   <= unsigned(slv);
s   <= signed(slv);

-- unsigned / signed → std_logic_vector
slv <= std_logic_vector(u);
slv <= std_logic_vector(s);

-- unsigned / signed → integer
n   <= to_integer(u);
n   <= to_integer(s);

-- integer → unsigned / signed (especificar cantidad de bits)
u   <= to_unsigned(n, 8);
s   <= to_signed(n, 8);
```

---

## 5. Señales, variables y constantes

### Comparativa

| | `CONSTANT` | `VARIABLE` | `SIGNAL` |
|---|---|---|---|
| **Operador de asignación** | `:=` | `:=` | `<=` |
| **Ámbito** | Global o local | Local al proceso | Global a la arquitectura |
| **Actualización** | Fija (no cambia) | Inmediata | Siguiente delta cycle |
| **Representa** | Valor fijo | Almacenamiento temporal | Cable físico |
| **Declaración** | Antes del `BEGIN` | Dentro del proceso | Antes del `BEGIN` |

> [!IMPORTANT]
> Las señales (`<=`) se actualizan al **final del delta cycle**. Las variables (`:=`) se actualizan **inmediatamente** dentro del proceso.

### Constantes

```vhdl
-- Declaradas en la arquitectura, antes del BEGIN. No modificables.
CONSTANT N_BITS  : integer   := 8;
CONSTANT CLK_DIV : integer   := 50_000_000;  -- 50 MHz
CONSTANT RESET   : std_logic := '0';
```

### Variables

```vhdl
-- Declaradas dentro de PROCESS, antes del BEGIN del proceso
PROCESS(clk)
    VARIABLE temp : integer := 0;
    VARIABLE acc  : unsigned(15 downto 0) := (others => '0');
BEGIN
    temp := temp + 1;           -- actualización inmediata
    acc  := (others => '0');
END PROCESS;
```

### Señales

```vhdl
-- Declaradas en la arquitectura, antes del BEGIN
-- Si no se inicializan, toman el estado 'U' por defecto
SIGNAL sig1  : std_logic := '0';
SIGNAL bus8  : std_logic_vector(7 downto 0);
SIGNAL cnt   : unsigned(3 downto 0) := (others => '0');
SIGNAL n_sig : integer range 0 to 255 := 0;

-- Asignación de señal (ocurre en el siguiente delta cycle)
sig1 <= '1';
bus8 <= "10101010";
cnt  <= cnt + 1;

-- Concatenación con &
bus8 <= sig1 & "0101010";  -- MSB=sig1, resto fijo
```

---

## 6. Atributos

| Atributo | Aplica a | Descripción |
|----------|----------|-------------|
| `'event` | Señales | `true` si la señal tuvo un evento en este delta |
| `'range` | Vectores | Retorna el rango completo del vector |
| `'high` | Vectores / escalares | Índice o valor más alto |
| `'low` | Vectores / escalares | Índice o valor más bajo |
| `'length` | Vectores | Número de elementos |

```vhdl
-- Flanco de subida — dos formas equivalentes
IF clk = '1' AND clk'event THEN ...   -- forma clásica
IF rising_edge(clk)        THEN ...   -- forma recomendada (IEEE 1076-2008)

-- Flanco de bajada
IF falling_edge(clk) THEN ...

-- Iterar sobre el rango completo de un vector
FOR i IN wire1'range LOOP ...
-- equivale a: FOR i IN 7 DOWNTO 0 LOOP (para wire1 de 8 bits)

-- Tamaño dinámico
N := bus_data'length;   -- 8 si bus_data es (7 downto 0)
```

---

## 7. Sentencias concurrentes

Se ejecutan en **paralelo** y de forma asíncrona. Modelan **lógica combinacional**.

> [!WARNING]
> Para evitar **latches** en `with-select` y `when-else`, siempre incluye `WHEN OTHERS` o `ELSE` final.

### `with-select`

Equivale a un `case` para asignación concurrente. Todas las condiciones deben ser mutuamente excluyentes.

```vhdl
WITH signal_condition SELECT
    signal_out <= signal1 WHEN value1,
                  signal2 WHEN value2,
                  signal3 WHEN value3,
                  signalN WHEN OTHERS;   -- evita latch
```

**Ejemplo — Multiplexador 4:1:**

```vhdl
WITH addr SELECT
    Q <= A WHEN "00",
         B WHEN "01",
         C WHEN "10",
         D WHEN OTHERS;
```

### `when-else`

Permite condiciones más complejas (no solo igualdad). Las condiciones se evalúan en orden (prioridad).

```vhdl
signal_out <= signal1 WHEN condition1 ELSE
              signal2 WHEN condition2 ELSE
              signal3 WHEN condition3 ELSE
              signalN;   -- ELSE final obligatorio
```

**Ejemplo — Multiplexador 4:1:**

```vhdl
Q <= A WHEN addr = "00" ELSE
     B WHEN addr = "01" ELSE
     C WHEN addr = "10" ELSE
     D;
```

### Proceso (`PROCESS`)

Los procesos se ejecutan en **paralelo entre sí**, pero el código interno es **secuencial**. Se activan cuando una señal de la lista sensitiva cambia.

```vhdl
PROCESS(lista_sensitiva)
    -- Variables locales (constantes, variables, subprogramas)
BEGIN
    -- Sentencias secuenciales
END PROCESS;
```

### Sentencias de tiempo

#### `AFTER`

Asigna valores con retardo. Útil en testbenches.

```vhdl
-- Puede usarse en sentencias concurrentes o secuenciales
clk <= '0', '1' AFTER 10 ns, '0' AFTER 20 ns, '1' AFTER 30 ns;
rst <= '1', '0' AFTER 50 ns;
```

#### `WAIT`

Solo en sentencias secuenciales (dentro de procesos o procedimientos).

```vhdl
WAIT FOR 100 ns;               -- espera tiempo fijo
WAIT UNTIL rising_edge(clk);   -- espera condición
WAIT ON a, b;                  -- espera cambio en cualquiera de las señales
```

---

## 8. Sentencias secuenciales

Se ejecutan **dentro de procesos**, funciones o procedimientos. Modelan **lógica secuencial** (flip-flops, contadores, FSM).

### `IF — ELSIF — ELSE`

```vhdl
IF condition1 THEN
    sentence1;
ELSIF condition2 THEN
    sentence2;
ELSE
    sentenceN;    -- evita latch en lógica combinacional
END IF;
```

**Ejemplo — Flip-flop D con reset asíncrono:**

```vhdl
PROCESS(clk, rst)
BEGIN
    IF rst = '1' THEN              -- reset asíncrono (mayor prioridad)
        Q <= '0';
    ELSIF rising_edge(clk) THEN
        IF en = '1' THEN
            Q <= D;
        END IF;
    END IF;
END PROCESS;
```

> [!NOTE]
> Para evitar un latch en lógica combinacional, incluir siempre la condición `ELSE` o definir un valor por defecto antes del `IF`.

### `CASE — WHEN`

```vhdl
CASE expression IS
    WHEN value1 => sentence1;
    WHEN value2 => sentence2;
    WHEN OTHERS => sentenceN;   -- obligatorio para cubrir todos los casos
END CASE;
```

**Ejemplo — Decodificador BCD a 7 segmentos:**

```vhdl
CASE bcd IS
    WHEN "0000" => seg <= "1111110";  -- 0
    WHEN "0001" => seg <= "0110000";  -- 1
    WHEN "0010" => seg <= "1101101";  -- 2
    WHEN "0011" => seg <= "1111001";  -- 3
    WHEN OTHERS => seg <= "0000000";  -- apagado
END CASE;
```

### `FOR LOOP`

```vhdl
FOR index IN range LOOP
    sentences;
END LOOP;
```

> [!NOTE]
> El índice de los bucles `FOR` **no se declara**.

**Ejemplo — XOR bit a bit:**

```vhdl
FOR i IN 0 TO 7 LOOP
    result(i) <= data(i) XOR mask(i);
END LOOP;

-- Usando el rango del vector
FOR i IN data'range LOOP
    result(i) <= data(i) XOR mask(i);
END LOOP;
```

### `WHILE LOOP`

```vhdl
WHILE condition LOOP
    sentences;
END LOOP;
```

### Control de bucles

| Sentencia | Descripción |
|-----------|-------------|
| `EXIT` | Sale del bucle inmediatamente |
| `NEXT` | Salta a la siguiente iteración |
| `NULL` | No hace nada (placeholder) |

---

## 9. Subprogramas

### Funciones

Retornan un valor. Solo lectura de parámetros.

```vhdl
FUNCTION function_name (parameters) RETURN return_type IS
    -- declaraciones locales
BEGIN
    -- sentencias secuenciales
    RETURN value;
END FUNCTION;
```

**Ejemplo:**

```vhdl
FUNCTION max2(a, b : integer) RETURN integer IS
BEGIN
    IF a > b THEN
        RETURN a;
    ELSE
        RETURN b;
    END IF;
END FUNCTION;
```

### Procedimientos

No retornan valor. Pueden modificar parámetros `OUT` o `INOUT`.

```vhdl
PROCEDURE procedure_name (parameters) IS
    -- declaraciones locales
BEGIN
    -- sentencias secuenciales
END PROCEDURE;
```

**Ejemplo:**

```vhdl
PROCEDURE reset_bus(SIGNAL bus : OUT std_logic_vector) IS
BEGIN
    bus <= (others => '0');
END PROCEDURE;
```

---

## 10. Componentes

Permiten reutilizar entidades como bloques en diseños jerárquicos (**structural modeling**).

**Paso 1** — Declarar el componente (antes del `BEGIN` de la arquitectura):

```vhdl
COMPONENT component_name
    PORT(
        signal1 : mode type;
        signal2 : mode type;
        signalN : mode type
    );
END COMPONENT;
```

**Paso 2** — Instanciar y mapear puertos (después del `BEGIN`):

```vhdl
instance_name : component_name
    PORT MAP(
        port1 => signal1,
        port2 => signal2,
        portN => signalN
    );
```

**Ejemplo completo — Instanciar una compuerta AND:**

```vhdl
ARCHITECTURE structural OF TopLevel IS

    COMPONENT Gate_AND
        PORT(a, b : IN std_logic; c : OUT std_logic);
    END COMPONENT;

    SIGNAL w1, w2, w3 : std_logic;

BEGIN
    U1 : Gate_AND
        PORT MAP(a => w1, b => w2, c => w3);

    -- Instanciación directa (sin declarar componente)
    U2 : ENTITY work.Gate_AND(arch_Gate_AND)
        PORT MAP(a => w1, b => w2, c => w3);

END structural;
```

---

## 11. Bibliotecas

Las librerías son paquetes reutilizables en todos los proyectos.

```vhdl
LIBRARY library_name;
USE library_name.package_name.package_parts;
```

### Librerías más utilizadas

| Librería | Paquete | Contenido |
|----------|---------|-----------|
| *(ninguna)* | `standard` | `bit`, `boolean`, `integer`, `time`... |
| `ieee` | `std_logic_1164` | `std_logic`, `std_logic_vector` |
| `ieee` | `numeric_std` | `unsigned`, `signed`, conversiones |
| `ieee` | `fixed_pkg` | `sfixed`, `ufixed` |
| `ieee` | `float_pkg` | `float32`, `float64` |
| `std` | `textio` | Lectura/escritura de archivos (simulación) |
| `work` | — | Entidades del proyecto actual |

```vhdl
-- Importación típica para la mayoría de diseños
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;
```

---

## 12. Ejemplos completos

### Entidad

```vhdl
ENTITY entity_name IS
    PORT(
        port1_name : port_mode port_type;
        port2_name : port_mode port_type;
        portN_name : port_mode port_type
    );
END entity_name;
```

### Arquitectura

```vhdl
ARCHITECTURE architecture_name OF entity_name IS
    -- Constantes y señales globales
BEGIN
    -- Sentencias concurrentes
END architecture_name;
```

---

### Ejemplo 1 — Compuerta AND

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY Gate_AND IS
    PORT(a, b : IN  std_logic;
         c    : OUT std_logic);
END Gate_AND;

ARCHITECTURE arch_Gate_AND OF Gate_AND IS
BEGIN
    c <= a AND b;
END arch_Gate_AND;
```

---

### Ejemplo 2 — D Flip-Flop con reset síncrono

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY DFF IS
    PORT(
        clk, rst, D : IN  std_logic;
        Q, Qn       : OUT std_logic
    );
END DFF;

ARCHITECTURE behavioral OF DFF IS
BEGIN
    PROCESS(clk)
    BEGIN
        IF rising_edge(clk) THEN
            IF rst = '1' THEN
                Q  <= '0';
                Qn <= '1';
            ELSE
                Q  <= D;
                Qn <= NOT D;
            END IF;
        END IF;
    END PROCESS;
END behavioral;
```

---

### Ejemplo 3 — Contador ascendente/descendente parametrizable

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY Counter IS
    GENERIC(N : integer := 8);   -- parametrizable: 4, 8, 16 bits...
    PORT(
        clk : IN  std_logic;
        rst : IN  std_logic;
        up  : IN  std_logic;     -- '1' = subir, '0' = bajar
        cnt : OUT std_logic_vector(N-1 downto 0)
    );
END Counter;

ARCHITECTURE behavioral OF Counter IS
    SIGNAL count : unsigned(N-1 downto 0);
BEGIN
    PROCESS(clk, rst)
    BEGIN
        IF rst = '1' THEN
            count <= (others => '0');
        ELSIF rising_edge(clk) THEN
            IF up = '1' THEN
                count <= count + 1;
            ELSE
                count <= count - 1;
            END IF;
        END IF;
    END PROCESS;

    cnt <= std_logic_vector(count);
END behavioral;
```

---

### Ejemplo 4 — ALU 4 bits

| `op` | Operación |
|------|-----------|
| `"000"` | A + B |
| `"001"` | A − B |
| `"010"` | A AND B |
| `"011"` | A OR B |
| `"100"` | A XOR B |
| `"101"` | NOT A |
| otros | 0 |

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY ALU4 IS
    PORT(
        A, B : IN  std_logic_vector(3 downto 0);
        op   : IN  std_logic_vector(2 downto 0);
        Y    : OUT std_logic_vector(3 downto 0);
        zero : OUT std_logic
    );
END ALU4;

ARCHITECTURE behavioral OF ALU4 IS
    SIGNAL result : unsigned(3 downto 0);
BEGIN
    PROCESS(A, B, op)
    BEGIN
        CASE op IS
            WHEN "000"  => result <= unsigned(A) + unsigned(B);
            WHEN "001"  => result <= unsigned(A) - unsigned(B);
            WHEN "010"  => result <= unsigned(A AND B);
            WHEN "011"  => result <= unsigned(A OR  B);
            WHEN "100"  => result <= unsigned(A XOR B);
            WHEN "101"  => result <= unsigned(NOT A);
            WHEN OTHERS => result <= (others => '0');
        END CASE;
    END PROCESS;

    Y    <= std_logic_vector(result);
    zero <= '1' WHEN result = (others => '0') ELSE '0';
END behavioral;
```

---

### Ejemplo 5 — Máquina de estados (FSM Moore) — Detector de secuencia `"101"`

```
      x=0         x=1
S0 ──────> S0    S0 ──────> S1
S1 ──────> S2    S1 ──────> S1
S2 ──────> S0    S2 ──────> S3  ← detected = '1'
S3 ──────> S0    S3 ──────> S1
```

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY SeqDetector IS
    PORT(
        clk, rst, x : IN  std_logic;
        detected    : OUT std_logic
    );
END SeqDetector;

ARCHITECTURE behavioral OF SeqDetector IS
    TYPE state_type IS (S0, S1, S2, S3);
    SIGNAL state, next_state : state_type;
BEGIN

    -- Registro de estado (secuencial)
    PROCESS(clk, rst)
    BEGIN
        IF rst = '1' THEN
            state <= S0;
        ELSIF rising_edge(clk) THEN
            state <= next_state;
        END IF;
    END PROCESS;

    -- Lógica de transición (combinacional)
    PROCESS(state, x)
    BEGIN
        CASE state IS
            WHEN S0 => IF x = '1' THEN next_state <= S1; ELSE next_state <= S0; END IF;
            WHEN S1 => IF x = '0' THEN next_state <= S2; ELSE next_state <= S1; END IF;
            WHEN S2 => IF x = '1' THEN next_state <= S3; ELSE next_state <= S0; END IF;
            WHEN S3 => IF x = '1' THEN next_state <= S1; ELSE next_state <= S0; END IF;
        END CASE;
    END PROCESS;

    -- Salida Moore (depende solo del estado)
    detected <= '1' WHEN state = S3 ELSE '0';

END behavioral;
```

---

## 13. Testbench

El **testbench** se usa para simulación RTL en herramientas como ModelSim, GHDL o Vivado Simulator. Permite observar, depurar y analizar el comportamiento de señales y puertos.

> [!IMPORTANT]
> El testbench es una entidad **sin puertos** y su código **no es sintetizable**.

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY tb_Gate_AND IS
    -- Sin puertos
END tb_Gate_AND;

ARCHITECTURE sim OF tb_Gate_AND IS

    -- Declarar el componente a probar (DUT)
    COMPONENT Gate_AND
        PORT(a, b : IN std_logic; c : OUT std_logic);
    END COMPONENT;

    -- Señales de estímulo y observación
    SIGNAL tb_a, tb_b, tb_c : std_logic;

BEGIN
    -- Instanciar el DUT
    DUT : Gate_AND PORT MAP(a => tb_a, b => tb_b, c => tb_c);

    -- Proceso de estímulos
    PROCESS
    BEGIN
        tb_a <= '0'; tb_b <= '0'; WAIT FOR 10 ns;  -- esperado: c = '0'
        tb_a <= '0'; tb_b <= '1'; WAIT FOR 10 ns;  -- esperado: c = '0'
        tb_a <= '1'; tb_b <= '0'; WAIT FOR 10 ns;  -- esperado: c = '0'
        tb_a <= '1'; tb_b <= '1'; WAIT FOR 10 ns;  -- esperado: c = '1'
        WAIT;  -- detiene la simulación
    END PROCESS;

END sim;
```

---

<p align="center">
  <sub>Guía de referencia VHDL · IEEE 1076 · Elaborada con fines académicos</sub>
</p>
