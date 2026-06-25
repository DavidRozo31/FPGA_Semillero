# Módulos Operadores Aritméticos — Proyecto Brazo 2R FPGA

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo)  
> **Plataforma:** DE10-Nano (Cyclone V) · Quartus 18.1 · 50 MHz  
> **Universidad Militar Nueva Granada**

Todos los operadores trabajan sobre la misma representación numérica:

```
Factor de escala = 2^13 = 8192
Valor real = entero_Q2.13 / 8192
Rango representable ≈ [-4.0, +4.0]
```

---

## Índice de módulos

| Módulo | Función | Latencia |
|--------|---------|----------|
| `fp_multiplier` | Multiplicación Q2.13 × Q2.13 | 1 ciclo |
| `fp_adder` | Suma / resta Q2.13 | 1 ciclo |
| `fp_divider` | División Q2.13 / Q2.13 | 15 ciclos |
| `cordic_sincos_16` | sin() y cos() en Q2.13 | N_ITER+1 ciclos |
| `cordic_atan2` | atan2(y,x) en Q2.13 | N_ITER+1 ciclos |

---

## 1 · `fp_multiplier` — Multiplicador

**Operación:** `p_out = a_in × b_in` (ambos en Q2.13, resultado en Q2.13)

### El problema del escalado

Multiplicar dos números Q2.13 produce un resultado Q4.26 de 32 bits:

```
a × b = [entero × 8192] × [entero × 8192] = producto × 8192²
```

Para volver a Q2.13 hay que desplazar 13 bits hacia la derecha, extrayendo los bits `[28:13]` del producto de 32 bits:

```
bits: 31 30 29 28 | 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 | 12 11 ... 0
      ↑ signo+int  ←————————— fracción Q2.13 (13 bits) ——————————→   descartados
      extraer prod_reg(28 downto 13)
```

### Señales y proceso

```
valid_i='1' → latcha a_in, b_in y calcula el producto en 1 ciclo
flag        → mantiene valid_o en alto indefinidamente una vez activado
```

> **Nota:** La señal `flag` hace que el resultado permanezca válido tras el primer `valid_i`. En un pipeline continuo esto es intencional; el módulo aguas abajo debe capturar `p_out` en el ciclo en que necesita el dato.

### Diagrama de tiempo

```
clk:      __|‾|_|‾|_|‾|_
valid_i:  ___|‾|__________
a_in:     ───[A]──────────
b_in:     ───[B]──────────
p_out:    ────────[A×B]───   (1 ciclo de latencia)
valid_o:  ________|‾‾‾‾‾‾
```

---

## 2 · `fp_adder` — Sumador / Restador

**Operación:** `p_out = a_in + b_in` (op=0) ó `p_out = a_in - b_in` (op=1)

### Por qué no hay problema de escalado

La suma de dos Q2.13 produce directamente otro Q2.13 (mismo factor de escala), por lo que **no se necesita reescalado**:

```
(a × 8192) + (b × 8192) = (a + b) × 8192   ✓ sigue siendo Q2.13
```

El único riesgo es overflow si `a + b > 3.999` (límite del rango Q2.13). En la cinemática del brazo 2R los operandos están acotados por la geometría, por lo que no ocurre.

### Puerto `op`

| `op` | Operación |
|------|-----------|
| `'0'` | suma: `a + b` |
| `'1'` | resta: `a − b` |

### Diagrama de tiempo

```
clk:      __|‾|_|‾|_|‾|_
valid_i:  ___|‾|__________
a_in:     ───[A]──────────
b_in:     ───[B]──────────
op:       ───[0/1]─────────
p_out:    ────────[A±B]───   (1 ciclo de latencia)
valid_o:  ________|‾‾‾‾‾‾
```

---

## 3 · `fp_divider` — Divisor iterativo

**Operación:** `quot_out = num_in / den_in` (ambos Q2.13, resultado Q2.13)

### Algoritmo: división binaria larga (non-restoring)

La FPGA no tiene divisor hardware. El módulo implementa el algoritmo clásico de **división larga binaria** bit a bit, que en `N_DIV = 13` iteraciones produce 13 bits de cociente — exactamente los 13 bits de fracción del formato Q2.13.

Cada iteración en `S_RUN`:

```
1. Desplaza el residuo 1 bit a la izquierda (rem << 1)
2. Si rem >= den:
     rem  = rem - den
     quot = (quot << 1) | 1      ← bit de cociente = 1
   Si no:
     quot = (quot << 1) | 0      ← bit de cociente = 0
```

Tras 13 iteraciones, `quot_s[12:0]` contiene el resultado en Q2.13.

### Manejo de signo

Antes de iterar, el módulo extrae los valores absolutos y registra el signo del resultado:

```vhdl
neg_s <= num_s(15) xor den_sig(15);   -- XOR de signos
```

Al finalizar, si `neg_s = '1'` se niega el resultado.

### FSM de control

```
         start='1'              cnt = N_DIV-1
S_IDLE ───────────► S_RUN ──────────────────► S_FINISH ──► S_IDLE
                     │                            │
                     │ 13 iteraciones             │ result = ±quot_s[12:0]
                     │ shift + compare            │ done_r = '1'
                     └────────────────────────────┘
```

### Diagrama de tiempo

```
clk:       __|‾|_|‾|_ ... _|‾|_|‾|_|‾|_
start:     ___|‾|_______________
           S_IDLE│S_RUN (13 ciclos)│S_FINISH
quot_out:  ──────────────────────────[N/D]──
done:      ─────────────────────────────|‾|─  (1 ciclo)
```

> **Latencia total: 15 ciclos** (1 S_IDLE + 13 S_RUN + 1 S_FINISH).

---

## 4 · `cordic_sincos_16` — Seno y Coseno

**Operación:** dado `angle_in` en Q2.13 (radianes), calcula `sin_out` y `cos_out` en Q2.13.

### Modo de operación: CORDIC rotación

A diferencia del módulo `atan2` (que usa modo *vectoring* para llevar `y→0`), este módulo usa modo **rotación**: parte de un vector conocido y lo rota hasta que el acumulador de ángulo `z→0`. El vector resultante tiene componentes `(cos θ, sin θ)`.

### Etapa 0: reducción de cuadrante

CORDIC en modo rotación converge solo para ángulos en `[-π/2, +π/2]`. Si el ángulo está fuera de ese rango, se aplica una reflexión:

| Condición | Ángulo usado | Corrección al final |
|-----------|-------------|---------------------|
| `θ ∈ [-π/2, +π/2]` | `θ` | ninguna |
| `θ > π/2` (Q2) | `π - θ` | negar `cos` |
| `θ < -π/2` (Q3) | `-π - θ` | negar `cos` y `sin` |

Las banderas de corrección `fcos_pipe` y `fsin_pipe` viajan por el pipeline junto al dato.

**Valor inicial del vector:**

```vhdl
x_pipe(0) <= shift_right(CORDIC_K, 1);   -- K/2
y_pipe(0) <= 0;
```

`CORDIC_K` es la inversa de la ganancia CORDIC (≈ 0.607 × 8192 = 4977). Se divide por 2 aquí y se multiplica por 2 a la salida (`shift_left(..., 1)`), lo que compensa exactamente la ganancia del algoritmo sin perder rango.

### Etapas 1..N_ITER: rotaciones

```
d = +1  si z(i-1) ≥ 0    (rotar en sentido positivo)
d = -1  si z(i-1) < 0    (rotar en sentido negativo)

x(i) = x(i-1) − d · y(i-1) · 2^-(i-1)
y(i) = y(i-1) + d · x(i-1) · 2^-(i-1)
z(i) = z(i-1) − d · atan(2^-(i-1))
```

Al converger, `z(N_ITER) ≈ 0` y el vector `(x, y)` apunta en la dirección del ángulo original.

### Salidas con corrección

```vhdl
cos_out <= -shift_left(x_pipe(N_ITER), 1)  when fcos_pipe(N_ITER)='1'
           else shift_left(x_pipe(N_ITER), 1);

sin_out <= -shift_left(y_pipe(N_ITER), 1)  when fsin_pipe(N_ITER)='1'
           else shift_left(y_pipe(N_ITER), 1);
```

El `shift_left(..., 1)` (×2) compensa la división por 2 hecha en la entrada.

> **Latencia: N_ITER + 1 ciclos** (tipicamente 13 ciclos con `N_ITER = 12`).

---

## 5 · `cordic_atan2` — Arco tangente

**Operación:** dado un vector `(x_in, y_in)` en Q2.13, calcula `angle = atan2(y, x)` en Q2.13.

### Modo de operación: CORDIC vectoring

El objetivo es rotar el vector `(x, y)` hasta que `y → 0`. Cuando eso ocurre, el acumulador `z` contiene el ángulo que había que rotar, que es justamente `atan2(y, x)`.

### Etapa 0: corrección de cuadrante

CORDIC vectoring converge solo cuando `x > 0` (semicírculo derecho). Para vectores con `x < 0` se refleja el vector 180° y se guarda la corrección:

| Condición | Corrección `quad_corr` |
|-----------|----------------------|
| `x ≥ 0` | `0` |
| `x < 0`, `y ≥ 0` | `+π` (+25736) |
| `x < 0`, `y < 0` | `−π` (−25736) |

Las entradas también se dividen por 2 (`shift_right(..., 1)`) para evitar overflow por la ganancia CORDIC.

### Iteraciones y salida

Las iteraciones siguen la misma fórmula que `cordic_sincos_16` pero con la dirección inversa (basada en el signo de `y` en lugar de `z`):

```
d = +1  si y(i) ≥ 0
d = -1  si y(i) < 0

x(i+1) = x(i) + d · y(i) · 2^-i
y(i+1) = y(i) − d · x(i) · 2^-i
z(i+1) = z(i) + d · atan(2^-i)
```

La salida final suma la corrección de cuadrante:

```vhdl
angle <= std_logic_vector(z_pipe(N_ITER) + quad_corr);
```

> **Latencia: N_ITER + 1 ciclos.**

---

## Comparativa de módulos

| Módulo | Operación | Latencia | Recursos principales |
|--------|-----------|----------|----------------------|
| `fp_adder` | A ± B | 1 ciclo | 1 sumador 16-bit |
| `fp_multiplier` | A × B | 1 ciclo | 1 multiplicador 16×16 (DSP block) |
| `fp_divider` | A / B | 15 ciclos | 1 sumador 32-bit + FSM |
| `cordic_sincos_16` | sin, cos | N_ITER+1 | N_ITER sumadores en pipeline |
| `cordic_atan2` | atan2(y,x) | N_ITER+1 | N_ITER sumadores en pipeline |

---

---

# Código fuente completo

## `fp_multiplier.vhd`

```vhdl
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fp_multiplier is
    Port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        valid_i : in  std_logic;
        a_in    : in  std_logic_vector(15 downto 0);
        b_in    : in  std_logic_vector(15 downto 0);
        p_out   : out std_logic_vector(15 downto 0);
        valid_o : out std_logic
    );
end fp_multiplier;

architecture rtl of fp_multiplier is
    signal flag      : std_logic := '0';
    signal prod_reg  : signed(31 downto 0) := (others => '0');
    signal valid_reg : std_logic := '0';
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                flag      <= '0';
                prod_reg  <= (others => '0');
                valid_reg <= '0';
            elsif valid_i = '1' or flag = '1' then
                flag      <= '1';
                prod_reg  <= signed(a_in) * signed(b_in);
                valid_reg <= '1';
            end if;
        end if;
    end process;

    p_out   <= std_logic_vector(prod_reg(28 downto 13));
    valid_o <= valid_reg;
end rtl;
```

---

## `fp_adder.vhd`

```vhdl
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fp_adder is
    Port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        op      : in  std_logic;
        valid_i : in  std_logic;
        a_in    : in  std_logic_vector(15 downto 0);
        b_in    : in  std_logic_vector(15 downto 0);
        p_out   : out std_logic_vector(15 downto 0);
        valid_o : out std_logic
    );
end fp_adder;

architecture rtl of fp_adder is
    signal flag       : std_logic := '0';
    signal result_reg : signed(15 downto 0) := (others => '0');
    signal valid_reg  : std_logic := '0';
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                flag       <= '0';
                result_reg <= (others => '0');
                valid_reg  <= '0';
            elsif valid_i = '1' or flag = '1' then
                flag      <= '1';
                valid_reg <= '1';
                if op = '1' then
                    result_reg <= signed(a_in) - signed(b_in);
                else
                    result_reg <= signed(a_in) + signed(b_in);
                end if;
            end if;
        end if;
    end process;

    p_out   <= std_logic_vector(result_reg);
    valid_o <= valid_reg;
end rtl;
```

---

## `fp_divider.vhd`

```vhdl
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fp_divider is
    Port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        num_in   : in  std_logic_vector(15 downto 0);
        den_in   : in  std_logic_vector(15 downto 0);
        quot_out : out std_logic_vector(15 downto 0);
        done     : out std_logic
    );
end fp_divider;

architecture rtl of fp_divider is
    constant N_DIV : integer := 13;
    type state_t is (S_IDLE, S_RUN, S_FINISH);
    signal state  : state_t := S_IDLE;
    signal rem_s  : unsigned(31 downto 0) := (others => '0');
    signal den_s  : unsigned(31 downto 0) := (others => '0');
    signal quot_s : unsigned(31 downto 0) := (others => '0');
    signal neg_s  : std_logic := '0';
    signal cnt    : integer range 0 to N_DIV := 0;
    signal result : signed(15 downto 0) := (others => '0');
    signal done_r : std_logic := '0';
    signal flag   : std_logic := '0';
    signal num_s  : signed(15 downto 0);
    signal den_sig: signed(15 downto 0);
begin
    num_s   <= signed(num_in);
    den_sig <= signed(den_in);

    process(clk)
        variable rem_v   : unsigned(31 downto 0);
        variable q16     : signed(15 downto 0);
        variable abs_num : unsigned(15 downto 0);
        variable abs_den : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state  <= S_IDLE;
                flag   <= '0';
                rem_s  <= (others => '0');
                den_s  <= (others => '0');
                quot_s <= (others => '0');
                neg_s  <= '0';
                cnt    <= 0;
                result <= (others => '0');
                done_r <= '0';
            else
                case state is
                    when S_IDLE =>
                        if start = '1' or flag = '1' then
                            flag  <= '1';
                            neg_s <= num_s(15) xor den_sig(15);

                            if num_s(15) = '1' then
                                abs_num := unsigned(-num_s);
                            else
                                abs_num := unsigned(num_s);
                            end if;

                            if den_sig(15) = '1' then
                                abs_den := unsigned(-den_sig);
                            else
                                abs_den := unsigned(den_sig);
                            end if;

                            rem_s  <= resize(abs_num, 32);
                            den_s  <= resize(abs_den, 32);
                            quot_s <= (others => '0');
                            cnt    <= 0;
                            state  <= S_RUN;
                        end if;

                    when S_RUN =>
                        rem_v := shift_left(rem_s, 1);
                        if rem_v >= den_s then
                            rem_s  <= rem_v - den_s;
                            quot_s <= shift_left(quot_s, 1) or to_unsigned(1, 32);
                        else
                            rem_s  <= rem_v;
                            quot_s <= shift_left(quot_s, 1);
                        end if;
                        if cnt = N_DIV - 1 then
                            state <= S_FINISH;
                        else
                            cnt <= cnt + 1;
                        end if;

                    when S_FINISH =>
                        q16 := signed(resize(quot_s(12 downto 0), 16));
                        if neg_s = '1' then
                            result <= -q16;
                        else
                            result <= q16;
                        end if;
                        done_r <= '1';
                        flag   <= '0';
                        state  <= S_IDLE;
                end case;
            end if;
        end if;
    end process;

    quot_out <= std_logic_vector(result);
    done     <= done_r;
end rtl;
```

---

## `cordic_sincos_16.vhd`

```vhdl
-- =============================================================
--  cordic_sincos_16.vhd
--  CORDIC modo rotacion - Sin y Cos en Q2.13 (16 bits)
--  Latencia: N_ITER + 1 = 13 ciclos
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;

entity cordic_sincos_16 is
    Port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        angle_in : in  signed(15 downto 0);
        sin_out  : out signed(15 downto 0);
        cos_out  : out signed(15 downto 0);
        done     : out std_logic
    );
end cordic_sincos_16;

architecture rtl of cordic_sincos_16 is

    type pipe_array is array (0 to N_ITER) of signed(15 downto 0);

    signal x_pipe     : pipe_array := (others => (others => '0'));
    signal y_pipe     : pipe_array := (others => (others => '0'));
    signal z_pipe     : pipe_array := (others => (others => '0'));
    signal valid_pipe : std_logic_vector(N_ITER downto 0) := (others => '0');
    signal fcos_pipe  : std_logic_vector(N_ITER downto 0) := (others => '0');
    signal fsin_pipe  : std_logic_vector(N_ITER downto 0) := (others => '0');

    constant SC_PI2_POS : signed(15 downto 0) := to_signed( 12868, 16);
    constant SC_PI2_NEG : signed(15 downto 0) := to_signed(-12868, 16);
    constant SC_PI_Q13  : signed(15 downto 0) := to_signed( 25736, 16);

begin

    process(clk, rst)
        variable x_sh : signed(15 downto 0);
        variable y_sh : signed(15 downto 0);
        variable ang  : signed(15 downto 0);
    begin
        if rst = '1' then
            x_pipe     <= (others => (others => '0'));
            y_pipe     <= (others => (others => '0'));
            z_pipe     <= (others => (others => '0'));
            valid_pipe <= (others => '0');
            fcos_pipe  <= (others => '0');
            fsin_pipe  <= (others => '0');

        elsif rising_edge(clk) then

            -- Etapa 0: carga y reduccion de cuadrante
            valid_pipe(0) <= start;
            fcos_pipe(0)  <= '0';
            fsin_pipe(0)  <= '0';

            if start = '1' then
                x_pipe(0) <= shift_right(CORDIC_K, 1);
                y_pipe(0) <= (others => '0');

                if angle_in > SC_PI2_POS then
                    ang          := SC_PI_Q13 - angle_in;
                    fcos_pipe(0) <= '1';
                    fsin_pipe(0) <= '0';
                elsif angle_in < SC_PI2_NEG then
                    ang          := -SC_PI_Q13 - angle_in;
                    fcos_pipe(0) <= '1';
                    fsin_pipe(0) <= '1';
                else
                    ang          := angle_in;
                    fcos_pipe(0) <= '0';
                    fsin_pipe(0) <= '0';
                end if;
                z_pipe(0) <= ang;
            end if;

            -- Etapas 1..N_ITER: rotaciones CORDIC
            for i in 1 to N_ITER loop
                valid_pipe(i) <= valid_pipe(i-1);
                fcos_pipe(i)  <= fcos_pipe(i-1);
                fsin_pipe(i)  <= fsin_pipe(i-1);

                x_sh := shift_right(x_pipe(i-1), i-1);
                y_sh := shift_right(y_pipe(i-1), i-1);

                if z_pipe(i-1) >= 0 then
                    x_pipe(i) <= x_pipe(i-1) - y_sh;
                    y_pipe(i) <= y_pipe(i-1) + x_sh;
                    z_pipe(i) <= z_pipe(i-1) - ATAN_TABLE(i-1);
                else
                    x_pipe(i) <= x_pipe(i-1) + y_sh;
                    y_pipe(i) <= y_pipe(i-1) - x_sh;
                    z_pipe(i) <= z_pipe(i-1) + ATAN_TABLE(i-1);
                end if;
            end loop;

        end if;
    end process;

    cos_out <= -shift_left(x_pipe(N_ITER), 1)
               when fcos_pipe(N_ITER) = '1'
               else shift_left(x_pipe(N_ITER), 1);

    sin_out <= -shift_left(y_pipe(N_ITER), 1)
               when fsin_pipe(N_ITER) = '1'
               else shift_left(y_pipe(N_ITER), 1);

    done <= valid_pipe(N_ITER);

end rtl;
```

---

## `cordic_atan2.vhd`

```vhdl
-- =============================================================
--  cordic_atan2.vhd  (Opcion A: puertos STD_LOGIC_VECTOR)
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;

entity cordic_atan2 is
    Port (
        clk   : in  std_logic;
        rst   : in  std_logic;
        start : in  std_logic;
        x_in  : in  std_logic_vector(15 downto 0);
        y_in  : in  std_logic_vector(15 downto 0);
        angle : out std_logic_vector(15 downto 0);
        done  : out std_logic
    );
end cordic_atan2;

architecture vectoring of cordic_atan2 is

    type data_array is array (0 to N_ITER) of signed(15 downto 0);
    signal x_pipe     : data_array;
    signal y_pipe     : data_array;
    signal z_pipe     : data_array;
    signal valid_pipe : std_logic_vector(N_ITER downto 0);
    signal quad_corr  : signed(15 downto 0);

    constant PI_POS : signed(15 downto 0) := to_signed( 25736, 16);
    constant PI_NEG : signed(15 downto 0) := to_signed(-25736, 16);

    signal x_in_s : signed(15 downto 0);
    signal y_in_s : signed(15 downto 0);

begin

    x_in_s <= signed(x_in);
    y_in_s <= signed(y_in);

    process(clk, rst)
        variable x_scaled : signed(15 downto 0);
        variable y_scaled : signed(15 downto 0);
    begin
        if rst = '1' then
            x_pipe(0)     <= (others => '0');
            y_pipe(0)     <= (others => '0');
            z_pipe(0)     <= (others => '0');
            valid_pipe(0) <= '0';
            quad_corr     <= (others => '0');
        elsif rising_edge(clk) then
            if start = '1' then
                valid_pipe(0) <= '1';
                z_pipe(0)     <= (others => '0');
                x_scaled := shift_right(x_in_s, 1);
                y_scaled := shift_right(y_in_s, 1);
                if x_in_s >= 0 then
                    x_pipe(0) <= x_scaled;
                    y_pipe(0) <= y_scaled;
                    quad_corr <= to_signed(0, 16);
                elsif y_in_s >= 0 then
                    x_pipe(0) <= -x_scaled;
                    y_pipe(0) <= -y_scaled;
                    quad_corr <= PI_POS;
                else
                    x_pipe(0) <= -x_scaled;
                    y_pipe(0) <= -y_scaled;
                    quad_corr <= PI_NEG;
                end if;
            else
                valid_pipe(0) <= '0';
            end if;
        end if;
    end process;

    GEN_VEC: for i in 0 to N_ITER-1 generate
        process(clk, rst)
            variable x_sh : signed(15 downto 0);
            variable y_sh : signed(15 downto 0);
        begin
            if rst = '1' then
                x_pipe(i+1)     <= (others => '0');
                y_pipe(i+1)     <= (others => '0');
                z_pipe(i+1)     <= (others => '0');
                valid_pipe(i+1) <= '0';
            elsif rising_edge(clk) then
                valid_pipe(i+1) <= valid_pipe(i);
                x_sh := shift_right(x_pipe(i), i);
                y_sh := shift_right(y_pipe(i), i);
                if y_pipe(i) >= 0 then
                    x_pipe(i+1) <= x_pipe(i) + y_sh;
                    y_pipe(i+1) <= y_pipe(i) - x_sh;
                    z_pipe(i+1) <= z_pipe(i) + ATAN_TABLE(i);
                else
                    x_pipe(i+1) <= x_pipe(i) - y_sh;
                    y_pipe(i+1) <= y_pipe(i) + x_sh;
                    z_pipe(i+1) <= z_pipe(i) - ATAN_TABLE(i);
                end if;
            end if;
        end process;
    end generate GEN_VEC;

    angle <= std_logic_vector(z_pipe(N_ITER) + quad_corr);
    done  <= valid_pipe(N_ITER);

end vectoring;
```
