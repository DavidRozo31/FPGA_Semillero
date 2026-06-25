# CORDIC `atan2` en VHDL — Cálculo de ángulo por hardware

> **Módulo:** `cordic_atan2.vhd`  
> **Algoritmo:** CORDIC modo vectoring (rotación iterativa)  
> **Aritmética:** Punto fijo Q2.13 (16 bits con signo)  
> **Aplicación:** Cinemática inversa del brazo 2R en FPGA (Cyclone IV)

---

## ¿Por qué CORDIC?

Una FPGA **no tiene unidad de punto flotante** ni instrucción `atan2`. Calcular un arco tangente requeriría división, raíces cuadradas y series de Taylor — operaciones costosas en hardware.

**CORDIC** (COordinate Rotation DIgital Computer) resuelve esto usando únicamente:
- Sumas y restas
- Desplazamientos de bits (`shift_right`) → equivalentes a divisiones por potencias de 2

El resultado es un módulo completamente sintetizable, sin multiplicadores, con latencia fija de **`N_ITER` ciclos de reloj**.

---

## Representación numérica: Punto fijo Q2.13

Todos los valores (ángulos, coordenadas) se representan en **Q2.13**:

```
bit 15  bit 14  bit 13 | bit 12 ... bit 0
 signo   entero  entero |  fracción (13 bits)
```

| Valor real | Representación Q2.13 |
|------------|----------------------|
| π  ≈ 3.14159 | 25736 (≈ 3.1416 × 8192) |
| −π | −25736 |
| 1.0 | 8192 |
| 0.5 | 4096 |

El factor de escala es **2¹³ = 8192**. Para convertir: `valor_real = entero / 8192`.

---

## Puertos del módulo

| Puerto  | Dirección | Bits | Descripción                                    |
|---------|-----------|------|------------------------------------------------|
| `clk`   | `in`      | 1    | Reloj del sistema (50 MHz)                     |
| `rst`   | `in`      | 1    | Reset síncrono activo en alto                  |
| `start` | `in`      | 1    | Pulso: lanza el cálculo con `x_in`, `y_in`     |
| `x_in`  | `in`      | 16   | Componente X del vector (Q2.13, STD_LOGIC_VECTOR) |
| `y_in`  | `in`      | 16   | Componente Y del vector (Q2.13, STD_LOGIC_VECTOR) |
| `angle` | `out`     | 16   | Ángulo resultante en radianes (Q2.13)           |
| `done`  | `out`     | 1    | Pulso: resultado válido en `angle`              |

**Resultado:** `angle` ≈ `atan2(y_in, x_in)` en el rango `[−π, +π]`.

---

## Estructura interna: el pipeline

El módulo implementa un **pipeline de `N_ITER` etapas**, donde cada etapa es un registro:

```
        Etapa 0           Etapa 1           Etapa 2      ...   Etapa N_ITER
start ──► [x,y,z]_0 ──► [x,y,z]_1 ──► [x,y,z]_2 ──► ... ──► [x,y,z]_N
           valid_0         valid_1         valid_2                valid_N
                                                                    │
                                                             angle = z + quad_corr
                                                             done  = valid_N
```

Las señales internas son arrays de señales:

```vhdl
type data_array is array (0 to N_ITER) of signed(15 downto 0);
signal x_pipe     : data_array;   -- Componente X en cada etapa
signal y_pipe     : data_array;   -- Componente Y en cada etapa
signal z_pipe     : data_array;   -- Acumulador de ángulo en cada etapa
signal valid_pipe : std_logic_vector(N_ITER downto 0);  -- Token de validez
```

`valid_pipe` es un **tren de bits de validez** que "viaja" junto al dato a través del pipeline. Cuando llega al final (`valid_pipe(N_ITER) = '1'`), el resultado en `z_pipe(N_ITER)` es válido.

---

## Paso 1 — Pre-procesamiento (Etapa 0): corrección de cuadrante

```vhdl
x_scaled := shift_right(x_in_s, 1);   -- x_in / 2
y_scaled := shift_right(y_in_s, 1);   -- y_in / 2
```

### ¿Por qué dividir por 2?

El algoritmo CORDIC introduce una **ganancia intrínseca** de `K ≈ 1.647` por las rotaciones sucesivas. Dividir la entrada por 2 previene desbordamiento (`overflow`) en los registros de 16 bits a lo largo del pipeline.

### Corrección de cuadrante

CORDIC en modo vectoring sólo converge correctamente cuando el vector inicial está en el **semicírculo derecho** (cuadrante I o IV, es decir, `x ≥ 0`). Si `x < 0`, hay que rotar el vector 180° antes de iterar y corregir el ángulo final sumando o restando π.

```vhdl
if x_in_s >= 0 then
    -- Cuadrantes I y IV: CORDIC converge directamente
    x_pipe(0) <= x_scaled;
    y_pipe(0) <= y_scaled;
    quad_corr <= 0;

elsif y_in_s >= 0 then
    -- Cuadrante II: reflejar vector, luego sumar π al resultado
    x_pipe(0) <= -x_scaled;
    y_pipe(0) <= -y_scaled;
    quad_corr <= PI_POS;   -- +π = +25736 en Q2.13

else
    -- Cuadrante III: reflejar vector, luego restar π al resultado
    x_pipe(0) <= -x_scaled;
    y_pipe(0) <= -y_scaled;
    quad_corr <= PI_NEG;   -- −π = −25736 en Q2.13
end if;
```

Visualización en el plano:

```
         y
         │   Q II  │  Q I
         │         │
─────────┼─────────┼──── x
         │         │
         │  Q III  │  Q IV
         │         │

Q I  y Q IV → x ≥ 0 → sin corrección
Q II         → x < 0, y ≥ 0 → rotar 180° → sumar +π
Q III        → x < 0, y < 0 → rotar 180° → restar −π
```

`z_pipe(0)` se inicializa en **0** porque el acumulador de ángulo parte de cero.

---

## Paso 2 — Iteraciones CORDIC (Etapas 1 a N_ITER)

Este es el corazón del algoritmo. Se genera con un `for generate`:

```vhdl
GEN_VEC: for i in 0 to N_ITER-1 generate
    process(clk, rst)
        variable x_sh : signed(15 downto 0);
        variable y_sh : signed(15 downto 0);
    begin
        ...
        x_sh := shift_right(x_pipe(i), i);   -- x_pipe(i) / 2^i
        y_sh := shift_right(y_pipe(i), i);   -- y_pipe(i) / 2^i

        if y_pipe(i) >= 0 then
            x_pipe(i+1) <= x_pipe(i) + y_sh;
            y_pipe(i+1) <= y_pipe(i) - x_sh;
            z_pipe(i+1) <= z_pipe(i) + ATAN_TABLE(i);
        else
            x_pipe(i+1) <= x_pipe(i) - y_sh;
            y_pipe(i+1) <= y_pipe(i) + x_sh;
            z_pipe(i+1) <= z_pipe(i) - ATAN_TABLE(i);
        end if;
    end process;
end generate;
```

### ¿Qué hace cada iteración?

Cada etapa `i` aplica una **micro-rotación** al vector `(x, y)`:

```
Dirección d_i:  +1  si y_pipe(i) ≥ 0  (rotar en sentido horario)
                −1  si y_pipe(i) < 0  (rotar en sentido antihorario)

x_new = x + d_i · (y / 2^i)
y_new = y − d_i · (x / 2^i)
z_new = z + d_i · atan(1/2^i)          ← ángulo acumulado
```

El objetivo en **modo vectoring** es llevar `y` hacia cero. Cada rotación reduce la magnitud de `y` aproximándola a 0, y acumula en `z` el ángulo correspondiente.

### La tabla `ATAN_TABLE`

Definida en `cordic_pkg`, contiene los ángulos precomputados en Q2.13:

```
ATAN_TABLE(0) = atan(1/1)   = 45.000° = π/4   → 6434
ATAN_TABLE(1) = atan(1/2)   = 26.565°          → 3798
ATAN_TABLE(2) = atan(1/4)   = 14.036°          → 2009
ATAN_TABLE(3) = atan(1/8)   =  7.125°          → 1017
...
ATAN_TABLE(i) = atan(2^-i)  en Q2.13
```

### Trazado de una iteración completa (ejemplo)

Supón que queremos calcular `atan2(y=1.0, x=1.0)` → resultado esperado: `π/4 = 45°`.

| Etapa | x (real) | y (real) | z (real) | Dirección |
|-------|----------|----------|----------|-----------|
| 0     | 0.500    | 0.500    | 0.000    | —         |
| 1     | 0.750    | 0.250    | 0.785    | +1 (y≥0)  |
| 2     | 0.875    | −0.125   | 1.249    | +1 (y≥0)  |
| 3     | 0.844    | 0.094    | 1.015    | −1 (y<0)  |
| ...   | ...      | ...      | →0       | ...       |
| N     | ≈K       | ≈0       | ≈π/4     | —         |

Al final: `y ≈ 0`, `z ≈ atan2(y_original, x_original)`.

---

## Paso 3 — Salida: corrección de cuadrante y `done`

```vhdl
angle <= std_logic_vector(z_pipe(N_ITER) + quad_corr);
done  <= valid_pipe(N_ITER);
```

- `z_pipe(N_ITER)` contiene el ángulo calculado por CORDIC para el vector rotado.
- Se le suma `quad_corr` (+π, −π, ó 0) para compensar la reflexión de cuadrante hecha en la etapa 0.
- El resultado se convierte de `signed` a `std_logic_vector` para la salida.
- `done` se activa exactamente cuando el token de validez recorre todos los registros del pipeline.

---

## Diagrama de tiempo (pipeline)

```
clk:    __|‾|_|‾|_|‾|_|‾|_|‾|_|‾| ...

start:  ___|‾|_________________________

        ← etapa 0 →← 1 →← 2 → ... ←N→
valid:  ______|‾‾‾‾‾|‾‾‾|‾‾‾|...|‾‾‾|
                                      │
done:   _____________________________|‾|___

angle:  ──────────────────────────[ válido ]──
```

El módulo tiene una **latencia fija de `N_ITER + 1` ciclos** desde `start='1'` hasta `done='1'`. Una vez lanzado, no se puede cancelar a mitad del pipeline (cada etapa trabaja de forma independiente).

---

## Resumen del paquete `cordic_pkg`

El módulo depende de definiciones externas en `cordic_pkg`:

| Constante/Tipo   | Descripción                                           |
|------------------|-------------------------------------------------------|
| `N_ITER`         | Número de iteraciones CORDIC (típicamente 12–15)      |
| `ATAN_TABLE`     | Array con `atan(2^-i)` en Q2.13 para i=0..N_ITER-1   |

---

## Resumen del flujo completo

```
                      ┌─────────────────────────────────────────────────────┐
                      │                 cordic_atan2                        │
                      │                                                      │
  x_in (Q2.13) ──────►│ Pre-proc   ┌──────────────────────┐                │
                      │ cuadrante  │ Pipeline de N_ITER    │ + quad_corr    │
  y_in (Q2.13) ──────►│ + escala   │ etapas CORDIC         ├────────────────►  angle
                      │ → etapa 0  │ (solo sumas y shifts) │                │
  start ──────────────►│            └──────────────────────┘                │
                      │ valid_pipe viaja por N_ITER registros               ►  done
                      └─────────────────────────────────────────────────────┘

  Latencia: N_ITER + 1 ciclos de reloj
  Precisión: ~0.01° para N_ITER = 13
  Recursos:  Solo sumadores + registros (sin multiplicadores)
```

---



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

    -- Senales internas signed para operar
    signal x_in_s : signed(15 downto 0);
    signal y_in_s : signed(15 downto 0);
begin

    -- Conversion de entradas SLV -> signed
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

    -- Conversion signed -> SLV en la salida
    angle <= std_logic_vector(z_pipe(N_ITER) + quad_corr);
    done  <= valid_pipe(N_ITER);

end vectoring;
```

## Notas de implementación

- **`shift_right` sobre `signed`** realiza desplazamiento aritmético (preserva el signo), lo cual es correcto para punto fijo con signo.
- **Desbordamiento:** La división por 2 en la entrada garantiza que los valores intermedios no excedan el rango Q2.13 durante las iteraciones.
- **Throughput:** Al ser un pipeline, puede aceptar una nueva operación **cada ciclo de reloj** (aunque cada resultado individual tarda `N_ITER+1` ciclos).
- **`quad_corr` es un registro:** Se captura en el mismo ciclo que `start` y permanece fijo durante todo el pipeline, por lo que siempre está sincronizado con el resultado que emerge por `z_pipe(N_ITER)`.
