# PWM_50Hz - Generador de Senal PWM en VHDL

## Descripcion General

`PWM_50Hz` es un modulo VHDL que genera una senal de modulacion por ancho de pulso (PWM) con
una frecuencia de salida de **50 Hz**, disenado para ser sintetizado en FPGAs con un reloj de
sistema de **50 MHz** (por ejemplo, la familia Cyclone de Intel/Altera).

El modulo recibe un ciclo de trabajo (`dutycycle`) como entrada digital y produce una senal
`pwm_out` cuyo pulso alto es proporcional a ese valor. Es util para controlar servomotores,
dimmers de LED, convertidores DC-DC, entre otros.

---

## Tabla de Contenidos

1. [Parametros genericos](#1-parametros-genericos)
2. [Puertos de entrada y salida](#2-puertos-de-entrada-y-salida)
3. [Constantes y senales internas](#3-constantes-y-senales-internas)
4. [Proceso secuencial: el contador](#4-proceso-secuencial-el-contador)
5. [Logica combinacional: comparador PWM](#5-logica-combinacional-comparador-pwm)
6. [Calculo de frecuencia y ciclo de trabajo](#6-calculo-de-frecuencia-y-ciclo-de-trabajo)
7. [Diagrama de temporizado](#7-diagrama-de-temporizado)
8. [Tabla de estados del contador](#8-tabla-de-estados-del-contador)
9. [Uso tipico: control de servomotor](#9-uso-tipico-control-de-servomotor)
10. [Consideraciones de sintesis](#10-consideraciones-de-sintesis)
11. [Resumen visual de la arquitectura](#11-resumen-visual-de-la-arquitectura)

---

## 1. Parametros Genericos

```vhdl
Generic(
    N : integer := 20
);
```

| Parametro | Valor por defecto | Descripcion |
|-----------|-------------------|-------------|
| `N`       | `20`              | Numero de bits del contador y del bus `dutycycle` |

`N` define el ancho de palabra de todas las estructuras internas del modulo.
Con `N = 20` bits se pueden representar valores sin signo entre **0** y **1,048,575** (2^20 - 1).

El periodo del PWM es de 1,000,000 ciclos de reloj (ver seccion 3), lo cual cabe
perfectamente en 20 bits ya que 2^20 = 1,048,576 > 1,000,000. Si se usara un reloj mas
rapido o un periodo mas largo que requiriera mas de 2^20 ciclos, habria que aumentar `N`.

---

## 2. Puertos de Entrada y Salida

```vhdl
Port (
    clk       : in  std_logic;
    start     : in  std_logic;
    dutycycle : in  std_logic_vector(N-1 downto 0);
    pwm_out   : out std_logic
);
```

### `clk` - Reloj del sistema

- Tipo: `std_logic`
- Direccion: entrada
- Funcion: reloj principal del modulo. Todo el comportamiento secuencial ocurre en el
  flanco de subida de esta senal. Se asume un reloj de **50 MHz** (periodo de 20 ns).

### `start` - Habilitacion del modulo

- Tipo: `std_logic`
- Direccion: entrada
- Funcion: control de encendido/apagado del generador PWM.
  - `start = '0'`: el contador se reinicia a cero y la salida se fuerza a `'0'`.
  - `start = '1'`: el modulo opera normalmente, generando la senal PWM.

> Nota: `start` NO es un reset asincronico. El reinicio del contador ocurre en el
> proximo flanco de subida del reloj (reset sincrono controlado).

### `dutycycle` - Ciclo de trabajo

- Tipo: `std_logic_vector(N-1 downto 0)` (equivalente a 20 bits sin signo)
- Direccion: entrada
- Funcion: determina cuantos ciclos de reloj dentro del periodo total la salida estara en `'1'`.
- Rango util: de `0` a `PERIOD` (1,000,000). Valores mayores a `PERIOD` saturan la
  salida en `'1'` permanentemente.

Se recibe como `std_logic_vector` y se convierte a `unsigned` internamente al momento
de la comparacion (ver seccion 5).

### `pwm_out` - Salida PWM

- Tipo: `std_logic`
- Direccion: salida
- Funcion: senal PWM resultante. Alterna entre `'0'` y `'1'` con la frecuencia y
  ciclo de trabajo configurados.

---

## 3. Constantes y Senales Internas

```vhdl
constant PERIOD : unsigned(N-1 downto 0) := to_unsigned(1000000, N);
signal counter  : unsigned(N-1 downto 0) := (others => '0');
```

### `PERIOD` - Periodo del PWM en ciclos de reloj

- Tipo: `unsigned(19 downto 0)`
- Valor: `1,000,000`

Con un reloj de 50 MHz, cada ciclo dura 20 ns. El periodo total del PWM es:

```
T_pwm = PERIOD x T_clk = 1,000,000 x 20 ns = 20,000,000 ns = 20 ms
```

Lo que equivale a una frecuencia de salida de:

```
f_pwm = 1 / 20 ms = 50 Hz
```

La funcion `to_unsigned(1000000, N)` convierte el literal entero `1000000` al tipo
`unsigned` de `N = 20` bits, que es el tipo usado en el contador.

### `counter` - Contador libre

- Tipo: `unsigned(19 downto 0)`
- Valor inicial: `(others => '0')` -> todos los bits en `'0'`, es decir, valor 0
- Funcion: lleva la cuenta de ciclos de reloj dentro del periodo actual. Se incrementa
  cada ciclo mientras `start = '1'` y se reinicia al llegar a `PERIOD - 1`.

---

## 4. Proceso Secuencial: El Contador

```vhdl
process(clk)
begin
    if rising_edge(clk) then
        if start = '0' then
            counter <= (others => '0');
        elsif counter = PERIOD - 1 then
            counter <= (others => '0');
        else
            counter <= counter + 1;
        end if;
    end if;
end process;
```

Este proceso es puramente secuencial: solo reacciona al **flanco de subida del reloj**.
Su lista de sensibilidad contiene unicamente `clk`, lo cual es correcto para logica
registrada sincrona.

### Flujo de decision en cada flanco de subida:

```
rising_edge(clk)?
    SI ->
        start = '0'?
            SI  -> counter = 0       (reset sincrono)
            NO  ->
                counter = PERIOD - 1?
                    SI  -> counter = 0   (reinicio al completar el periodo)
                    NO  -> counter++     (incremento normal)
```

### Por que `PERIOD - 1` y no `PERIOD`?

El contador comienza en **0**, no en **1**. Entonces para contar exactamente 1,000,000
ciclos, los valores van de 0 a 999,999, que son exactamente 1,000,000 estados distintos.
Si se comparara con `PERIOD` (1,000,000), el contador alcanzaria el valor 1,000,000 en
un ciclo adicional antes de reiniciarse, generando un periodo de 1,000,001 ciclos.

```
Valores: 0, 1, 2, ..., 999998, 999999, 0, 1, 2, ...
                                  ^
                           Aqui se reinicia (= PERIOD - 1)
```

### Prioridades de la logica interna:

El `if/elsif` establece una jerarquia clara:
1. `start = '0'` tiene mayor prioridad: si el modulo esta deshabilitado, el contador
   siempre es cero sin importar ningun otro estado.
2. `counter = PERIOD - 1` es la condicion de desborde/wrap-around.
3. El `else` es el caso normal de incremento.

---

## 5. Logica Combinacional: Comparador PWM

```vhdl
pwm_out <= '1' when (counter < unsigned(dutycycle) and start = '1') else '0';
```

Esta es una asignacion concurrente (fuera de cualquier proceso), lo que significa que
se evalua continuamente cada vez que `counter`, `dutycycle` o `start` cambian.

### Condiciones para `pwm_out = '1'`:

Ambas condiciones deben cumplirse simultaneamente:

| Condicion | Descripcion |
|-----------|-------------|
| `counter < unsigned(dutycycle)` | El contador aun no ha alcanzado el umbral del ciclo de trabajo |
| `start = '1'` | El modulo esta habilitado |

### Conversion de tipo: `unsigned(dutycycle)`

`dutycycle` se declara como `std_logic_vector`, que en VHDL es un tipo sin
interpretacion numerica. Para compararlo con `counter` (tipo `unsigned`) se necesita
una conversion explicita usando la funcion `unsigned()` de la libreria `IEEE.NUMERIC_STD`.

Esta conversion es **puramente en tiempo de compilacion/sintesis**: no genera hardware
adicional. Los bits son identicos; solo cambia como el sintetizador los interpreta
(como numero sin signo).

### Comportamiento segun el valor de `dutycycle`:

| Valor de dutycycle | Comportamiento |
|--------------------|----------------|
| `0` | `pwm_out` siempre `'0'` (0% duty cycle) |
| `500000` | `pwm_out = '1'` durante la primera mitad del periodo (50%) |
| `1000000` | `pwm_out = '1'` todo el tiempo (100%) |
| `> 1000000` | `pwm_out = '1'` todo el tiempo (saturacion) |

---

## 6. Calculo de Frecuencia y Ciclo de Trabajo

### Frecuencia de salida

Fijada por la constante `PERIOD` y la frecuencia del reloj:

```
f_pwm = f_clk / PERIOD = 50,000,000 Hz / 1,000,000 = 50 Hz
```

### Ciclo de trabajo (Duty Cycle)

El porcentaje de tiempo que `pwm_out` permanece en `'1'` dentro de un periodo:

```
Duty Cycle (%) = (dutycycle / PERIOD) x 100
```

### Ejemplos para control de servomotor (50 Hz, 20 ms de periodo):

Los servomotores estandar operan con pulsos entre 1 ms (0 grados) y 2 ms (180 grados).

| Angulo deseado | Tiempo en alto | dutycycle (ciclos) | dutycycle (hex) |
|----------------|---------------|-------------------|-----------------|
| 0 grados       | 1.0 ms        | 50,000            | 0xC350          |
| 45 grados      | 1.25 ms       | 62,500            | 0xF424          |
| 90 grados      | 1.5 ms        | 75,000            | 0x124F8         |
| 135 grados     | 1.75 ms       | 87,500            | 0x155CC         |
| 180 grados     | 2.0 ms        | 100,000           | 0x186A0         |

Calculo para 1.5 ms (90 grados):
```
dutycycle = t_on / T_clk = 1.5 ms / 20 ns = 75,000 ciclos
```

---

## 7. Diagrama de Temporizado

```
clk:       _|^|_|^|_|^|_|^|_|^|_|^|_|^|_|^|_|^|_|^|_
start:     _____|^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
counter:   0    0  1  2  3  ...  D-1  D  ...  999999  0
                                  ^
                              dutycycle
pwm_out:   _______|^^^^^^^^^^^^^^^^^^^|_________________|
                  <--- D ciclos --->  <- 1M-D ciclos ->
           <------------- 1,000,000 ciclos = 20 ms -------->
```

Donde `D` es el valor numerico de `dutycycle`.

Nota: hay un ciclo de latencia al activar `start` porque el contador es sincrono.
La salida `pwm_out` es combinacional respecto al contador, asi que reacciona
inmediatamente al cambio del contador en cada flanco.

---

## 8. Tabla de Estados del Contador

| Condicion evaluada (en orden) | Accion | Prioridad |
|-------------------------------|--------|-----------|
| `start = '0'` | `counter <= 0` | Alta (1) |
| `counter = 999999` | `counter <= 0` | Media (2) |
| Ninguna de las anteriores | `counter <= counter + 1` | Baja (3) |

---

## 9. Uso Tipico: Control de Servomotor

Ejemplo de instanciacion en un diseno de nivel superior:

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY Top_Servo IS
    Port (
        clk     : in  std_logic;  -- 50 MHz
        enable  : in  std_logic;
        angle   : in  std_logic_vector(7 downto 0);  -- 0 a 180
        servo   : out std_logic
    );
END Top_Servo;

ARCHITECTURE rtl OF Top_Servo IS

    COMPONENT PWM_50Hz
        Generic( N : integer := 20 );
        Port (
            clk       : in  std_logic;
            start     : in  std_logic;
            dutycycle : in  std_logic_vector(19 downto 0);
            pwm_out   : out std_logic
        );
    END COMPONENT;

    signal duty : std_logic_vector(19 downto 0);
    signal duty_val : unsigned(19 downto 0);

BEGIN
    -- Convertir angulo (0-180) a ciclos (50000 a 100000)
    -- Formula: duty = 50000 + (angle * 50000 / 180)
    duty_val <= to_unsigned(50000, 20) +
                resize(unsigned(angle) * to_unsigned(278, 9), 20);
    duty <= std_logic_vector(duty_val);

    PWM_INST : PWM_50Hz
        Generic map( N => 20 )
        Port map (
            clk       => clk,
            start     => enable,
            dutycycle => duty,
            pwm_out   => servo
        );

END rtl;
```

---

## 10. Consideraciones de Sintesis

### Recursos de hardware generados

| Recurso | Cantidad aproximada |
|---------|---------------------|
| Flip-flops (registro del contador) | 20 |
| Sumador de 20 bits | 1 |
| Comparador de 20 bits (= PERIOD-1) | 1 |
| Comparador de 20 bits (< dutycycle) | 1 |
| Multiplexor 20 bits (seleccion de next_counter) | 1 |

### Frecuencia maxima de operacion

El camino critico es la cadena: `counter -> comparador -> mux -> D del FF`.
En Cyclone V, esto tipicamente sintetiza a frecuencias superiores a 100 MHz, por lo
que no hay problema operando a 50 MHz.

### Cambio de `dutycycle` en tiempo de ejecucion

El puerto `dutycycle` es una entrada combinacional al comparador. Un cambio en
`dutycycle` se refleja en `pwm_out` de forma asincronico respecto al contador.
Si se cambia `dutycycle` en mitad de un periodo, el pulso actual puede quedar truncado
o extendido. Para evitar glitches en aplicaciones criticas, se recomienda usar un
registro intermedio que actualice `dutycycle` solo al inicio de cada periodo (cuando
`counter = 0`).

### Uso con relojes distintos a 50 MHz

Si el reloj del sistema es diferente, ajustar `PERIOD` en consecuencia:

```
PERIOD = f_clk / f_pwm_deseada
```

| Reloj del sistema | PERIOD para 50 Hz | N minimo requerido |
|-------------------|--------------------|---------------------|
| 25 MHz            | 500,000            | 19 bits             |
| 50 MHz            | 1,000,000          | 20 bits             |
| 100 MHz           | 2,000,000          | 21 bits             |
| 200 MHz           | 4,000,000          | 22 bits             |

---

## 11. Resumen Visual de la Arquitectura

```
         +------------------------------------------------------+
         |                    PWM_50Hz                          |
         |                                                       |
  clk -->|--+-------> [Proceso sincrono]                        |
         |  |          rising_edge(clk)                         |
start -->|--+--+        start=0 --> counter=0                   |--> pwm_out
         |     |        counter=PERIOD-1 --> counter=0          |
         |     |        else --> counter+1                       |
         |     |              |                                  |
         |     |         [counter: 20 bits unsigned]            |
         |     |              |                                  |
         |     +-----> [Comparador combinacional]               |
         |              counter < unsigned(dutycycle)           |
dutycycle|----------->  AND start='1'                           |
(slv 20b)|              |                                       |
         |              v                                       |
         |           pwm_out = '1' / '0'                       |
         +------------------------------------------------------+
```

### Librerias utilizadas

| Libreria | Paquete | Uso |
|----------|---------|-----|
| `IEEE` | `STD_LOGIC_1164` | Tipos `std_logic` y `std_logic_vector` |
| `IEEE` | `NUMERIC_STD` | Tipo `unsigned`, funcion `to_unsigned()`, operaciones aritmeticas |

---

*Documentacion generada para Quartus 18.1 - Familia Cyclone V - Reloj de sistema 50 MHz*
