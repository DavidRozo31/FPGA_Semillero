# Comunicación UART en VHDL — Transmisor y Receptor

> **Proyecto:** Módulos UART para FPGA (Cyclone IV)  
> **Frecuencia de reloj:** 50 MHz | **Baudrate:** 9600 bps  
> **Arquitectura:** FSM (Máquina de Estados Finita) + Registro de desplazamiento

---

## ¿Qué es UART?

**UART** (Universal Asynchronous Receiver-Transmitter) es un protocolo de comunicación serial **asíncrono**: no existe una línea de reloj compartida entre emisor y receptor. En su lugar, ambos lados acuerdan de antemano una velocidad de transmisión llamada **baudrate** (bits por segundo).

Cada "trama" UART transmite **1 byte (8 bits)** con la siguiente estructura:

```
IDLE  START  D0  D1  D2  D3  D4  D5  D6  D7  STOP
 1      0     ?   ?   ?   ?   ?   ?   ?   ?    1
```

| Bit      | Valor | Propósito                                      |
|----------|-------|------------------------------------------------|
| `IDLE`   | `1`   | Línea en reposo (sin transmisión)              |
| `START`  | `0`   | Anuncia el inicio de una trama                 |
| `D0–D7`  | dato  | 8 bits de datos, **LSB primero**               |
| `STOP`   | `1`   | Indica el fin de la trama                      |

---

## Parámetro clave: `BAUDS`

```vhdl
constant BAUDS : integer := 5208;
```

Este valor determina **cuántos ciclos de reloj dura cada bit**.

```
BAUDS = F_clk / Baudrate = 50,000,000 / 9600 ≈ 5208 ciclos
```

Un bit a 9600 bps dura `~104 µs`. A 50 MHz, eso equivale a 5208 pulsos de reloj.

---

## Módulo 1 — `UART_Transmitter`

### Puertos

| Puerto    | Dirección | Descripción                              |
|-----------|-----------|------------------------------------------|
| `clk`     | `in`      | Reloj del sistema (50 MHz)               |
| `rst`     | `in`      | Reset síncrono activo en alto            |
| `strt`    | `in`      | Pulso para iniciar transmisión           |
| `tx_data` | `in`      | Byte a transmitir (8 bits)              |
| `tx`      | `out`     | Línea serial de salida                   |
| `tx_flag` | `out`     | Indica que hay transmisión en curso      |

### Señales internas

```vhdl
constant bauds    : integer := 5208;
signal counter    : integer range 0 to bauds-1;   -- Contador de ciclos de reloj
signal bits       : integer range 0 to 9;         -- Índice del bit actual (0..9)
signal shift_reg  : std_logic_vector(9 downto 0); -- Trama completa: STOP+DATA+START
```

### Armado del registro de desplazamiento

Cuando llega `strt = '1'`:

```vhdl
shift_reg <= '1' & tx_data & '0';
--           STOP   8 bits   START
--            [9]   [8..1]    [0]
```

La trama completa queda empaquetada en `shift_reg`. Los bits se envían **de menor a mayor índice** usando el puntero `bits`.

```
shift_reg: [ '1' | D7 D6 D5 D4 D3 D2 D1 D0 | '0' ]
índice:       9    8  7  6  5  4  3  2  1    0
```

### Diagrama de estados (FSM)

```
         strt='1'              counter=BAUDS-1         bits=8          counter=BAUDS-1
  IDLE ──────────► START ──────────────────► DATA ──────────────► STOP ────────────────► IDLE
   │                                          │                                              │
   │ tx='1'                                   │ tx=shift_reg(bits)                          │ tx_flag='0'
   │ tx_flag='0'                              │ bits++                                      │
   │                                          │ (si bits<8 → DATA)                          │
   └─────────────────────────────────────────────────────────────────────────────────────────┘
```

### Paso a paso por estado

#### IDLE — Espera

```vhdl
when IDLE =>
    tx      <= '1';       -- Línea en reposo (alto)
    tx_flag <= '0';
    if strt = '1' then
        tx_flag   <= '1';
        shift_reg <= '1' & tx_data & '0';  -- Arma la trama
        counter   <= 0;
        bits      <= 0;
        state     <= START;
    end if;
```

> La línea `tx` permanece en `'1'` (IDLE). Al detectar `strt`, se arma `shift_reg` y se pasa a `START`.

---

#### START — Envía el bit de inicio (`shift_reg(0) = '0'`)

```vhdl
when START =>
    tx <= shift_reg(bits);    -- bits=0, envía '0' (start bit)
    if counter = bauds-1 then
        counter <= 0;
        bits    <= bits + 1;  -- Avanza al bit D0
        state   <= DATA;
    else
        counter <= counter + 1;
    end if;
```

> El start bit `'0'` se mantiene durante exactamente **5208 ciclos** antes de pasar a DATA.

---

#### DATA — Envía 8 bits de datos (D0 a D7)

```vhdl
when DATA =>
    tx <= shift_reg(bits);    -- bits va de 1 a 8
    if counter = bauds-1 then
        counter <= 0;
        if bits = 8 then
            bits  <= bits + 1;
            state <= STOP;    -- Todos los datos enviados
        else
            bits  <= bits + 1;
        end if;
    else
        counter <= counter + 1;
    end if;
```

> Cada bit dura 5208 ciclos. Se envían D0 (`shift_reg(1)`) hasta D7 (`shift_reg(8)`), es decir, **LSB primero**.

---

#### STOP — Envía el bit de parada (`shift_reg(9) = '1'`)

```vhdl
when STOP =>
    tx <= shift_reg(bits);    -- bits=9, envía '1' (stop bit)
    if counter = bauds-1 then
        tx_flag <= '0';
        counter <= 0;
        state   <= IDLE;
    else
        counter <= counter + 1;
    end if;
```

> El stop bit `'1'` cierra la trama. La FSM regresa a IDLE lista para el siguiente byte.

### Cronograma de transmisión

```
clk:  __|‾|_|‾|_|‾| ... (50 MHz)

tx:   ‾‾‾‾|_____|‾‾‾‾‾|_____|‾‾‾‾‾| ... |‾‾‾‾‾|‾‾‾‾‾‾‾‾‾‾
           START   D0     D1           D7    STOP
      IDLE         ←——————  8 bits ——————→
      
      ←5208 ciclos→←5208 ciclos→ ... ←5208 ciclos→
```

---

## Módulo 2 — `UART_Receiver`

### Puertos

| Puerto    | Dirección | Descripción                              |
|-----------|-----------|------------------------------------------|
| `clk`     | `in`      | Reloj del sistema (50 MHz)               |
| `rst`     | `in`      | Reset síncrono activo en alto            |
| `rx`      | `in`      | Línea serial de entrada                  |
| `rx_data` | `out`     | Byte recibido (8 bits)                   |
| `rx_flag` | `out`     | Pulso de 1 ciclo: dato listo             |

### Señales internas

```vhdl
constant BAUDS     : integer := 5208;
constant HALF_BAUD : integer := BAUDS / 2;  -- 2604 ciclos

signal counter    : integer range 0 to BAUDS - 1;
signal bit_idx    : integer range 0 to 7;
signal shift_reg  : std_logic_vector(7 downto 0);
```

### La clave del receptor: muestreo en el centro del bit

A diferencia del transmisor, el receptor **no sabe exactamente cuándo empieza cada bit**. Su estrategia es:

1. Detectar el flanco de bajada del start bit.
2. Esperar **medio período** (`HALF_BAUD = 2604 ciclos`) para estar en el **centro** del start bit.
3. A partir de ahí, muestrear cada **periodo completo** (`BAUDS = 5208 ciclos`), quedando siempre en el centro de cada bit.

```
                ← medio baud →←—— baud ——→←—— baud ——→
rx:   ‾‾‾‾‾‾‾|_______________|___D0____|___D1____| ...
              ↑               ↑         ↑
          flanco bajada    muestrea   muestrea
          (detectado en    start bit  D0        ...
           IDLE)
```

> Muestrear en el centro maximiza la tolerancia al ruido y a pequeñas diferencias de velocidad entre transmisor y receptor.

### Diagrama de estados (FSM)

```
        rx='0'          counter=HALF_BAUD-1      bit_idx=7 & counter=BAUDS-1
 IDLE ─────────► START ──────────────────► DATA ───────────────────────────► STOP
                                            │                                   │
                                            │ counter=BAUDS-1                   │ rx='1' → rx_data=shift_reg
                                            │ shift_reg(bit_idx)<=rx            │          rx_flag='1'
                                            │ bit_idx++                         │
                                            └─── (si bit_idx<7)─────────────────┘
                                                                         siempre → IDLE
```

### Paso a paso por estado

#### IDLE — Espera el start bit

```vhdl
when IDLE =>
    bit_idx <= 0;
    counter <= 0;
    if rx = '0' then      -- Detecta flanco de bajada
        state <= START;
    end if;
```

> La línea `rx` normalmente está en `'1'`. Un `'0'` indica el inicio de una trama.

---

#### START — Espera el centro del start bit

```vhdl
when START =>
    if counter = HALF_BAUD - 1 then
        counter <= 0;
        state   <= DATA;
    else
        counter <= counter + 1;
    end if;
```

> Espera 2603 ciclos (≈ medio bit). Al terminar, el puntero de tiempo queda **centrado** sobre los bits de datos subsiguientes.

---

#### DATA — Muestrea los 8 bits de datos

```vhdl
when DATA =>
    if counter = BAUDS - 1 then
        counter            <= 0;
        shift_reg(bit_idx) <= rx;    -- Captura el bit (LSB primero)
        if bit_idx = 7 then
            bit_idx <= 0;
            state   <= STOP;
        else
            bit_idx <= bit_idx + 1;
        end if;
    else
        counter <= counter + 1;
    end if;
```

> Cada 5208 ciclos se lee `rx` y se almacena en `shift_reg`. El primer bit leído va a `shift_reg(0)` (D0, LSB), el último a `shift_reg(7)` (D7, MSB).

---

#### STOP — Verifica el stop bit y entrega el dato

```vhdl
when STOP =>
    if counter = BAUDS - 1 then
        counter <= 0;
        if rx = '1' then       -- Stop bit válido
            rx_data <= shift_reg;
            rx_flag <= '1';    -- Pulso de 1 ciclo: dato disponible
        end if;
        state <= IDLE;         -- Siempre regresa a IDLE
    else
        counter <= counter + 1;
    end if;
```

> Si el stop bit es `'1'`, la trama es válida: se transfiere `shift_reg` a `rx_data` y se emite `rx_flag` por **exactamente 1 ciclo de reloj**. Si el stop bit es `'0'` (error de encuadre), el dato se descarta silenciosamente pero la FSM regresa a IDLE igualmente.

---

## Comparativa: Transmisor vs Receptor

| Aspecto              | Transmisor                          | Receptor                              |
|----------------------|-------------------------------------|---------------------------------------|
| Inicio de trama      | `strt='1'` del usuario              | Detección de `rx='0'`                 |
| Sincronización       | Genera el timing                    | Se sincroniza al inicio del start bit |
| Técnica de timing    | Cuenta `BAUDS` por bit              | Espera `HALF_BAUD`, luego `BAUDS`     |
| Registro de datos    | `shift_reg(9:0)` con start/stop     | `shift_reg(7:0)` solo datos           |
| Señal de control     | `tx_flag` en alto durante tx        | `rx_flag` pulso de 1 ciclo            |
| Dirección de bits    | LSB primero (`shift_reg(0)` → `tx`) | LSB primero (`rx` → `shift_reg(0)`)   |

---

## Flujo completo de una comunicación

```
Sistema externo                                      FPGA
──────────────                                  ──────────────

1. FPGA activa strt='1'  ──────────────────────► UART_Tx arma trama
                                                   shift_reg <= '1' & data & '0'

2. tx envía trama serial ──────────────────────► Línea RX del receptor externo
   0 D0 D1 D2 D3 D4 D5 D6 D7 1

3. Receptor externo responde ──────────────────► rx línea de la FPGA
   (otro byte)

4. UART_Rx detecta start ──────────────────────► state <= START
   bit en rx='0'

5. Muestrea 8 bits ────────────────────────────► shift_reg completo

6. Verifica stop bit ──────────────────────────► rx_data <= shift_reg
                                                   rx_flag <= '1' (1 ciclo)

7. Sistema lee rx_data ────────────────────────► Dato disponible para procesamiento
   cuando rx_flag='1'
```

---

## Notas de implementación

- **Metaestabilidad:** Para uso en FPGA real, la señal `rx` debe pasar por un **doble flip-flop de sincronización** antes de entrar al receptor, dado que es una señal asíncrona externa.
- **Baudrate:** Cambiando `BAUDS = F_clk / baudrate_deseado` se puede configurar cualquier velocidad estándar (115200, 57600, 19200, etc.).
- **`rx_flag` es un pulso de 1 ciclo:** El sistema que lee `rx_data` debe capturarlo inmediatamente o usar un latch/registro adicional.
- **Error de encuadre:** El receptor actual descarta silenciosamente tramas con stop bit inválido. Para aplicaciones críticas se puede agregar una señal `frame_error`.
