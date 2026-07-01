# Control de Servomotores por UART — De 1 a 6 servos

## Descripción General

Ficha técnica de los proyectos de `Pruebas_Servo_Fisicas/` y las GUIs de `GUI_Python/`.
A diferencia de las lecciones 6-8 (cinemática), esta carpeta documenta la **capa de
actuación física**: cómo se mueve cada servomotor real del brazo, en progresión de
complejidad creciente. Todos los proyectos reutilizan `PWM_50Hz` (documentado en
[`3_Modulo_PWM_50Hz`](../3_Modulo_PWM_50Hz/README.md)) y `uart_rx`
(documentado en [`4_Comunicacion_UART`](../4_Comunicacion_UART/README.md)) — esta lección
no repite esos módulos, se enfoca en cómo se combinan para controlar servos.

> **Universidad Militar Nueva Granada**

---

## Progresión de proyectos

| Proyecto | Qué agrega | Archivos VHDL propios |
|---|---|---|
| `Servo_1Grado/` | Control de 1 servo con `PWM_50Hz`, sin UART | `Bloques_Servo.vhd` |
| `Servo_Mov_Step/` | Movimiento escalonado (incrementos de ángulo controlados) | `Blocks_Interrupt.vhd`, `dutycycle_servo_controller.vhd`, `servo_controller_top.vhd` |
| `Servo_1Grado_MultiplesPWM/` | Varios generadores PWM independientes en paralelo | mismo set que `Servo_1Grado` |
| `Servo_UART/` | Reemplaza el control físico por comandos UART: 4 servos en paralelo, **mismo** dutycycle para los 4 | `Bloques_Servo_UART.vhd`, `dutycycle_generator_uart.vhd` |
| `6Servos_Indep/` | 6 servos **independientes** vía UART, cada uno con su propio ángulo — es el que alimenta el brazo real de 6 grados de libertad | `Bloques_Servo6_UART.vhd`, `uart_cmd_decoder.vhd`, `angle_to_dutycycle.vhd` |

---

## `6Servos_Indep` — arquitectura de referencia (integra las lecciones 3 y 4)

```
rx (UART) -> [uart_rx] -> data_out, data_rdy -> [uart_cmd_decoder] -> angle_0..angle_5
                                                                            |
                                                              [angle_to_dutycycle] x6
                                                                            |
                                                                   [PWM_50Hz] x6 -> pwm_out1..6
```

### Protocolo UART: 3 bytes por comando

Implementado en `uart_cmd_decoder.vhd` con una FSM de 3 estados (`WAIT_HEADER → WAIT_ID → WAIT_ANGLE`):

| Byte | Rango | Significado |
|---|---|---|
| 1 (header) | `0xAA` fijo | Marca el inicio de un comando — si no coincide, se ignora y se sigue esperando |
| 2 (ID) | `0x00`-`0x05` | Qué servo se controla (6 posibles); un ID fuera de rango reinicia la búsqueda de header |
| 3 (ángulo) | `0x00`-`0xB4` (0-180) | Ángulo deseado en grados; valores mayores a 180 se saturan a 180 |

Si llega un byte inválido en cualquier paso, la FSM vuelve a `WAIT_HEADER` — **auto-resincronización** sin necesitar reset externo.

### `angle_to_dutycycle`: de grados a ciclos de reloj

```vhdl
d := 25000 + (a * 556);   -- a: angulo 0-180, d: dutycycle en ciclos de reloj (20 bits)
```

| Ángulo | `dutycycle` (ciclos) | Tiempo en alto @ 50 MHz |
|---|---|---|
| `0°` | `25 000` | `0.5 ms` |
| `90°` | `75 020` | `1.5 ms` |
| `180°` | `125 040` | `2.5 ms` |

> Nota: estos valores de pulso (0.5-2.5 ms) son un rango típico para servos de mayor
> recorrido angular (p. ej. MG996R/RDS3225), distinto del rango 1.0-2.0 ms usado como
> ejemplo genérico en la lección 3 — cada modelo de servo tiene su propio rango de pulso,
> hay que verificarlo en la hoja de datos antes de ajustar esta fórmula.

### Inversión de reset

```vhdl
rst_int <= not rst;  -- el boton de la Cyclone IV es activo en bajo, la logica interna espera activo en alto
```

Detalle de integración con hardware físico que vale la pena recordar al portar cualquier
proyecto de este semillero a una placa distinta.

---

## `Servo_UART` (versión anterior, 4 servos con el mismo dutycycle)

Variante más simple: un solo byte por comando (vía `dutycycle_generator_uart`) controla el
mismo `dutycycle` para 4 servos en paralelo — no hay ID de servo individual ni ángulo con
signo de rango extendido como en `6Servos_Indep`. Útil como punto de comparación para ver
cómo evolucionó el protocolo hasta llegar a la versión de 6 servos independientes.

---

## `GUI_Python/` — paneles de control en PC

| Script | Descripción |
|---|---|
| `GUI_1Servo.py` | GUI mínima (tkinter) para probar 1 servo por UART |
| `6Servo.py` | Panel completo (PyQt5) para los 6 servos: sliders + campos numéricos, un `QComboBox` para el puerto serial. Envía el protocolo de 3 bytes (`0xAA`, ID, ángulo) descrito arriba — ver cabecera del script |
| `Modos_6Servos.py` | Extiende `6Servo.py` con distintos modos de operación (secuencias predefinidas, etc.) |
| `Matrices.py` | Utilidad de conversión de matrices decimales a formato de punto fijo (Q4, escala ×16) para pegarlas directamente como constantes en un testbench VHDL — mismo principio de conversión Q2.13 usado en las lecciones 6 y 7, pero con otra escala (Q4 en vez de Q2.13) para un caso de prueba distinto (cadena de 6 matrices genéricas, no necesariamente el robot de este semillero) |

### Requisitos

```
pip install pyserial pyqt5
```

(`GUI_1Servo.py` usa `tkinter`, incluido con la instalación estándar de Python en Windows.)

---

*Ver también: [Módulo PWM 50Hz (lección 3)](../3_Modulo_PWM_50Hz/README.md) ·
[Comunicación UART (lección 4)](../4_Comunicacion_UART/README.md)*
