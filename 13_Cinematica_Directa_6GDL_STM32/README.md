# Cinemática Directa 6 GDL en STM32 — comparación contra la FPGA

## Resumen

La [lección 12](../12_Cinematica_Directa_6GDL/README.md) implementó la cinemática directa del
brazo de 6 GDL real en VHDL sobre FPGA. Esta lección hace **exactamente el mismo cálculo**
(mismas fórmulas, mismo robot, mismos casos de prueba) en un STM32F767ZI (Cortex-M7, FPU doble
precisión), en dos configuraciones de reloj gemelas (216 MHz y 16 MHz), para responder: *¿cuánto
más rápido es hardware dedicado (FPGA) que software en un microcontrolador rápido?*

**Conclusión corta:** el STM32 a su velocidad máxima es entre **8.6× y 12.8× más lento** que la
FPGA según el ángulo de entrada (la FPGA no varía, el STM32 sí — hasta 49% entre el mejor y el
peor caso). Bajar el STM32 a su reloj de fábrica (16 MHz) multiplica esa brecha otras ~13.5×.

> **Plataforma:** STM32F767ZI en Nucleo-144 (Cortex-M7, FPU doble precisión) · Keil µVision5,
> estilo 100% a registro (`RCC->...`, sin HAL/LL). **Universidad Militar Nueva Granada.**

---

## Tabla de Contenidos

1. [El puerto: por qué y cómo](#1-el-puerto-por-qué-y-cómo)
2. [`forward_kinematics()`](#2-forward_kinematics)
3. [Dos relojes, registro por registro](#3-dos-relojes-registro-por-registro)
4. [Medición: TIM5](#4-medición-tim5)
5. [Resultados — los 6 casos, las 2 velocidades](#5-resultados--los-6-casos-las-2-velocidades)
6. [Por qué varía el tiempo en software (y no en la FPGA)](#6-por-qué-varía-el-tiempo-en-software-y-no-en-la-fpga)
7. [FPGA vs. STM32 — comparación final](#7-fpga-vs-stm32--comparación-final)
8. [Experimento en curso: CMSIS-DSP](#8-experimento-en-curso-cmsis-dsp)
9. [Cómo compilar y correr](#9-cómo-compilar-y-correr)

---

## 1. El puerto: por qué y cómo

La FPGA no tiene FPU, así que la lección 12 usa punto fijo Q2.13 + CORDIC. El STM32F767ZI **sí
tiene FPU doble precisión** — el puerto usa `double` y `cos()/sin()/atan2()/sqrt()` de
`<math.h>` directo, en radianes y metros. Con eso, toda la cinemática cabe en ~130 líneas.

La posición se calcula con el método **recursivo eslabón por eslabón** (`Inversa2R.pdf`, pedido
del profesor — ver [lección 12, sección 3.1](../12_Cinematica_Directa_6GDL/README.md#31-segunda-correccion-el-metodo-recursivo-de-verdad-inversa2rpdf)),
no con un atajo trigonométrico. La orientación usa la **cadena de matrices** `R1·R2·...·R6`
(no la fórmula cerrada reducida de la FPGA) porque conserva el residuo de punto flotante de
`cos(π/2)` que resuelve el reparto de la singularidad igual que MATLAB.

---

## 2. `forward_kinematics()`

Geometría del brazo (metros, tabla DH de la lección 12, "regla 4": sistemas 3 y 5 coinciden con
2 y 4):

```cpp
#define L1   0.065
#define L2   0.107
#define LD4  0.140   // L3+L4
#define LD6  0.173   // L5+L6
#define EPS_SINGULARIDAD 1e-6
```

```cpp
FK_Result forward_kinematics(double theta1, double theta2, double theta3,
                              double theta4, double theta5, double theta6) {
    Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
    dh_rot(theta1,        PI/2,  R1);
    dh_rot(theta2,        0,     R2);
    dh_rot(theta3 + PI/2, PI/2,  R3);
    dh_rot(theta4 + PI/2, PI/2,  R4);
    dh_rot(theta5,       -PI/2,  R5);
    dh_rot(theta6,        0,     R6);
    mat3_mul(R1,R2,R02); mat3_mul(R02,R3,R03); mat3_mul(R03,R4,R04);
    mat3_mul(R04,R5,R05); mat3_mul(R05,R6,R06);

    // Posicion: O_i = O_(i-1) + R_(i-1)^0 * p_i, eslabon por eslabon,
    // reutilizando las mismas R1,R02,R03,R04,R05 de arriba.
    // (ver codigo fuente para el detalle de cada eslabon)

    double R11=R06[0][0], R21=R06[1][0], R31=R06[2][0], R32=R06[2][1], R33=R06[2][2];
    double mag = sqrt(R11*R11 + R21*R21);
    double pitch = atan2(-R31, mag);
    double yaw, roll;
    if (mag < EPS_SINGULARIDAD) { yaw = 0.0; roll = PI; }          // gimbal lock: estandarizado
    else { yaw = atan2(R21, R11); roll = atan2(R32, R33); }
    return { x, y, z, yaw, pitch, roll };
}
```

Código completo: [`FK_6R_Geometrico_STM32.cpp`](FK_6R_Geometrico_STM32.cpp) (216MHz) /
[`FK_6R_Geometrico_STM32_HSI16MHz.cpp`](FK_6R_Geometrico_STM32_HSI16MHz.cpp) (16MHz) — son
idénticos salvo el reloj.

> **Gotcha real:** las 11 matrices `Mat3` (posición + orientación) desbordaron el `Stack_Size`
> por defecto de Keil (1 KB) → el chip se reiniciaba en bucle sin imprimir nada. Se subió a
> `0x1000` (4 KB) en `startup_stm32f767xx.s` — insignificante frente a los 512 KB de RAM del F767.

---

## 3. Dos relojes, registro por registro

| | 216 MHz (máximo real) | 16 MHz (reset, sin PLL) |
|---|---|---|
| Fuente | PLL: `HSI(16MHz)/PLLM(16)=1MHz ×PLLN(432)=432MHz /PLLP(2)=216MHz` | HSI directo, sin PLL |
| Requiere | **Over-drive** (`PWR->CR1` `ODEN`+`ODSWEN`) — sube el voltaje del núcleo | Nada — es el estado de fábrica |
| Flash | `FLASH->ACR = 7` wait states + prefetch + ART | 0 wait states |
| APB1 | `/4` → 54 MHz | `/1` → 16 MHz |
| **TIM5** | **108 MHz** (APB1 prescaler ≠1 → el reloj de timers se dobla, regla del F7) | **16 MHz** (prescaler =1, no se dobla) |
| USART3 `BRR` | calculado para APB1=54MHz | calculado para APB1=16MHz |

Todo lo demás (`USART3_Init`, `TIM5_Init`, `Boton_Init`, `forward_kinematics`) es el mismo código
en los dos proyectos.

---

## 4. Medición: TIM5

Reemplaza a `DWT->CYCCNT` (pedido del profesor, mismo patrón que el curso usa con I2C):

```cpp
TIM5->CNT = 0; TIM5->CR1 |= 1;              // arranca
FK_Result r = forward_kinematics(...);
TIM5->CR1 &= ~1;                            // para
uint32_t ticks = TIM5->CNT;                 // ticks / TIM5_CLK_MHZ = microsegundos
```

Dos mediciones por caso: **"en frío"** (1 ejecución — comparable con la FPGA, que tampoco tiene
caché) y **"promedio"** (1000 ejecuciones — costo en estado estable, caché ya caliente).

---

## 5. Resultados — los 6 casos, las 2 velocidades

3 casos base (los de la lección 12) + 3 casos EXTRA para acotar el rango real de tiempo dentro
del espacio físico de un servo (0°-180°): **EXTRA-1** (mejor caso "a ojo", los 6 ángulos en 90°),
**EXTRA-2** (peor caso, ángulos irregulares) y **EXTRA-3** (mejor caso *real*, encontrado por
álgebra simbólica — ver sección 6).

| Caso | θ1..θ6 (grados) | 216MHz, promedio | 16MHz, promedio |
|---|---|---|---|
| 1 — extendido | `0,0,0,0,0,0` | 38.546 µs | 520.625 µs |
| 2 — singularidad | `0,0,0,0,90,90` | 37.370 µs | 504.438 µs |
| 3 — genérico | `30,20,-15,45,60,-70` | 50.889 µs | 687.000 µs |
| EXTRA-1 — "mejor" a ojo | `90,90,90,90,90,90` | 42.352 µs | 571.688 µs |
| **EXTRA-2 — peor caso** | `137.6,23.9,168.2,74.5,109.3,41.7` | **52.037 µs (máx)** | **702.438 µs (máx)** |
| **EXTRA-3 — mejor caso real** | `0,0,-90,-90,90,0` | **34.917 µs (mín)** | **471.375 µs (mín)** |

Posición y orientación coinciden exactas con lo esperado en los 6 casos, en las 2 velocidades
(logs completos: [216MHz](output_2026-09-01_STM32_6GDL_216MHz_EXTRA3_confirmado.log),
[16MHz](output_2026-09-01_STM32_6GDL_16MHz_completo.log)).

**Cosas que valen la pena notar de esta tabla:**
- **Ratio 16MHz/216MHz = 13.4985–13.5000 en los 6 casos** (variación de 0.0015) — el tiempo
  escala linealmente con el reloj, sin importar el caso.
- **Rango peor/mejor caso = 49.0% en las dos velocidades** (52.037/34.917 = 702.438/471.375 =
  1.490) — la misma proporción hasta la 4ª cifra decimal.

---

## 6. Por qué varía el tiempo en software (y no en la FPGA)

| # | Hallazgo |
|---|---|
| 1 | El costo es en **ciclos de CPU**, no en tiempo — por eso escala 13.5× exacto con el reloj (sección 5). |
| 2 | La primera ejecución ("en frío") paga un costo de caché de instrucción (~5-13% extra) que desaparece en el promedio — mayor en proporción a 216MHz porque ahí Flash tiene 7 wait states (sección 3). |
| 3 | `cos()`/`sin()` de `<math.h>` (ARM Compiler 6 `libm`) tienen ruta rápida para ángulos "limpios" (0°, múltiplos de 90°) y ruta lenta (reducción de rango completa) para ángulos genéricos — por eso el Caso 3 tarda ~32% más que el Caso 1 con el mismo camino de código, solo cambia el *valor* de los ángulos. |
| 4 | El Caso 2 (singularidad) es más rápido que el EXTRA-1 (ángulos "limpios" a propósito) porque **evita 2 llamadas a `atan2()`** (rama `if (mag < EPS)`) — ahorrarse una llamada completa pesa más que solo tomar la ruta rápida dentro de ella. |
| 5 | **Los dos efectos (3 y 4) se suman.** Con `sympy` se encontró `θ3=-90°,θ4=-90°` — cancela el offset `+90°` de `dh_rot()`, dejando 5 de 6 ángulos en 0° exacto — **y** sigue cayendo en la singularidad (`mag=0`, verificado antes de tocar el firmware). Resultado (EXTRA-3): **6.6% más rápido que el Caso 2**, en las dos velocidades por igual (216MHz: 6.56%; 16MHz: 6.55%). |

**Consecuencia para control en tiempo real:** en la FPGA, el peor caso **es** el caso típico — el
CORDIC tarda lo mismo siempre. En el STM32, el peor caso realista es 49% más lento que el mejor
— y eso no se puede leer del código ni del datasheet, hay que medirlo empíricamente con casos
diseñados para ello (como EXTRA-2/EXTRA-3 aquí).

---

## 7. FPGA vs. STM32 — comparación final

| | FPGA (50 MHz) | STM32 @ 216 MHz | STM32 @ 16 MHz |
|---|---|---|---|
| Tiempo | **~4.05 µs, siempre** | 34.9 – 52.0 µs (según ángulo) | 471.4 – 702.4 µs (según ángulo) |
| Veces más lento que la FPGA | 1× | **8.6× – 12.8×** | **116.4× – 173.4×** |

La FPGA no solo gana en velocidad — gana en **previsibilidad**: su tiempo no depende del ángulo
de entrada, propiedad clave para control en tiempo real que ningún reloj de CPU compra en
software.

---

## 8. Experimento en curso: CMSIS-DSP

Pregunta de los profesores: ¿existe una librería que dé tiempo **constante** sin importar el
ángulo (a diferencia del Hallazgo 3)? Candidata: **CMSIS-DSP** (`arm_sin_f32`/`arm_cos_f32`,
tabla + interpolación en `float` en vez de reducción de rango en `double`).

Proyecto separado (no toca el `.cpp` ya validado):
[`FK_6R_Geometrico_STM32_CMSIS/`](FK_6R_Geometrico_STM32_CMSIS/).

**Estado: en debugging.** El chip se reinicia en bucle (HardFault) al calcular el Caso 2 con
CMSIS-DSP — el primer ángulo de 90° que se le pasa a `arm_sin_f32`/`arm_cos_f32` en todo el
programa. Sospechas en orden de probabilidad: (1) variante de librería CMSIS-DSP mal emparejada
con el FPU al agregarla vía Manage Run-Time Environment, (2) caso límite real de la tabla en
π/2 exacto. Esta sección se actualiza con la causa raíz y los datos reales cuando se resuelva.

---

## 9. Cómo compilar y correr

1. Abrir `FK_6R_Geometrico_STM32.uvprojx` (216MHz) o `FK_6R_Geometrico_STM32_HSI16MHz.uvprojx`
   (16MHz) en Keil µVision5 — mismo target `STM32F767ZITx`.
2. `Build` (F7) y `Download` (Ctrl+F5), con el ST-LINK de la Nucleo-144 conectado.
3. Terminal serie (HTerm/PuTTY) al puerto COM del ST-LINK, 9600 baudios, 8N1.
4. Al resetear corre `run_all_cases()` una vez — el botón de usuario (B1) lo repite.
