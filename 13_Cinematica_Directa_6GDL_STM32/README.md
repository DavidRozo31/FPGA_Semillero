# Cinemática Directa 6 GDL en STM32 — dos relojes, un mismo cálculo, comparación contra la FPGA

## Descripción General

La [lección 12](../12_Cinematica_Directa_6GDL/README.md) implementó la cinemática directa del
brazo de 6 GDL real en VHDL, sobre FPGA. Esta lección hace el mismo cálculo — **exactamente las
mismas fórmulas, mismo robot, mismos casos de prueba** — pero en un microcontrolador STM32F767ZI
(Cortex-M7, con FPU de doble precisión), para responder una pregunta muy concreta: *¿cuánto más
rápido es hardware dedicado en FPGA que software corriendo en un microcontrolador rápido?*

Se implementaron **dos proyectos gemelos**, idénticos en todo excepto el reloj, para separar dos
preguntas distintas:

1. **FPGA vs. STM32** — con el STM32 a su velocidad máxima real (216 MHz).
2. **¿Cuánto pesa el reloj mismo?** — el mismo STM32, mismo código, a su velocidad de fábrica
   (16 MHz, sin PLL), sin tocar una sola línea de la lógica de cálculo.

> **Plataforma:** STM32F767ZI en Nucleo-144 (Cortex-M7, FPU doble precisión) · Keil µVision5
> **Estilo:** 100% a registro (`RCC->...`, `GPIOD->...`), sin HAL ni LL — mismo estilo que el
> resto de proyectos STM32 del semillero.
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Por qué portar a STM32 en vez de reusar Q2.13/CORDIC](#1-por-qué-portar-a-stm32-en-vez-de-reusar-q213cordic)
2. [La geometría del brazo, en C](#2-la-geometría-del-brazo-en-c)
3. [Matrices DH genéricas: `dh_rot()` y `mat3_mul()`](#3-matrices-dh-genéricas-dh_rot-y-mat3_mul)
4. [`forward_kinematics()`, paso a paso](#4-forward_kinematics-paso-a-paso)
5. [Dos configuraciones de reloj, explicadas registro por registro](#5-dos-configuraciones-de-reloj-explicadas-registro-por-registro)
6. [UART3, DWT y el botón de usuario](#6-uart3-dwt-y-el-botón-de-usuario)
7. [Cómo se mide: "en frío" vs. promedio de 1000](#7-cómo-se-mide-en-frío-vs-promedio-de-1000)
8. [La salida real del UART](#8-la-salida-real-del-uart)
9. [Análisis de los resultados](#9-análisis-de-los-resultados)
10. [FPGA vs. STM32 — la comparación final](#10-fpga-vs-stm32--la-comparación-final)
11. [Cómo compilar y correr en Keil](#11-cómo-compilar-y-correr-en-keil)

---

## 1. Por qué portar a STM32 en vez de reusar Q2.13/CORDIC

La FPGA no tiene unidad de punto flotante, así que todo el proyecto de la lección 12 está en
punto fijo Q2.13 con CORDIC para seno/coseno/atan2. El STM32F767ZI **sí tiene FPU de doble
precisión** — no hace falta replicar nada de eso. El puerto usa `double` y las funciones
`cos()`, `sin()`, `atan2()`, `sqrt()` de `<math.h>` directamente, en radianes y metros, tal cual.

Con eso, todo el código de cinemática (secciones 2-4 de este documento) cabe en menos de 130
líneas — mucho más simple que su equivalente en VHDL — pero eso mismo es lo que hace interesante
la comparación de velocidad: ¿cuánto cuesta esa simplicidad en tiempo de ejecución?

---

## 2. La geometría del brazo, en C

Mismas longitudes que la tabla DH de la lección 12, ya en metros y ya combinadas donde hace
falta (`Ld4 = L3+L4`, `Ld6 = L5+L6`, porque los sistemas 3 y 5 coinciden con 2 y 4 — "regla 4"):

```cpp
#define L1   0.065
#define L2   0.107
#define LD4  0.140   // L3+L4 = 0.095+0.045
#define LD6  0.173   // L5+L6 = 0.07+0.103

#define EPS_SINGULARIDAD 1e-6
```

`EPS_SINGULARIDAD` es el mismo criterio de siempre para detectar gimbal lock: si
`|cos(pitch)|` cae por debajo de este umbral, se está en la singularidad.

---

## 3. Matrices DH genéricas: `dh_rot()` y `mat3_mul()`

En vez de escribir las 6 matrices `R1..R6` a mano (como en VHDL, donde cada `mat_r0X_elems` está
optimizado para gastar el mínimo de multiplicaciones), aquí se puede dar el lujo de una función
genérica — el STM32 con FPU no tiene el problema de recursos de la FPGA:

```cpp
void dh_rot(double theta, double alpha, Mat3 out) {
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);
    out[0][0] = ct;   out[0][1] = -st*ca;  out[0][2] =  st*sa;
    out[1][0] = st;   out[1][1] =  ct*ca;  out[1][2] = -ct*sa;
    out[2][0] = 0;    out[2][1] =  sa;     out[2][2] =  ca;
}
```

Es la matriz `Rz(theta)*Rx(alpha)` genérica de cualquier fila de una tabla DH — la misma fórmula
que usa `Metodo_Geometrico_RPY_6R.m` matriz por matriz. `mat3_mul()` es una multiplicación 3×3
directa, sin trucos.

---

## 4. `forward_kinematics()`, paso a paso

### 4.1 Posición — parte A (hasta el sistema 4)

```cpp
double phi2  = theta2;
double phi23 = theta2 + theta3;

double r4 = L2*cos(phi2) + LD4*cos(phi23);
double z4 = L1 + L2*sin(phi2) + LD4*sin(phi23);
double x4 = r4*cos(theta1);
double y4 = r4*sin(theta1);
```

Idéntico al atajo geométrico de siempre (trigonometría plana, sin matrices) — ver
[lección 12, sección 3](../12_Cinematica_Directa_6GDL/README.md#3-el-hallazgo-la-posicion-ya-no-es-independiente-de-la-muñeca)
para la derivación completa de por qué solo llega hasta el sistema 4.

### 4.2 Orientación — cadena de matrices, NO la fórmula cerrada

```cpp
Mat3 R1, R2, R3, R4, R5, R6, R02, R03, R04, R05, R06;
dh_rot(theta1,          PI/2,  R1);
dh_rot(theta2,          0,     R2);
dh_rot(theta3 + PI/2,   PI/2,  R3);
dh_rot(theta4 + PI/2,   PI/2,  R4);
dh_rot(theta5,         -PI/2,  R5);
dh_rot(theta6,          0,     R6);

mat3_mul(R1, R2, R02);
mat3_mul(R02, R3, R03);
mat3_mul(R03, R4, R04);
mat3_mul(R04, R5, R05);
mat3_mul(R05, R6, R06);
```

Aquí hay una decisión de diseño importante, heredada del puerto del brazo de 5R: la FPGA usa una
fórmula **cerrada y reducida** (`mat_r06_elems.vhd`, obtenida con `sympy`/CSE) para gastar el
mínimo de multiplicadores — pero esa reducción simplifica algebraicamente términos como
`cos(pi/2)` a `0` **en papel**. En `double`, `cos(pi/2)` no da exactamente `0` (da
`6.12e-17`, un residuo fijo de cómo se representa `pi/2` en binario) — y ese residuo es
justamente lo que "resuelve" el reparto entre `yaw` y `roll` en la singularidad, de forma
consistente con cómo lo resuelve MATLAB (que también multiplica matrices, no usa la fórmula
reducida). Si aquí se usara la fórmula cerrada de la FPGA, el STM32 caería en un
`atan2(0,0)` en la singularidad — un caso degenerado distinto, que no reconstruye la matriz real.

### 4.3 Posición — parte B (aporte de la muñeca) y extracción RPY

```cpp
double R11 = R06[0][0], R21 = R06[1][0], R31 = R06[2][0];
double R32 = R06[2][1], R33 = R06[2][2];
double R13 = R06[0][2], R23 = R06[1][2];  // columna 3 -- direccion de Z6 en el mundo

double x = x4 + LD6*R13;
double y = y4 + LD6*R23;
double z = z4 + LD6*R33;

double mag   = sqrt(R11*R11 + R21*R21);
double pitch = atan2(-R31, mag);
double yaw, roll;

if (mag < EPS_SINGULARIDAD) {
    yaw  = 0.0;
    roll = PI;
} else {
    yaw  = atan2(R21, R11);
    roll = atan2(R32, R33);
}
```

`R13,R23,R33` (columna 3 de `R0_6`) son la dirección de `Z6` en el mundo — se reutilizan tal
cual para la posición, mismo razonamiento que en VHDL (lección 12, sección 3). `pitch` se
calcula siempre; `yaw`/`roll` se estandarizan a `0`/`180°` en la singularidad, acordado con el
profesor, mismo criterio en las tres plataformas (FPGA, MATLAB, STM32).

---

## 5. Dos configuraciones de reloj, explicadas registro por registro

### 5.1 216 MHz — [`FK_6R_Geometrico_STM32.cpp`](FK_6R_Geometrico_STM32.cpp)

```cpp
void SystemClock_216MHz(void) {
    RCC->APB1ENR |= (1<<28);               // PWREN
    PWR->CR1 |= (0b11<<14);                // VOS = Scale 1
    PWR->CR1 |= (1<<16);                   // ODEN (Over-drive)
    while (!(PWR->CSR1 & (1<<16)));        // espera ODRDY
    PWR->CR1 |= (1<<17);                   // ODSWEN (conmuta a Over-drive)
    while (!(PWR->CSR1 & (1<<17)));        // espera ODSWRDY

    FLASH->ACR = 7 | (1<<8) | (1<<9);      // 7 wait states + prefetch + ART

    // HSI=16MHz -> PLLM=16 -> 1MHz -> PLLN=432 -> 432MHz -> PLLP=2 -> 216MHz
    RCC->PLLCFGR = (16UL<<0) | (432UL<<6) | (0UL<<16) | (9UL<<24);

    RCC->CR |= (1<<24);                    // PLLON
    while (!(RCC->CR & (1<<25)));          // espera PLLRDY

    RCC->CFGR |= (0b101<<10);              // APB1 = AHB/4  -> 54 MHz
    RCC->CFGR |= (0b100<<13);              // APB2 = AHB/2  -> 108 MHz

    RCC->CFGR |= (0b10<<0);                // SW = PLL
    while (((RCC->CFGR>>2) & 0b11) != 0b10); // espera SWS = PLL

    SystemCoreClock = 216000000UL;
}
```

216 MHz es el **máximo real** del STM32F767ZI, y solo se alcanza con el modo **Over-drive**
activado (`ODEN`/`ODSWEN` — sube el voltaje del núcleo por encima del rango normal). La cadena
de PLL: `HSI (16MHz) / PLLM(16) = 1MHz`, `× PLLN(432) = 432MHz`, `/ PLLP(2) = 216MHz`. A esa
velocidad, Flash necesita **7 ciclos de espera** por acceso (`FLASH->ACR = 7 | prefetch | ART`)
— dato que vuelve a aparecer en la sección 9.

### 5.2 16 MHz, sin PLL — [`FK_6R_Geometrico_STM32_HSI16MHz.cpp`](FK_6R_Geometrico_STM32_HSI16MHz.cpp)

```cpp
void SystemClock_HSI16MHz(void) {
    RCC->CR |= (1<<0);                     // HSION (por si acaso)
    while (!(RCC->CR & (1<<1)));           // espera HSIRDY

    RCC->CFGR &= ~(0b11<<0);               // SW = HSI (000)
    while (((RCC->CFGR>>2) & 0b11) != 0b00); // espera SWS = HSI

    // HPRE, PPRE1, PPRE2 en /1 (reset por defecto) -> AHB=APB1=APB2=16MHz
    SystemCoreClock = 16000000UL;
}
```

Esta es, literalmente, la configuración con la que arranca el chip después de cualquier reset —
HSI interno (oscilador RC de 16MHz, sin cristal externo) directo a `SYSCLK`, sin PLL, sin
Over-drive, sin wait-states de Flash. La función existe solo para dejarlo **explícito** en el
código (documentar la intención), no porque haga falta tocar ningún registro.

Todo lo demás — `USART3_Init()`, `DWT_Init()`, `Boton_Init()`, `forward_kinematics()` — es
**exactamente el mismo código** en los dos archivos. La única diferencia real entre los dos
proyectos son estas dos funciones y las constantes `AHB_CLK_HZ`/`APB1_CLK_HZ` que dependen de
ellas (`216000000`/`54000000` vs `16000000`/`16000000`, usadas para recalcular el `BRR` del UART
y el conteo del `SysTick`).

---

## 6. UART3, DWT y el botón de usuario

Reutilizados tal cual del proyecto del brazo de 5R
(`C:\Keil_Ejercicios\FK_5R_Geometrico_STM32\`), sin cambios de lógica:

- **USART3** (PD8=TX, PD9=RX, AF7) — en las Nucleo-144, el puerto virtual COM del ST-LINK está
  cableado a USART3, no a UART7. `BRR` se recalcula según `APB1_CLK_HZ`, así que el mismo
  código de inicialización da 9600 baudios reales sin importar el reloj del sistema.
- **DWT->CYCCNT** — contador de ciclos de 32 bits, resolución de 1 ciclo, se activa una vez en
  `DWT_Init()` y nunca se reinicia entre casos de prueba (por diseño: así el conteo de ciclos de
  cada caso es limpio, `delta = CYCCNT_final - CYCCNT_inicial`, sin importar cuánto tiempo lleve
  corriendo el programa).
- **Botón B1 (PC13)** — con pull-up interno por software (la Nucleo-144 no trae resistencia
  externa en ese pin), detectado por flanco de bajada con antirrebote de 30ms por `SysTick`.

---

## 7. Cómo se mide: "en frío" vs. promedio de 1000

```cpp
uint32_t c0 = DWT->CYCCNT;
FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
uint32_t ciclos_frio = DWT->CYCCNT - c0;

c0 = DWT->CYCCNT;
for (int i = 0; i < N_REPS; i++) {
    r = forward_kinematics(t1, t2, t3, t4, t5, t6);
}
uint32_t ciclos_prom = (DWT->CYCCNT - c0) / N_REPS;
```

Dos mediciones por caso, a propósito:

- **"En frío"** — una sola ejecución. Comparable directo con la FPGA, que también hace todo en
  una sola pasada por su pipeline sin nada "precalentado".
- **Promedio de 1000** — con el I-cache/D-cache ya calientes y el predictor de saltos ya
  entrenado. Mide el costo real de `forward_kinematics()` en estado estable, sin el ruido de la
  primera ejecución (ver sección 9, hallazgo 2).

---

## 8. La salida real del UART

Log completo capturado por HTerm, en las dos velocidades, con el botón de usuario presionado
varias veces para confirmar que los resultados son estables entre corridas — archivo completo en
[`output_2026-08-18_STM32_6GDL_216MHz_vs_16MHz.log`](output_2026-08-18_STM32_6GDL_216MHz_vs_16MHz.log).
Extracto (una corrida de cada velocidad):

```
=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 216MHz ===

=== Caso 1 [extendido] q=0 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=0.0 th6=0.0
  x=0.4200 m  y=-0.0000 m  z=0.0650 m
  yaw=-90.00 deg  pitch=-0.00 deg  roll=-90.00 deg
  ciclos (1 ejecucion, en frio)    = 8428  (39.019 us @ 216MHz)
  ciclos (promedio 1000 ejecuciones) = 7452  (34.500 us @ 216MHz)

=== Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=90.0 th6=90.0
  x=0.2470 m  y=0.1730 m  z=0.0650 m
  yaw=0.00 deg  pitch=90.00 deg  roll=180.00 deg
  ciclos (1 ejecucion, en frio)    = 7232  (33.481 us @ 216MHz)
  ciclos (promedio 1000 ejecuciones) = 7240  (33.519 us @ 216MHz)

=== Caso 3 [generico] ===
  entradas (grados): th1=30.0 th2=20.0 th3=-15.0 th4=45.0 th5=60.0 th6=-70.0
  x=0.2215 m  y=0.2502 m  z=0.2269 m
  yaw=-42.50 deg  pitch=-34.56 deg  roll=-37.47 deg
  ciclos (1 ejecucion, en frio)    = 10636  (49.241 us @ 216MHz)
  ciclos (promedio 1000 ejecuciones) = 10545  (48.819 us @ 216MHz)


=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 16MHz (HSI, sin PLL) ===

=== Caso 1 [extendido] q=0 ===
  ciclos (1 ejecucion, en frio)    = 7849  (490.562 us @ 16MHz)
  ciclos (promedio 1000 ejecuciones) = 7451  (465.688 us @ 16MHz)

=== Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90 ===
  ciclos (1 ejecucion, en frio)    = 7234  (452.125 us @ 16MHz)
  ciclos (promedio 1000 ejecuciones) = 7239  (452.438 us @ 16MHz)

=== Caso 3 [generico] ===
  ciclos (1 ejecucion, en frio)    = 10581  (661.312 us @ 16MHz)
  ciclos (promedio 1000 ejecuciones) = 10546  (659.125 us @ 16MHz)
```

Posición y orientación coincidieron **exactas** con lo esperado (misma verificación que la
[tabla de la lección 12, sección 7](../12_Cinematica_Directa_6GDL/README.md#7-verificación-modelsim-y-matlab)),
en las dos velocidades, en las tres corridas repetidas — confirma que el puerto a C está bien
hecho, independiente del reloj.

---

## 9. Análisis de los resultados

**Hallazgo 1 — los ciclos no cambian con el reloj, el tiempo sí.** El promedio de ciclos es
prácticamente idéntico entre 216MHz y 16MHz (7,452 vs 7,451 · 7,240 vs 7,239 · 10,545 vs
10,546). Tiene sentido: `DWT->CYCCNT` cuenta ciclos de CPU — el mismo código ejecuta el mismo
número de instrucciones sin importar qué tan rápido tiquetee el reloj. Lo que cambia es cuánto
dura cada ciclo, por eso el tiempo real sí escala ~13.5× entre las dos tarjetas (justo la
relación 216/16).

**Hallazgo 2 — la primera ejecución a 216MHz paga un costo que no aparece en ningún otro caso.**
Caso 1 "en frío" da 8,428 ciclos contra un promedio de 7,452 — 976 ciclos de diferencia, que no
se repite en los Casos 2/3 (ahí "en frío" ≈ promedio) ni en ninguna medición a 16MHz (brecha de
solo 398 ciclos). Explicación: el Caso 1 es la *primera* vez que se ejecuta
`forward_kinematics()` desde el arranque, con el I-cache totalmente frío — cada instrucción se
trae de Flash, que a 216MHz necesita **7 ciclos de espera** por acceso (sección 5.1) contra
prácticamente 0 a 16MHz. Para cuando corren los Casos 2 y 3, el código ya quedó cacheado por las
1000 repeticiones del Caso 1 — por eso ahí la brecha desaparece.

**Hallazgo 3 — la singularidad es el caso más rápido, y se explica en el código mismo.** El
Caso 2 (singularidad) evita 2 llamadas a `atan2()` — `yaw` y `roll` se fijan a constantes en la
rama `if (mag < EPS_SINGULARIDAD)` (sección 4.3) en vez de calcularse. Eso explica los ~200
ciclos de diferencia frente al Caso 1, que sí calcula las 3 (`yaw`, `pitch`, `roll`) con
`atan2()`.

---

## 10. FPGA vs. STM32 — la comparación final

| | FPGA (50 MHz) | STM32 @ 216 MHz | STM32 @ 16 MHz |
|---|---|---|---|
| Tiempo (constante / según caso) | ~3.1 µs | 33.5–48.8 µs | 452–659 µs |
| **Veces más lento que la FPGA** | 1× | **10.8×–15.7×** | **146×–212×** |

A pesar de que el STM32 a máxima velocidad tiene un reloj **4.3 veces más rápido** que la FPGA
(216MHz vs 50MHz), termina el mismo cálculo entre **10.8 y 15.7 veces más lento** — el mismo
patrón que ya se había visto con el brazo de 5R: hardware dedicado en pipeline (la FPGA calcula
todo en paralelo, ciclo a ciclo, con circuitos construidos exactamente para esta cuenta) le gana
por mucho a software secuencial (el STM32 ejecuta instrucción por instrucción, y cada
`cos()`/`sin()`/`atan2()`/`sqrt()` de doble precisión cuesta decenas a cientos de ciclos), sin
importar cuánto reloj se le meta al software. Y bajar la tarjeta a su velocidad de fábrica
(16MHz, sin PLL) multiplica esa brecha por otras ~13.5×, hasta más de 200× frente a la FPGA.

---

## 11. Cómo compilar y correr en Keil

1. Abrir `FK_6R_Geometrico_STM32.uvprojx` (216MHz) o
   `FK_6R_Geometrico_STM32_HSI16MHz.uvprojx` (16MHz) en Keil µVision5 — son dos proyectos
   independientes, mismo target `STM32F767ZITx`.
2. Compilar (`Build`) y programar (`Download`, con el ST-LINK de la Nucleo-144 conectado).
3. Abrir un terminal serie (HTerm, PuTTY, etc.) al puerto COM del ST-LINK, 9600 baudios, 8N1.
4. Al resetear la tarjeta corre automáticamente `run_all_cases()` una vez — presionar el botón
   de usuario (B1) la repite cuantas veces se quiera, para confirmar que los ciclos son estables
   entre corridas (ver sección 8, tres corridas iguales por velocidad).
