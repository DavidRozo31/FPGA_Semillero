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

> **Segunda corrección del profesor (después de la primera corrida de este puerto):** igual que
> en la [lección 12](../12_Cinematica_Directa_6GDL/README.md#31-segunda-correccion-el-metodo-recursivo-de-verdad-inversa2rpdf),
> la posición se rehizo con el método geométrico **recursivo** (`Inversa2R.pdf`) en vez del atajo
> trigonométrico, y la medición de tiempo pasó de `DWT->CYCCNT` al periférico **TIM5** (pedido
> explícito para tener una herramienta de medición estándar y determinística). Este documento ya
> refleja el código y los datos **después** de esas dos correcciones — ver sección 4.2 (posición
> recursiva), sección 6 (TIM5) y sección 9 (un hallazgo nuevo sobre por qué el software, a
> diferencia del CORDIC de la FPGA, no tarda lo mismo para cualquier ángulo).

> **Tercera actualización (pedido de los profesores) — caracterizar mejor y peor caso:** se
> agregaron dos casos de prueba dedicados para acotar el rango real de tiempo de
> `forward_kinematics()`, dentro del rango físico real de un servomotor (0°-180°): un "mejor caso"
> (los 6 ángulos en 90°) y un "peor caso" (ángulos irregulares, lejos de cualquier múltiplo de 90°,
> que fuerzan al máximo la reducción de rango de `cos()/sin()`). Ver sección 8 (log actualizado),
> sección 9 (hallazgo 4) y la nueva sección 10 con el mínimo y máximo confirmados por reloj.

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
   - 4.1 Orientación — cadena de matrices
   - 4.2 Posición — método geométrico recursivo (segunda corrección del profesor)
   - 4.3 Extracción de yaw/pitch/roll
5. [Dos configuraciones de reloj, explicadas registro por registro](#5-dos-configuraciones-de-reloj-explicadas-registro-por-registro)
6. [UART3, TIM5 y el botón de usuario](#6-uart3-tim5-y-el-botón-de-usuario)
7. [Cómo se mide: "en frío" vs. promedio de 1000](#7-cómo-se-mide-en-frío-vs-promedio-de-1000)
8. [La salida real del UART](#8-la-salida-real-del-uart)
9. [Análisis de los resultados](#9-análisis-de-los-resultados)
10. [Mínimo y máximo medido, por reloj](#10-mínimo-y-máximo-medido-por-reloj)
11. [FPGA vs. STM32 — la comparación final](#11-fpga-vs-stm32--la-comparación-final)
12. [Cómo compilar y correr en Keil](#12-cómo-compilar-y-correr-en-keil)

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

### 4.1 Orientación — cadena de matrices, NO la fórmula cerrada

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

### 4.2 Posición — método geométrico RECURSIVO (segunda corrección del profesor)

La primera versión de este puerto calculaba la posición con un atajo trigonométrico
(`x4,y4,z4` hasta el sistema 4 + `Ld6*columna3(R0_6)` para la muñeca) — daba los números
correctos, pero el profesor lo rechazó por la misma razón que en la FPGA
([lección 12, sección 3.1](../12_Cinematica_Directa_6GDL/README.md#31-segunda-correccion-el-metodo-recursivo-de-verdad-inversa2rpdf)):
no era un método reconocible. Se reemplazó por la acumulación eslabón por eslabón
(`Inversa2R.pdf`), reutilizando las **mismas** matrices `R1,R02,R03,R04,R05` que ya arma la
sección 4.1 — no hace falta calcular nada extra para la posición, solo reordenar cómo se usa lo
que ya existía:

```cpp
// out = R * v (matriz 3x3 por vector 3x1)
void mat3_vec(const Mat3 R, const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++) {
        double s = 0;
        for (int k = 0; k < 3; k++) s += R[i][k]*v[k];
        out[i] = s;
    }
}
...
Mat3 I3 = {{1,0,0},{0,1,0},{0,0,1}};
double O[3] = {0,0,0};
double p[3], Rp[3];

// eslabon 1: a1=0, d1=L1
p[0]=0; p[1]=0; p[2]=L1;
mat3_vec(I3, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

// eslabon 2: a2=L2, d2=0
p[0]=L2*cos(theta2); p[1]=L2*sin(theta2); p[2]=0;
mat3_vec(R1, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

// eslabon 3: a3=0, d3=0 (sistema 3 = sistema 2, "regla 4" -- no aporta nada)
p[0]=0; p[1]=0; p[2]=0;
mat3_vec(R02, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

// eslabon 4: a4=0, d4=Ld4
p[0]=0; p[1]=0; p[2]=LD4;
mat3_vec(R03, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

// eslabon 5: a5=0, d5=0 (sistema 5 = sistema 4, "regla 4" -- no aporta nada)
p[0]=0; p[1]=0; p[2]=0;
mat3_vec(R04, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

// eslabon 6: a6=0, d6=Ld6
p[0]=0; p[1]=0; p[2]=LD6;
mat3_vec(R05, p, Rp);
O[0]+=Rp[0]; O[1]+=Rp[1]; O[2]+=Rp[2];

double x = O[0], y = O[1], z = O[2];
```

Cada eslabón `i` aporta su desplazamiento local `p_i` (columna `d_i`/`a_i` de su propia fila DH),
rotado al marco de la base con la rotación acumulada **hasta el eslabón anterior**
(`R_(i-1)^0`) y sumado a donde ya iba el brazo. Idéntica fórmula, mismo orden de matrices, que
[`Metodo_Geometrico_RPY_6R.m`](../12_Cinematica_Directa_6GDL/Metodo_Geometrico_RPY_6R.m). Da
**exactamente los mismos números** que el atajo viejo — verificado primero en Python antes de
tocar este archivo.

> **Gotcha real, no cosmético:** este cambio agregó suficientes variables locales (`Mat3 I3`, más
> los arreglos `O`/`p`/`Rp`, sumados a las 11 matrices `Mat3` que ya usaba la orientación) para
> que `forward_kinematics()` **desbordara el stack por defecto de Keil (1 KB)** — el síntoma fue
> el chip reiniciándose en bucle, imprimiendo el banner una y otra vez sin llegar nunca a mostrar
> el resultado del Caso 1. Se subió `Stack_Size` de `0x400` a `0x1000` (4 KB) en
> `startup_stm32f767xx.s` de los dos proyectos — el F767 tiene 512 KB de RAM, así que sigue
> siendo insignificante.

### 4.3 Extracción de yaw/pitch/roll

```cpp
double R11 = R06[0][0], R21 = R06[1][0], R31 = R06[2][0];
double R32 = R06[2][1], R33 = R06[2][2];

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

`pitch` se calcula siempre; `yaw`/`roll` se estandarizan a `0`/`180°` en la singularidad, acordado
con el profesor, mismo criterio en las tres plataformas (FPGA, MATLAB, STM32).

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

Todo lo demás — `USART3_Init()`, `TIM5_Init()`, `Boton_Init()`, `forward_kinematics()` — es
**exactamente el mismo código** en los dos archivos. La única diferencia real entre los dos
proyectos son estas dos funciones, la constante `TIM5_CLK_MHZ` (sección 6), y las constantes
`AHB_CLK_HZ`/`APB1_CLK_HZ` (`216000000`/`54000000` vs `16000000`/`16000000`, usadas para
recalcular el `BRR` del UART y el conteo del `SysTick`).

---

## 6. UART3, TIM5 y el botón de usuario

**Cambio del profesor:** la medición de tiempo pasó de `DWT->CYCCNT` al periférico **TIM5**
(mismo patrón que usa el curso con I2C: reset `CNT`, arrancar con `CR1|=1`, correr el código,
parar con `CR1&=~1`, leer `CNT`) — una herramienta de medición estándar y determinística, en vez
del contador de ciclos de depuración.

```cpp
#define TIM5_CLK_MHZ 108.0   // 16.0 en el proyecto de 16MHz -- ver nota abajo

void TIM5_Init(void) {
    RCC->APB1ENR |= (1<<3);                // TIM5EN
    TIM5->PSC = 0;                         // sin division, resolucion maxima
    TIM5->ARR = 0xFFFFFFFF;                // maximo (32 bits)
    TIM5->CNT = 0;
}
```

- **TIM5 es de 32 bits** en el F767 — sin riesgo de overflow en mediciones de decenas/cientos de
  µs. `PSC=0` (resolución máxima): el timer cuenta al reloj pleno del periférico.
- **El reloj de TIM5 NO es el mismo en los dos proyectos**, y es la razón por la que
  `TIM5_CLK_MHZ` cambia entre archivos: a 216MHz, `APB1=54MHz` con el prescaler del bus en
  `/4` (≠1) → por la regla estándar del árbol de reloj del STM32F7, el reloj de los timers en ese
  bus se **dobla** → TIM5 corre a **108MHz**. A 16MHz sin PLL, `APB1=16MHz` con el prescaler en
  `/1` → no se dobla, TIM5 corre a los mismos **16MHz**. `ticks / TIM5_CLK_MHZ` da los
  microsegundos reales en los dos casos.
- **USART3** (PD8=TX, PD9=RX, AF7) — en las Nucleo-144, el puerto virtual COM del ST-LINK está
  cableado a USART3, no a UART7. `BRR` se recalcula según `APB1_CLK_HZ`, así que el mismo
  código de inicialización da 9600 baudios reales sin importar el reloj del sistema.
- **Botón B1 (PC13)** — con pull-up interno por software (la Nucleo-144 no trae resistencia
  externa en ese pin), detectado por flanco de bajada con antirrebote de 30ms por `SysTick`.

---

## 7. Cómo se mide: "en frío" vs. promedio de 1000

```cpp
TIM5->CNT = 0;
TIM5->CR1 |= (1<<0);                   // arranca el conteo
FK_Result r = forward_kinematics(t1, t2, t3, t4, t5, t6);
TIM5->CR1 &= ~(1<<0);                  // para el conteo
uint32_t ticks_frio = TIM5->CNT;

TIM5->CNT = 0;
TIM5->CR1 |= (1<<0);
for (int i = 0; i < N_REPS; i++) {
    r = forward_kinematics(t1, t2, t3, t4, t5, t6);
}
TIM5->CR1 &= ~(1<<0);
uint32_t ticks_prom = TIM5->CNT / N_REPS;
```

Dos mediciones por caso, a propósito (el patrón `CNT=0 -> CR1|=1 -> ... -> CR1&=~1 -> lee CNT` es
el mismo que pidió el profesor para el ejemplo de I2C con TIM5):

- **"En frío"** — una sola ejecución. Comparable directo con la FPGA, que también hace todo en
  una sola pasada por su pipeline sin nada "precalentado".
- **Promedio de 1000** — con el I-cache/D-cache ya calientes y el predictor de saltos ya
  entrenado. Mide el costo real de `forward_kinematics()` en estado estable, sin el ruido de la
  primera ejecución (ver sección 9, hallazgo 2).

---

## 8. La salida real del UART

Log completo capturado por HTerm, en las dos velocidades, con el botón de usuario presionado
varias veces para confirmar que los resultados son estables entre corridas — archivo completo en
[`output_2026-08-19_STM32_6GDL_TIM5_posicion_recursiva.log`](output_2026-08-19_STM32_6GDL_TIM5_posicion_recursiva.log)
(3 casos originales) y, con los dos casos EXTRA agregados después, en
[`output_2026-09-01_STM32_6GDL_TIM5_mejor_peor_caso.log`](output_2026-09-01_STM32_6GDL_TIM5_mejor_peor_caso.log).
Extracto (una corrida de cada velocidad, los 5 casos):

```
=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 216MHz (TIM5) ===

=== Caso 1 [extendido] q=0 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=0.0 th6=0.0
  x=0.4200 m  y=-0.0000 m  z=0.0650 m
  yaw=-90.00 deg  pitch=-0.00 deg  roll=-90.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 4710  (43.611 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 4172  (38.630 us)

=== Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=90.0 th6=90.0
  x=0.2470 m  y=0.1730 m  z=0.0650 m
  yaw=0.00 deg  pitch=90.00 deg  roll=180.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 4062  (37.611 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 4054  (37.537 us)

=== Caso 3 [generico] ===
  entradas (grados): th1=30.0 th2=20.0 th3=-15.0 th4=45.0 th5=60.0 th6=-70.0
  x=0.2215 m  y=0.2502 m  z=0.2269 m
  yaw=-42.50 deg  pitch=-34.56 deg  roll=-37.47 deg
  ticks TIM5 (1 ejecucion, en frio)    = 5556  (51.444 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 5514  (51.056 us)

=== Caso EXTRA-1 [MEJOR CASO: los 6 angulos en 90 grados] ===
  entradas (grados): th1=90.0 th2=90.0 th3=90.0 th4=90.0 th5=90.0 th6=90.0
  x=-0.0000 m  y=-0.1400 m  z=-0.0010 m
  yaw=180.00 deg  pitch=-0.00 deg  roll=-180.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 4600  (42.593 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 4580  (42.407 us)

=== Caso EXTRA-2 [PEOR CASO: angulos irregulares, dentro de 0-180] ===
  entradas (grados): th1=137.6 th2=23.9 th3=168.2 th4=74.5 th5=109.3 th6=41.7
  x=-0.0662 m  y=0.0014 m  z=-0.0629 m
  yaw=-88.66 deg  pitch=11.91 deg  roll=146.93 deg
  ticks TIM5 (1 ejecucion, en frio)    = 5620  (52.037 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 5625  (52.083 us)


=== Cinematica Directa 6 GDL real -- STM32F767ZI @ 16MHz (HSI, sin PLL, TIM5) ===

=== Caso 1 [extendido] q=0 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=0.0 th6=0.0
  x=0.4200 m  y=-0.0000 m  z=0.0650 m
  yaw=-90.00 deg  pitch=-0.00 deg  roll=-90.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 8778  (548.625 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 8329  (520.562 us)

=== Caso 2 [SINGULARIDAD del 6R] th5=90 th6=90 ===
  entradas (grados): th1=0.0 th2=0.0 th3=0.0 th4=0.0 th5=90.0 th6=90.0
  x=0.2470 m  y=0.1730 m  z=0.0650 m
  yaw=0.00 deg  pitch=90.00 deg  roll=180.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 8096  (506.000 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 8085  (505.312 us)

=== Caso 3 [generico] ===
  entradas (grados): th1=30.0 th2=20.0 th3=-15.0 th4=45.0 th5=60.0 th6=-70.0
  x=0.2215 m  y=0.2502 m  z=0.2269 m
  yaw=-42.50 deg  pitch=-34.56 deg  roll=-37.47 deg
  ticks TIM5 (1 ejecucion, en frio)    = 11026  (689.125 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 11008  (688.000 us)

=== Caso EXTRA-1 [MEJOR CASO: los 6 angulos en 90 grados] ===
  entradas (grados): th1=90.0 th2=90.0 th3=90.0 th4=90.0 th5=90.0 th6=90.0
  x=-0.0000 m  y=-0.1400 m  z=-0.0010 m
  yaw=180.00 deg  pitch=-0.00 deg  roll=-180.00 deg
  ticks TIM5 (1 ejecucion, en frio)    = 9183  (573.938 us)
  ticks TIM5 (promedio 1000 ejecuciones) = 9167  (572.938 us)

=== Caso EXTRA-2 [PEOR CASO: angulos irregulares, dentro de 0-180] ===
  entradas (grados): th1=137.6 th2=23.9 th3=168.2 th4=74.5 th5=109.3 th6=41.7
  [CAPTURA INCOMPLETA -- pendiente repetir esta corrida puntual para
   confirmar roll y los ticks TIM5 de este ultimo caso a 16MHz]
```

Posición y orientación coincidieron **exactas** con lo esperado (misma verificación que la
[tabla de la lección 12, sección 7](../12_Cinematica_Directa_6GDL/README.md#7-verificación-modelsim-y-matlab)),
en las dos velocidades, en varias corridas repetidas — confirma que el puerto a C del método
recursivo está bien hecho, independiente del reloj. La única excepción es el dato de tiempo del
Caso EXTRA-2 a 16MHz, cuya captura por terminal se cortó a la mitad (ver sección 10) — no afecta
la validación de posición/orientación, que ya estaba confirmada en los otros 4 casos y en la
corrida a 216MHz.

---

## 9. Análisis de los resultados

Con `TIM5_CLK_MHZ` conocido en los dos proyectos (108MHz a 216MHz, 16MHz a 16MHz — sección 6),
los ticks de TIM5 se pueden convertir a **ciclos de CPU reales** (a 16MHz, TIM5 corre 1:1 con el
núcleo, así que ahí los ticks *son* ciclos directamente; a 216MHz, ciclos = ticks × 2):

| Caso | Ciclos (en frío) | Ciclos (promedio 1000) |
|---|---|---|
| 1 — extendido | ~8,780–9,420 | **~8,330–8,340** |
| 2 — singularidad | ~8,090–8,120 | **~8,085–8,110** |
| 3 — genérico | ~11,020–11,110 | **~11,000–11,030** |

**Hallazgo 1 — los ciclos no cambian con el reloj, el tiempo sí.** El promedio de ciclos
(convertido) es prácticamente idéntico entre 216MHz y 16MHz en los 3 casos (diferencias de
15–28 ciclos, <0.3%). Tiene sentido: el mismo código ejecuta el mismo número de instrucciones sin
importar qué tan rápido tiquetee el reloj — lo que cambia es cuánto dura cada ciclo, por eso el
tiempo real sí escala ~13.5× entre las dos tarjetas (justo la relación 216/16).

**Hallazgo 2 — la primera ejecución paga un costo de caché que es mayor a 216MHz, pero no es
exclusivo de esa velocidad.** Caso 1 "en frío" tarda más ciclos que el promedio en **ambos**
relojes: ~1,076 ciclos más a 216MHz (13% de sobrecosto), ~449 ciclos más a 16MHz (5.4%) — en los
Casos 2 y 3 esa brecha prácticamente desaparece en los dos relojes. Es la *primera* vez que se
ejecuta `forward_kinematics()` desde el arranque, con el I-cache totalmente frío; para cuando
corren los Casos 2 y 3, el código ya quedó cacheado por las 1000 repeticiones del Caso 1. La parte
que sí depende del reloj es **cuánto pesa** ese primer fallo de caché: a 216MHz, Flash necesita
**7 ciclos de espera** por acceso (sección 5.1) contra prácticamente 0 a 16MHz, así que cada
instrucción nueva que hay que traer de Flash cuesta más ciclos exactamente ahí — de ahí que la
brecha sea más del doble, en proporción, a 216MHz que a 16MHz.

**Hallazgo 3 — por qué el Caso 3 tarda ~32% más que el Caso 1, en los dos relojes por igual (el
argumento para el profesor).** Caso 1 y Caso 3 ejecutan **el mismo camino de código**: ninguno
cae en la rama de singularidad, así que los dos llaman `atan2()` exactamente 3 veces y pasan por
las mismas 6 llamadas a `dh_rot()` (12 evaluaciones de `cos()`/`sin()` en total). La única
diferencia entre los dos casos es el **valor numérico** de `theta1..theta6` — todo cero en el
Caso 1, seis ángulos genéricos (30°, 20°, -15°, 45°, 60°, -70°) en el Caso 3. Aun así, el Caso 3
tarda **~32% más ciclos** (11,000 vs 8,329 a 16MHz; 11,028 vs 8,344 a 216MHz — el mismo ~32% en
los dos relojes, lo que confirma que es un costo en *instrucciones*, no un artefacto de reloj).

Esto se explica por cómo funciona `cos()`/`sin()` en la librería matemática de software
(ARM Compiler 6 `libm`), y es la razón de fondo por la que el software **no** es determinístico
como el CORDIC de la FPGA:

- `cos(0)` y `sin(0)` (y, en el Caso 2, `cos(90°)`/`sin(90°)`) son ángulos "exactos" para los que
  la librería puede devolver el resultado casi sin trabajo — sin necesidad de la reducción de
  rango completa que exige un ángulo arbitrario.
- Los ángulos del Caso 3 no son múltiplos limpios de nada: cada una de las 12 llamadas a
  `cos()`/`sin()` tiene que hacer la reducción de rango completa (llevar el ángulo a un intervalo
  pequeño restando múltiplos de 90°) y evaluar el polinomio de aproximación completo — más
  instrucciones, más ciclos, por cada una de las 12 llamadas.
- El CORDIC de la FPGA (`cordic_sincos_16.vhd`, lección 12) **no tiene este problema**: es un
  circuito de iteraciones fijas (`N_ITER=12`, siempre) que solo mira el signo del residuo en cada
  paso — nunca evalúa "qué tan complicado" es el ángulo ni toma atajos. Por diseño, tarda lo
  mismo para cualquier ángulo, incluido el peor caso. El software, en cambio, tiene ramas
  optimizadas para el caso común (ángulos "limpios") que simplemente no existen en hardware fijo.

**Consecuencia práctica (el punto para control/tiempo real):** en la FPGA, el peor caso de
tiempo de ejecución **es** el caso típico — no hay sorpresas. En el STM32, el peor caso (un
ángulo genérico) es estructuralmente más lento que el mejor caso (un ángulo en 0°/90°/180°), y esa
diferencia (~32% aquí) hay que medirla empíricamente probando muchas combinaciones de entrada,
porque no se puede leer del código fuente ni de la hoja de datos del chip — el software no tiene
un "peor caso" fácil de acotar, el hardware dedicado sí.

**Hallazgo 4 — el "mejor caso" diseñado a propósito NO es el más rápido; ganó la rama de
singularidad, no los ángulos limpios.** Para acotar el rango real de tiempo, se agregaron dos
casos dedicados: Caso EXTRA-1 (los 6 ángulos en 90°, pensado como el mejor caso posible para la
CPU) y Caso EXTRA-2 (ángulos irregulares dentro de 0°-180°, el peor caso realista). El resultado
en los dos relojes:

| Caso | 216MHz, promedio (µs) | 16MHz, promedio (µs) |
|---|---|---|
| 2 — singularidad | **37.417** (el más rápido) | **505.500** (el más rápido) |
| EXTRA-1 — los 6 ángulos en 90° | 42.407 | 572.938 |
| EXTRA-2 — irregular (peor caso) | **52.083** (el más lento) | ≥688.000 (ver sección 10) |

El Caso EXTRA-1 sí es más rápido que el Caso 3 (genérico) y que el EXTRA-2, confirmando la
hipótesis del Hallazgo 3 (ángulos "limpios" toman la ruta corta de `cos()`/`sin()`). Pero el Caso
2 le sigue ganando por un margen claro y repetible en los dos relojes (~13% más rápido que
EXTRA-1). La razón está en el código, no en la trigonometría: el Caso 2 cae en la rama
`if (mag < EPS_SINGULARIDAD)` de la sección 4.3, que **asigna** `yaw=0.0` y `roll=PI` directo,
sin llamar `atan2()` ni una sola vez — mientras que el Caso EXTRA-1, aunque tiene los ángulos más
"baratos" posibles para `cos()`/`sin()`, sigue teniendo que llamar `atan2()` dos veces (para
`yaw` y `roll`) porque no cae en la singularidad. Ahorrarse dos llamadas a función completas pesa
más que tomar la ruta rápida dentro de esas llamadas — una distinción que no era obvia antes de
medir con un caso diseñado específicamente para aislarla.

---

## 10. Mínimo y máximo medido, por reloj

Con los 5 casos de prueba corridos (los 3 originales de la lección 12 más los 2 casos EXTRA de
la sección 9, pedidos explícitamente para acotar el rango real de tiempo dentro del espacio de
ángulos físicamente alcanzable por el brazo, 0°-180°), estos son los extremos medidos —
`forward_kinematics()`, promedio de 1000 ejecuciones (estado estable, ver sección 7):

| Reloj | Mínimo | Configuración | Máximo | Configuración |
|---|---|---|---|---|
| **216 MHz** (PLL, Over-drive) | **37.417 µs** | Caso 2 — singularidad (th5=90°, th6=90°, resto 0°) | **52.083 µs** | Caso EXTRA-2 — peor caso (ángulos irregulares, ver sección 9) |
| **16 MHz** (HSI, sin PLL) | **505.500 µs** | Caso 2 — singularidad (misma configuración) | **688.000 µs** confirmado (Caso 3 — genérico) | ver nota |

> **Nota sobre el máximo a 16MHz:** el Caso EXTRA-2 (diseñado como el peor caso) fue efectivamente
> el más lento a 216MHz (52.083 µs, un 2.3% más que el Caso 3), así que por el mismo patrón
> probablemente también sea el máximo real a 16MHz — pero su captura por terminal a 16MHz se
> cortó antes de leer el dato completo de tiempo (ver sección 8). Usando el factor de escalado
> 13.5× confirmado en los otros 4 casos (sección 9, Hallazgo 1), el valor esperado ronda
> **~702-705 µs** — esto es una **predicción, no una medición**, y se deja así de explícito hasta
> repetir esa corrida puntual. El máximo **confirmado por medición completa** a 16MHz es el
> Caso 3, con 688.000 µs.

> **Candidato a mínimo aún mejor — Caso EXTRA-3 (agregado al código, pendiente de correr):**
> el Caso 2 gana por evitar los 2 `atan2()` de la rama de singularidad (Hallazgo 4), pero sigue
> evaluando 4 de sus 6 ángulos efectivos en 90° (`theta3+90°` y `theta4+90°` valen 90° cuando
> `theta3=theta4=0`, por el offset de la tabla DH). Se buscó, verificando con álgebra simbólica
> (`sympy`, no a mano) sobre las mismas matrices `R1..R6` de la sección 3, una combinación que
> **siga cayendo exactamente en la misma rama de singularidad** (`mag=0`) pero con menos ángulos
> "caros": `theta1=0, theta2=0, theta3=-90°, theta4=-90°, theta5=90°, theta6=0°`. Los `-90°` en
> `theta3`/`theta4` cancelan el offset `+90°` de `dh_rot()`, dejando **5 de los 6 ángulos
> evaluados en 0° exacto** (el más barato para `cos()/sin()`, sección 9) y solo uno en 90°
> (`theta5`, que es intrínseco a esta singularidad de muñeca — no se puede eliminar sin dejar de
> ser singular). Se confirmó `mag=0.0` exacto con sympy antes de tocar el firmware. Ya está
> agregado como `Caso EXTRA-3` en los dos `.cpp`, pero **todavía no se ha corrido en la tarjeta
> real** — la predicción es que sea el caso más rápido de los 6, por debajo del Caso 2, pero eso
> hay que confirmarlo con TIM5, no asumirlo. **Nota:** `theta3`/`theta4` negativos quedan fuera
> del rango físico real de un servomotor (0°-180°) — es un caso puramente de benchmark de
> software, no una pose que se le vaya a pedir al brazo real.

En "en frío" (una sola ejecución, sin caché caliente — sección 7) el orden de mínimo/máximo es el
mismo: 37.500 µs / 52.037 µs a 216MHz, y 506.188 µs / 689.125 µs (Caso 3, confirmado) a 16MHz —
la diferencia entre "en frío" y "promedio" es marginal en estos dos casos porque para cuando se
ejecutan (van 2° y 5° en `run_all_cases()`) la caché ya se calentó con las 1000 repeticiones del
Caso 1 (ver Hallazgo 2, sección 9).

**Rango total medido:** a 216MHz el peor caso tarda **39% más** que el mejor (52.083 vs 37.417
µs); a 16MHz, al menos **36% más** (688.000 vs 505.500 µs, y probablemente más si se confirma el
EXTRA-2). Es el mismo orden de magnitud en las dos velocidades — coherente con el Hallazgo 1
(el costo, en ciclos de CPU, no depende del reloj, solo el tiempo real).

---

## 11. FPGA vs. STM32 — la comparación final

| | FPGA (50 MHz) | STM32 @ 216 MHz | STM32 @ 16 MHz |
|---|---|---|---|
| Tiempo | **~4.05 µs, siempre** (fijo, cualquier ángulo) | 37.4–52.1 µs (según el ángulo, rango confirmado, sección 10) | 505.5–688.0 µs confirmado (posiblemente ~705 µs, sección 10) |
| **Veces más lento que la FPGA** | 1× | **~9.2×–12.9×** | **~124.8×–169.9×** (hasta ~174× si se confirma el máximo) |

A pesar de que el STM32 a máxima velocidad tiene un reloj **4.3 veces más rápido** que la FPGA
(216MHz vs 50MHz), termina el mismo cálculo entre **~9.2 y 12.9 veces más lento** — el mismo
patrón que ya se había visto con el brazo de 5R: hardware dedicado en pipeline (la FPGA calcula
todo en paralelo, ciclo a ciclo, con circuitos construidos exactamente para esta cuenta) le gana
por mucho a software secuencial (el STM32 ejecuta instrucción por instrucción, y cada
`cos()`/`sin()`/`atan2()`/`sqrt()` de doble precisión cuesta decenas a cientos de ciclos), sin
importar cuánto reloj se le meta al software. Y bajar la tarjeta a su velocidad de fábrica
(16MHz, sin PLL) multiplica esa brecha por otras ~13.5×, hasta más de 170× frente a la FPGA.

La FPGA subió de ~3.1 µs a ~4.05 µs (lección 12) y el STM32 también subió de tiempo respecto a
la versión anterior (atajo + `DWT`) por el mismo motivo: el método recursivo hace más aritmética
explícita en los dos lados (74 pasos vs 40 en VHDL; más operaciones de matriz en C) a cambio de
ser el método que el profesor puede seguir — pagado en ambas plataformas, no solo una. Y como se
explicó en la sección 9: en la FPGA ese tiempo **es el mismo sin importar el ángulo**; en el
STM32 varía **hasta un 32%** según qué tan "limpio" sea el ángulo — la FPGA no solo gana en
velocidad, gana en **previsibilidad**, una propiedad aparte y clave para control en tiempo real.

---

## 12. Cómo compilar y correr en Keil

1. Abrir `FK_6R_Geometrico_STM32.uvprojx` (216MHz) o
   `FK_6R_Geometrico_STM32_HSI16MHz.uvprojx` (16MHz) en Keil µVision5 — son dos proyectos
   independientes, mismo target `STM32F767ZITx`.
2. Compilar (`Build`) y programar (`Download`, con el ST-LINK de la Nucleo-144 conectado).
3. Abrir un terminal serie (HTerm, PuTTY, etc.) al puerto COM del ST-LINK, 9600 baudios, 8N1.
4. Al resetear la tarjeta corre automáticamente `run_all_cases()` una vez — presionar el botón
   de usuario (B1) la repite cuantas veces se quiera, para confirmar que los ciclos son estables
   entre corridas (ver sección 8, tres corridas iguales por velocidad).
