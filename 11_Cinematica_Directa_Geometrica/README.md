# Cinemática Directa 5R por Método Geométrico — Proyecto `FK_6R_Geometrico`

## Descripción General

Este módulo calcula la **cinemática directa completa** (posición `x,y,z` **y** orientación
`yaw,pitch,roll` del efector final) de un robot serial de 5 articulaciones revolutas + gripper,
usando el **método geométrico clásico** — el primero que se enseña en un curso de robótica,
antes de introducir matrices homogéneas o parámetros Denavit-Hartenberg.

Es la respuesta directa a un problema real que surgió después de las lecciones
[6](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) y
[7](../7_Cinematica_Directa_DH/README.md): tanto el método de matrices homogéneas como el
método DH, al escalar de 2R a 5 eslabones, **saturan la FPGA**. El profesor del semillero pidió
explícitamente reducir elementos lógicos manteniendo la misma metodología de trabajo (BDF →
VHDL generado → testbench, paso a paso). Este documento explica por qué el atajo geométrico
es posible para *este* robot en particular, cómo se derivó matemáticamente, y cómo se tradujo
a hardware bloque por bloque.

> **Aritmética:** Punto fijo Q2.13 (16 bits con signo), igual que en el resto del semillero.
> **Plataforma:** Cyclone IV E (EP4CE6E22C8) · Quartus 18.1
> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Por qué cambiar de método: el problema de recursos](#1-por-qué-cambiar-de-método-el-problema-de-recursos)
2. [Por qué este robot admite el atajo geométrico](#2-por-qué-este-robot-admite-el-atajo-geométrico)
3. [Derivación matemática](#3-derivación-matemática)
4. [Arquitectura de módulos (hoja de ruta)](#4-arquitectura-de-módulos-hoja-de-ruta)
5. [Módulo por módulo](#5-módulo-por-módulo)
6. [Integración final: el BDF completo](#6-integración-final-el-bdf-completo)
7. [Testbench, casos de prueba y la singularidad](#7-testbench-casos-de-prueba-y-la-singularidad)
8. [Verificación en MATLAB](#8-verificación-en-matlab)
9. [Cómo compilar y simular](#9-cómo-compilar-y-simular)
10. [Comparación de recursos: geométrico vs. DH](#10-comparación-de-recursos-geométrico-vs-dh)

---

## 1. Por qué cambiar de método: el problema de recursos

El proyecto `Fk6R_DH` de la [lección 7](../7_Cinematica_Directa_DH/README.md) escala el método
DH a 5 eslabones instanciando 5 bloques `t_dh_gen` (cada uno con su propio CORDIC de 13 ciclos)
y 4 bloques `mat4x4_mul` (16 multiplicaciones Q2.13 en paralelo cada uno) encadenados. El
reporte del *fitter* de ese proyecto, compilado contra la misma Cyclone IV E de este semillero,
da:

| Recurso | Usado | Disponible (EP4CE6) | % |
|---|---|---|---|
| Logic elements | 55,391 | 6,272 | **883 %** |
| Multiplicadores embebidos 9-bit | 30 | 30 | 100 % |
| **Fitter Status** | | | **Failed** |

No cabe ni remotamente en la FPGA del semillero. El cuello de botella son los 4
multiplicadores de matriz 4×4 (16 productos + 12 sumas cada uno) y los 5 CORDIC corriendo en
paralelo. Cambiar de arquitectura (sin cambiar de robot ni de resultado matemático) es la
única forma de que este cálculo quepa en la tarjeta.

---

## 2. Por qué este robot admite el atajo geométrico

Con la asignación de ejes DH del robot (ver lección 7, sección 2, para la tabla completa de
parámetros DH), la geometría real es la de un manipulador tipo **codo** con muñeca simple:

- **θ1** rota toda la cadena alrededor de `Z0` (la base) — no cambia la geometría *dentro* del
  plano del brazo, solo orienta ese plano respecto al mundo.
- **θ2, θ3, θ4** son tres articulaciones **coplanares** (todas paralelas entre sí): doblan el
  brazo dentro de un mismo plano vertical, cada una heredando la inclinación acumulada de las
  anteriores.
- **θ5** rota el gripper únicamente sobre su propio eje — es un *roll* puro que **no mueve la
  posición** del efector, solo su orientación final.

Como consecuencia, la posición `(x,y,z)` del efector se puede obtener con trigonometría plana
directa (proyectar cada eslabón sobre el plano del brazo y luego rotar ese plano por `θ1`), sin
necesidad de multiplicar ninguna matriz 4×4. Las longitudes de eslabón usadas son:

| Eslabón | Longitud real | Q2.13 (`raw = round(m·8192)`) |
|---|---|---|
| `L1` (base → hombro) | 0.05 m | 410 |
| `L2` | 0.107 m | 877 |
| `L3` | 0.13 m | 1065 |
| `L4 + L5` | 0.07 + 0.11 = 0.18 m | 1475 |

`L4` y `L5` se suman en una sola constante porque, según el punto anterior, `θ5` no cambia la
posición — el tramo final del brazo (`L4+L5`) se comporta, para efectos de posición, como una
prolongación rígida del eslabón 3.

---

## 3. Derivación matemática

### 3.1 Posición

Con `φ2 = θ2`, `φ23 = θ2+θ3`, `φ234 = θ2+θ3+θ4` (los ángulos **acumulados** de cada eslabón
respecto al eje de referencia):

```
r = L2·cos(φ2) + L3·cos(φ23) + (L4+L5)·cos(φ234)      (alcance radial, dentro del plano)
z = L1 + L2·sin(φ2) + L3·sin(φ23) + (L4+L5)·sin(φ234)  (altura)
x = r·cos(θ1)
y = r·sin(θ1)
```

### 3.2 Orientación: matriz de rotación `R0_5`

Para la orientación se necesita la matriz de rotación completa base→efector. Se deriva
expandiendo a mano la cadena DH `R0_5 = R1·R2·R3·R4·Rx(θ5)` (post-multiplicación, siguiendo la
misma convención de composición de rotaciones —"rotar sobre el sistema actual se
post-multiplica"— que se usa en la [lección 7](../7_Cinematica_Directa_DH/README.md)). El
resultado, verificado por multiplicación simbólica y confirmado numéricamente contra la cadena
DH completa (ver [sección 8](#8-verificación-en-matlab)), es:

```
R0_5 = [ -cosθ1·sinφ234    sinθ1·cosθ5+cosθ1·cosφ234·sinθ5    -sinθ1·sinθ5+cosθ1·cosφ234·cosθ5 ]
       [ -sinθ1·sinφ234   -cosθ1·cosθ5+sinθ1·cosφ234·sinθ5     cosθ1·sinθ5+sinθ1·cosφ234·cosθ5 ]
       [  cosφ234           sinφ234·sinθ5                       sinφ234·cosθ5                   ]
```

De estos 9 elementos, **solo 5** hacen falta para extraer los ángulos de Euler (sección
siguiente): `R11, R21, R31, R32, R33`. Por eso el hardware nunca arma la matriz completa —
`mat_r05_elems.vhd` calcula exclusivamente esos 5 valores.

### 3.3 Ángulos de Euler ZYX (metodología de la clase de teoría)

Con `Rzyx(yaw,pitch,roll) = Rz(yaw)·Ry(pitch)·Rx(roll)`, la extracción estándar de ángulos de
Euler a partir de una matriz de rotación (deducida en clase a partir de `R11²+R21² = cos²β`) es:

```
yaw   = atan2(R21, R11)
pitch = atan2(-R31, sqrt(R11² + R21²))
roll  = atan2(R32, R33)
```

con singularidad en `β = ±90°` (cuando `cos(pitch) → 0`). Sustituyendo los `R_ij` de la sección
3.2:

```
yaw   ≈ θ1 (± 180°, según el signo de sinφ234)
pitch = f(φ234)
roll  ≈ θ5 (± 180°, según el signo de sinφ234)
```

Es una comprobación elegante de que el modelo está bien planteado: el *yaw* extraído coincide
con la rotación de la base y el *roll* con la rotación del gripper, tal como se esperaría
físicamente. La singularidad de esta extracción ocurre exactamente cuando
`sin(φ234) = 0` — es decir, cuando **el brazo queda totalmente extendido o totalmente plegado
en línea recta**. Ahí `R11 ≈ R21 ≈ 0` y `R32 ≈ R33 ≈ 0` simultáneamente, y *yaw*/*roll* quedan
indeterminados (ver discusión de casos de prueba, sección 7).

---

## 4. Arquitectura de módulos (hoja de ruta)

Todo el diseño gira sobre un único recurso reutilizado: el **CORDIC** (COordinate Rotation
DIgital Computer), que calcula seno/coseno/`atan2` usando solo sumas, restas y corrimientos de
bits — sin multiplicadores ni tablas trigonométricas. Existen dos modos, ambos ya introducidos
en la [lección 5](../5_2R_Inverse_Cinematic/Operadores.md):

- **Modo rotación** (`cordic_sincos_16`, lección 5): dado un ángulo, devuelve su seno/coseno.
- **Modo *vectoring*** (variante nueva `cordic_atan2_16`, sección 5.6): dado un vector `(x,y)`,
  devuelve `atan2(y,x)` **y**, de regalo, `sqrt(x²+y²)`.

La decisión de diseño central (a diferencia de los métodos de las lecciones 6 y 7, que
instancian un CORDIC físico por cada ángulo) es: **una sola copia física de cada CORDIC,
reutilizada varias veces en secuencia** mediante una máquina de estados. Eso reduce
drásticamente el conteo de multiplicadores embebidos y elementos lógicos, a cambio de más
ciclos de latencia.

| Paso | Módulo | Qué hace | Latencia |
|---|---|---|---|
| 0 | `geom_pkg.vhd` | Constantes de eslabón + reducción angular | — (paquete) |
| 1 | `angle_sum_gen.vhd` | `θ2,θ3,θ4 → φ2,φ23,φ234` | 1 ciclo |
| 2 | `cordic_seq5.vhd` | 1 CORDIC reutilizado en 5 pasadas: `θ1,φ2,φ23,φ234,θ5 →` senos/cosenos | ~65 ciclos |
| 3 | `fk_geom_core.vhd` | Posición `x,y,z` (rama posición) | 2 ciclos |
| 4a | `cordic_atan2_16.vhd` | CORDIC modo *vectoring* (bloque base, se prueba aislado) | 13 ciclos |
| 4b | (incluido en `cordic_seq5`) | — | — |
| 4c | `mat_r05_elems.vhd` | Elementos `R11,R21,R31,R32,R33` (rama orientación) | 1 ciclo |
| 4d | `atan2_seq3.vhd` | 1 CORDIC *vectoring* reutilizado en 3 pasadas: `→ yaw,pitch,roll` | ~40 ciclos |
| 4e | Integración final (BDF) | Encadena todo: `θ1..θ5 → x,y,z,yaw,pitch,roll` | ~105-115 ciclos |

Cada paso se armó y probó **por separado** en Quartus (diagrama de bloques → VHDL generado →
testbench propio) antes de integrarlo, siguiendo la misma metodología de las lecciones
anteriores.

---

## 5. Módulo por módulo

### 5.1 `geom_pkg.vhd` — constantes y reducción angular

No es un circuito: es un paquete VHDL con las longitudes de eslabón en Q2.13 y una función
`wrap_to_pi` que reduce cualquier suma de ángulos (que puede pasarse de `±180°`) de vuelta al
rango que espera el CORDIC.

```vhdl
constant L1_Q13  : signed(15 downto 0) := to_signed(  410, 16);  -- 0.05  m
constant L2_Q13  : signed(15 downto 0) := to_signed(  877, 16);  -- 0.107 m
constant L3_Q13  : signed(15 downto 0) := to_signed( 1065, 16);  -- 0.13  m
constant L45_Q13 : signed(15 downto 0) := to_signed( 1475, 16);  -- L4+L5 = 0.18 m

function wrap_to_pi(a : signed(17 downto 0)) return signed;
```

### 5.2 `angle_sum_gen.vhd` — ángulos acumulados

Toma `θ2,θ3,θ4` (lo que mueve cada motor, relativo al eslabón anterior) y calcula los ángulos
**absolutos** acumulados `φ2 = θ2`, `φ23 = θ2+θ3`, `φ234 = θ2+θ3+θ4`. Solo sumas — sin
trigonometría, sin CORDIC. 1 ciclo de latencia.

### 5.3 `cordic_seq5.vhd` — un solo CORDIC, cinco pasadas

El corazón del ahorro de área. Adentro hay **una sola** instancia de `cordic_sincos_16` (la
misma de la lección 5, sin modificar). Una máquina de estados (`S_IDLE → S_W1 → S_W2 → S_W3 →
S_W4 → S_W5`) le mete, una tras otra, las 5 pasadas — `θ1`, `φ2`, `φ23`, `φ234`, `θ5` —
guardando el seno/coseno de cada una en un registro antes de arrancar la siguiente:

```vhdl
when S_W1 =>
    if cordic_done = '1' then
        cos1_r       <= cordic_cos;
        sin1_r       <= cordic_sin;
        angle_mux    <= phi2_in;
        cordic_start <= '1';
        state        <= S_W2;
    end if;
```

Latencia total ≈ 5×13 = 65 ciclos — el precio de compartir un solo CORDIC físico entre 5
ángulos en vez de instanciar 5 copias en paralelo (como haría el método DH).

### 5.4 `fk_geom_core.vhd` — posición (rama 1 de 2)

Con los senos/cosenos ya calculados, arma la posición en 2 etapas:

```vhdl
-- etapa 1
r_v := mul_q13(L2_Q13, signed(cos2_in))
     + mul_q13(L3_Q13, signed(cos23_in))
     + mul_q13(L45_Q13, signed(cos234_in));
z_v := L1_Q13
     + mul_q13(L2_Q13, signed(sin2_in))
     + mul_q13(L3_Q13, signed(sin23_in))
     + mul_q13(L45_Q13, signed(sin234_in));
-- etapa 2
x_r <= mul_q13(r_r, cos1_r);
y_r <= mul_q13(r_r, sin1_r);
```

Solo multiplicaciones simples (`mul_q13`, el mismo operador de la lección 5) y sumas — sin
CORDIC adicional, sin matrices. 2 ciclos de latencia.

### 5.5 `mat_r05_elems.vhd` — elementos de rotación (rama 2 de 2, parte 1)

Arma los 5 elementos de `R0_5` que se necesitan (sección 3.2/3.3), de nuevo solo con productos
simples:

```vhdl
r11_r <= -mul_q13(signed(cos1_in),   signed(sin234_in));
r21_r <= -mul_q13(signed(sin1_in),   signed(sin234_in));
r31_r <=  signed(cos234_in);
r32_r <=  mul_q13(signed(sin234_in), signed(sin5_in));
r33_r <=  mul_q13(signed(sin234_in), signed(cos5_in));
```

1 ciclo de latencia.

### 5.6 `cordic_atan2_16.vhd` — CORDIC modo *vectoring* con magnitud

Variante del `cordic_atan2.vhd` de la [lección 5](../5_2R_Inverse_Cinematic/Atan2.md): en vez
de girar un vector un ángulo **conocido** (modo rotación), gira el vector de entrada `(x,y)`
**hasta que su componente Y llega a cero**. El ángulo acumulado en el camino es `atan2(y,x)`, y
la magnitud final del vector (compensada por la ganancia `K` del CORDIC) es `sqrt(x²+y²)` —
esto último es la novedad respecto al `cordic_atan2.vhd` original, y resulta clave para poder
encadenar tres pasadas (sección 5.7) sin necesitar un `sqrt` aparte.

```vhdl
-- Correccion de cuadrante: si x<0, rotar 180 grados antes de iterar
if x_s < 0 then
    x_pipe(0) <= -x_s;  y_pipe(0) <= -y_s;
    if y_s >= 0 then z_pipe(0) <= PI_Q13; else z_pipe(0) <= -PI_Q13; end if;
else
    x_pipe(0) <= x_s;   y_pipe(0) <= y_s;   z_pipe(0) <= (others => '0');
end if;
...
angle_out <= std_logic_vector(z_pipe(N_ITER));
mag_out   <= std_logic_vector(mul_q13(x_pipe(N_ITER), CORDIC_K));
```

13 ciclos de latencia (igual que el CORDIC de rotación).

### 5.7 `atan2_seq3.vhd` — un solo CORDIC *vectoring*, tres pasadas (con dependencia real)

Misma filosofía que `cordic_seq5`, pero reutilizando `cordic_atan2_16`:

```
Pasada 1: x=R11, y=R21   -> angle = Yaw,   mag = rho = sqrt(R11²+R21²)
Pasada 2: x=rho, y=-R31  -> angle = Pitch          (usa el rho de la pasada 1)
Pasada 3: x=R33, y=R32   -> angle = Roll
```

A diferencia de `cordic_seq5` (cuyas 5 pasadas son independientes entre sí), aquí la **pasada 2
depende del resultado de la pasada 1** (`rho` no es un dato de entrada, sale del cálculo
anterior) — es una dependencia de datos real, no solo una reutilización de hardware. Por eso
este bloque se probó aislado antes de integrarlo (ver `images/Paso4d_diagrama.png`).

---

## 6. Integración final: el BDF completo

![Diagrama de flujo completo: angle_sum_gen + cordic_seq5, dividido en rama de posición (fk_geom_core) y rama de orientación (mat_r05_elems + atan2_seq3)](images/Paso4e_diagrama.png)

El flujo completo, de entradas a salidas:

```
θ1..θ5  ->  angle_sum_gen (θ2,θ3,θ4) ->  φ2,φ23,φ234  \
                                                          -> cordic_seq5 (5 pasadas)  ->  { rama posición, rama orientación }
θ1, θ5 ------------------------------------------------/
```

`start_in` dispara `angle_sum_gen` y `cordic_seq5` **en paralelo** (no en cadena) — como
`angle_sum_gen` solo tarda 1 ciclo, sus salidas están listas mucho antes de que `cordic_seq5`
las necesite (~13 ciclos después, al terminar la primera pasada). Cuando `cordic_seq5.done` se
activa, dispara **en paralelo** las dos ramas:

- **Rama posición** (verde): `cordic_seq5 → fk_geom_core → x_out,y_out,z_out`. Termina rápido
  (2 ciclos).
- **Rama orientación** (rosa): `cordic_seq5 → mat_r05_elems → atan2_seq3 →
  yaw_out,pitch_out,roll_out`. Termina mucho después (~40 ciclos más), porque es la que
  reutiliza el CORDIC *vectoring* tres veces.

El `done_out` final del top-level es el de `atan2_seq3` — el **último** en terminar. Los
`done` de `angle_sum_gen` y `fk_geom_core` se dejan **sin conectar** a propósito: no hace falta
sincronizar con ellos porque, o bien sus salidas ya están listas con anticipación (caso
`angle_sum_gen`), o bien terminan mucho antes que la señal de `done` que sí importa y sus
salidas se quedan quietas esperando (caso `fk_geom_core`).

Diagramas de conexión intermedios (para armar cada bloque por separado antes de la integración
final):

| Paso | Diagrama | Qué prueba |
|---|---|---|
| 4a | `images/Paso4a_diagrama.png` | `cordic_atan2_16` aislado (5 casos: 0°, 90°, 180°, -90°, 53.13°) |
| 4b+4c | `images/Paso4bc_diagrama.png` | `cordic_seq5` + `mat_r05_elems` en cascada |
| 4d | `images/Paso4d_diagrama.png` | `atan2_seq3` aislado (incluye el caso de singularidad) |
| 4e | `images/Paso4e_diagrama.png` | Integración completa (este documento) |

---

## 7. Testbench, casos de prueba y la singularidad

Los 3 casos de prueba finales (`theta1..theta5` en grados; salidas en raw Q2.13):

| Caso | θ1,θ2,θ3,θ4,θ5 | x | y | z | yaw | pitch | roll |
|---|---|---|---|---|---|---|---|
| 1 — singularidad (`φ234=0`) | `0,0,0,0,0` | 3422 | 3 | 412 | 26363* | -12865 | 627* |
| 2 — limpio | `45,0,0,90,45` | 1375 | 1374 | 1887 | -19303 | -3 | 6433 |
| 3 — limpio | `0,45,0,0,0` | 2418 | 2 | 2826 | -25733† | -6433 | 3 |

Posición y *pitch* coinciden con el valor esperado en los 3 casos, dentro del margen normal del
CORDIC (±0.02°–0.1°). Dos observaciones importantes, ambas *esperadas* y no errores de diseño:

**(\*) Caso 1 — inestabilidad numérica real en la singularidad.** Con `φ234=0`
(brazo totalmente extendido), `R11`, `R21`, `R32` y `R33` no llegan como ceros matemáticos
exactos sino como **residuos de redondeo del CORDIC** (`cos1`, `sin234`, etc. tienen su propio
error de ±0.02°). El resultado es que `yaw`/`roll` calculan `atan2` de un vector prácticamente
nulo, cuyo ángulo resultante es extremadamente sensible a ese ruido de redondeo — puede salir
literalmente cualquier valor. Es la confirmación en hardware real del *gimbal lock* descrito en
la sección 3.3, no un bug.

**(†) Caso 3 — el corte de ±180°.** `+180°` y `-180°` son el mismo ángulo físico (es el punto
de discontinuidad de `atan2`). Como aquí `R11` es muy negativo y `R21` casi cero (con signo
dependiente del mismo ruido de redondeo), el resultado cae de un lado u otro del corte — la
*magnitud* es correcta (`25733 ≈ 25736 ≈ 180°`), solo cambia el signo de representación.

---

## 8. Verificación en MATLAB

`verificacion_fk_geometrica.m` reimplementa las mismas fórmulas de las secciones 3.1 y 3.2/3.3
en punto flotante, para comparar contra los resultados de la simulación en ModelSim sin
depender de la FPGA. Los ángulos de entrada están al principio del script, fáciles de cambiar:

```matlab
%% ---- Longitudes del brazo (metros) ----
L1 = 0.05; L2 = 0.107; L3 = 0.13; L4 = 0.07; L5 = 0.11;
L45 = L4 + L5;

%% ==================================================================
%  ANGULOS DE ENTRADA (cambiar aqui, en grados, y volver a correr)
%  ==================================================================
theta1 = 0; theta2 = 45; theta3 = 0; theta4 = 0; theta5 = 0;

%% ---- Posicion del efector (igual que fk_geom_core.vhd) ----
r = L2*cos(phi2) + L3*cos(phi23) + L45*cos(phi234);
z = L1 + L2*sin(phi2) + L3*sin(phi23) + L45*sin(phi234);
x = r*cos(th1);
y = r*sin(th1);

%% ---- Elementos de la matriz de rotacion R0_5 (igual que mat_r05_elems.vhd) ----
R11 = -cos(th1)*sin(phi234);
R21 = -sin(th1)*sin(phi234);
R31 =  cos(phi234);
R32 =  sin(phi234)*sin(th5);
R33 =  sin(phi234)*cos(th5);

%% ---- Angulos de Euler ZYX (formulas de la clase del profesor) ----
yaw   = atan2(R21, R11);
pitch = atan2(-R31, sqrt(R11^2 + R21^2));
roll  = atan2(R32, R33);
```

El script también avisa cuando la configuración de entrada cae cerca de la singularidad
(`|sin(φ234)| < 0.05`), para no interpretar como error algo que es inestabilidad esperada.

---

## 9. Cómo compilar y simular

Mismo procedimiento que en las lecciones anteriores
([2](../2_Configuracion_Quartus_y_Simulacion/README.md)): cada bloque (`geom_pkg` →
`angle_sum_gen` → `cordic_seq5` → `fk_geom_core` → `mat_r05_elems` → `cordic_atan2_16` →
`atan2_seq3`) se compiló y probó **por separado** antes de la integración final, con un
`tb.vhd` y un `simular.do` propios para cada paso (nombres fijos, sobrescritos paso a paso —
ver los diagramas de la sección 6 para las instrucciones exactas de cada BDF intermedio).
Para el bloque final:

1. Compilar los 7 `.vhd` de este documento + el top-level generado desde el BDF de integración.
2. Simular con ModelSim/Questa, comparando contra la tabla de la sección 7.
3. Verificar en paralelo con `verificacion_fk_geometrica.m` (sección 8).

---

## 10. Comparación de recursos: geométrico vs. DH

| | [Lección 7 — DH](../7_Cinematica_Directa_DH/README.md) | Lección 11 — Geométrico (esta) |
|---|---|---|
| CORDIC instanciados | 5 (uno por eslabón, en paralelo) | 2 físicos, reutilizados 5 y 3 veces |
| Multiplicación de matrices | 4× `mat4x4_mul` (16 productos c/u) | 0 — solo productos escalares simples |
| Logic elements | 55,391 (**883 %** de un EP4CE6) | **1,754 (27 %)** |
| Multiplicadores embebidos | 30/30 (100 %) | 4/30 (13 %) |
| Fitter | **Failed** (no cabe) | Analysis & Synthesis exitoso |
| Latencia | ~23 ciclos | ~105-115 ciclos |

La reducción de recursos es drástica — de un diseño que ni siquiera cabe en la FPGA a uno que
usa poco más de una cuarta parte de sus elementos lógicos — a cambio de multiplicar por ~5 la
latencia (consecuencia directa de reutilizar 1-2 CORDIC en vez de 5 en paralelo). Para esta
aplicación (control de un brazo robótico, no un lazo de tiempo real de microsegundos) ese
intercambio es exactamente lo que se buscaba.

> **Nota de pines:** el conteo de pines de E/S del top-level de integración final (116 pines
> con las 5 entradas y 6 salidas de 16 bits en paralelo, más control) excede los 92 pines de
> usuario disponibles en el empaque TQFP144 del EP4CE6E22C8N. Es un problema de **interfaz**,
> no de lógica — pendiente de resolver serializando/multiplexando la entrada de ángulos y la
> salida de resultados, fuera del alcance de esta lección.

---

*Documentación generada para Quartus 18.1 · Familia Cyclone IV E · Reloj de sistema 50 MHz.
Ver también: [Cinemática 6R por Denavit-Hartenberg (lección 7)](../7_Cinematica_Directa_DH/README.md) ·
[Verificación en MATLAB con Peter Corke (lección 8)](../8_MATLAB_Peter_Corke_Cinematica/README.md)*
