# Cinemática en MATLAB — Robotics Toolbox de Peter Corke y verificación DH

## Descripción General

Antes de implementar cualquiera de los dos métodos de cinemática directa en VHDL
([matrices homogéneas](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) y
[Denavit-Hartenberg](../7_Cinematica_Directa_DH/README.md)), el semillero valida la
geometría del robot y los resultados numéricos esperados en MATLAB. Esta carpeta reúne
5 scripts con dos propósitos distintos:

1. **Visualizar y simular** el robot con el *Robotics Toolbox* de Peter Corke
   (`Metodo_DH_Peter.m`) — responde a "cómo se ve el robot simulado".
2. **Calcular a mano** (sin toolbox) los mismos resultados que debe producir el hardware,
   para tener valores de referencia con los que comparar la salida de las simulaciones
   VHDL (`FK_2R_MTH.m`, `Verificador_DH.m`, `Prueba_Generalidad_DH.m`, `Multiplicacion_Matrices.m`).

> **Universidad Militar Nueva Granada**

---

## Tabla de Contenidos

1. [Instalación del Robotics Toolbox](#1-instalación-del-robotics-toolbox)
2. [`Metodo_DH_Peter.m` — simulación y visualización con el toolbox](#2-metodo_dh_peterm--simulación-y-visualización-con-el-toolbox)
3. [`FK_2R_MTH.m` — cinemática 2R manual (equivalente MATLAB de la lección 6)](#3-fk_2r_mthm--cinemática-2r-manual-equivalente-matlab-de-la-lección-6)
4. [`Verificador_DH.m` y `Prueba_Generalidad_DH.m` — vectores de prueba para el testbench VHDL](#4-verificador_dhm-y-prueba_generalidad_dhm--vectores-de-prueba-para-el-testbench-vhdl)
5. [`Multiplicacion_Matrices.m` — utilidad didáctica](#5-multiplicacion_matricesm--utilidad-didáctica)
6. [Flujo de trabajo recomendado: de MATLAB a VHDL](#6-flujo-de-trabajo-recomendado-de-matlab-a-vhdl)

---

## 1. Instalación del Robotics Toolbox

`Metodo_DH_Peter.m` depende del **Robotics Toolbox for MATLAB** de Peter Corke
(clases `Link`, `SerialLink`, métodos `.plot()`, `.teach()`, `.fkine()`). Este toolbox
**no se versiona en el repositorio** (es software de terceros, no código propio del
semillero) — se instala aparte:

1. Descargar desde el sitio oficial: https://petercorke.com/toolboxes/robotics-toolbox/
2. En MATLAB: `Home → Add-Ons → Install from file...` y seleccionar el `.mltbx` descargado
   (o usar `Home → Add-Ons → Get Add-Ons` y buscar "Robotics Toolbox" si está disponible
   en el explorador de Add-Ons).
3. Verificar la instalación ejecutando `help SerialLink` en la consola de MATLAB.

---

## 2. `Metodo_DH_Peter.m` — simulación y visualización con el toolbox

Construye un robot de 5 eslabones revolutos con la clase `Link` del toolbox, usando
**los mismos parámetros DH** que el testbench `fk6r_tb.vhd` de la
[lección 7](../7_Cinematica_Directa_DH/README.md#2-parámetros-dh-reales-del-robot-5-eslabones):

```matlab
L1 = 10; L2 = 12; L3 = 10; L4 = 8; L5 = 6;

R(1) = Link('revolute', 'd', L1,  'alpha',  pi/2, 'a', 0,  'offset', 0);
R(2) = Link('revolute', 'd', 0,   'alpha',  0,    'a', L2, 'offset', 0);
R(3) = Link('revolute', 'd', 0,   'alpha',  0,    'a', L3, 'offset', 0);
R(4) = Link('revolute', 'd', 0,   'alpha',  pi/2, 'a', 0,  'offset', pi/2);
R(5) = Link('revolute', 'd', L5,  'alpha',  0,    'a', 0,  'offset', 0);

Robot = SerialLink(R, 'name', 'MiBrazo5DOF');
```

> Nota de unidades: aquí las longitudes están en centímetros (`L1=10`, etc.) solo para que
> la visualización con `.plot()` se vea a una escala razonable en la ventana de MATLAB — es
> un cambio de escala puramente gráfico. Los valores en metros que realmente se cargan al
> testbench VHDL (`0.10 m`, `0.12 m`, ...) están en `Verificador_DH.m` (sección 4).

### Cada `Link` es un eslabón DH

| Parámetro de `Link` | Corresponde a (notación lección 7) |
|---|---|
| `'d'` | `d` — desplazamiento fijo en Z |
| `'alpha'` | `α` — torsión del eslabón (mismo valor que carga `cx_in`/`sx_in` en `t_dh_gen`) |
| `'a'` | `a` — longitud del eslabón (mismo valor que `lx_in`) |
| `'offset'` | desplazamiento angular sumado a `θ` (usado en el eslabón 4, `offset=pi/2`) |

El `offset` del eslabón 4 (`pi/2`) es importante: no aparece como parámetro explícito en
`t_dh_gen.vhd` (que solo recibe `theta_in` ya calculado) — si se usa este mismo offset, hay
que sumarlo **antes** de convertir el ángulo a Q2.13 y pasarlo como `theta_in4` al testbench.

### Cómo se ve el robot simulado

```matlab
Robot.plot([q1, q2, q3, q4, q5], 'scale', 0.5, 'workspace', [-40 40 -40 40 -40 40]);
zlim([-10, 40]);
Robot.teach([q1, q2, q3, q4, q5], 'rpy/zyx');
```

- **`Robot.plot(...)`** dibuja el robot en una figura 3D en la pose de ángulos `[q1..q5]`
  dados (todos en `0` por defecto = posición *home*).
- **`Robot.teach(...)`** abre una ventana interactiva con **sliders**, uno por articulación,
  que permite mover el robot en tiempo real y ver cómo cambia la pose del efector — es la
  forma más directa de "ver el robot simulado" y entender visualmente qué hace cada ángulo
  antes de mandarlo al hardware.

### Cinemática directa con el toolbox

```matlab
MTH = Robot.fkine([q1, q2, q3, q4, q5])
```

`fkine()` hace exactamente lo que hace todo el circuito de la
[lección 7](../7_Cinematica_Directa_DH/README.md) en hardware: multiplica las 5 matrices DH
en cadena (`T05 = T01·T12·T23·T34·T45`) y devuelve la pose homogénea final. Es la forma más
rápida de obtener un valor de referencia para comparar contra la salida de `fk6r_tb.vhd`.

### Matrices DH paso a paso, sin `fkine()`

El mismo script incluye una segunda parte (tras el bloque de comentario
`MATRICES DH PASO A PASO`) que arma manualmente cada matriz DH multiplicando por separado
`Rz(θ)`, `Tz(d)`, `Rx(α)`, `Tx(a)`:

```matlab
RZ0 = [cos(q1) -sin(q1) 0 0;  sin(q1)  cos(q1) 0 0;  0 0 1 0;  0 0 0 1];
TZ0 = [1 0 0 0; 0 1 0 0; 0 0 1 L1; 0 0 0 1];
RX1 = [1 0 0 0; 0 cos(pi/2) -sin(pi/2) 0; 0 sin(pi/2) cos(pi/2) 0; 0 0 0 1];
TX1 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];

T01 = RZ0 * TZ0 * RX1 * TX1
```

Esto es pedagógicamente valioso porque descompone la matriz DH compacta de la sección 1 de
la lección 7 en sus **4 transformaciones elementales** (`Rz`, `Tz`, `Rx`, `Tx`), la misma
descomposición que subyace a la fórmula que implementa `t_dh_gen.vhd` en hardware, solo que
ahí ya está simplificada algebraicamente a una única matriz cerrada.

---

## 3. `FK_2R_MTH.m` — cinemática 2R manual (equivalente MATLAB de la lección 6)

Script corto (27 líneas) que calcula la cinemática directa del robot 2R **sin** el toolbox,
construyendo cada matriz homogénea directamente — el equivalente MATLAB exacto de lo que
hace `t_matrix_gen` + `fk2r_pipeline_core` en la
[lección 6](../6_Cinematica_Directa_Matrices_Homogeneas/README.md):

```matlab
h1 = 0;  l1 = 0.40;  l2 = 0.35;  theta1 = pi/6;  theta2 = pi/4;

T01 = [cos(theta1) -sin(theta1) 0 0;
       sin(theta1)  cos(theta1) 0 0;
       0            0           1 h1;
       0            0           0 1];

T12 = [cos(theta2) -sin(theta2) 0 l1;
       sin(theta2)  cos(theta2) 0 0;
       0            0           1 0;
       0            0           0 1];

T23 = [1 0 0 l2;
       0 1 0 0;
       0 0 1 0;
       0 0 0 1];

T03 = T01*T12*T23
```

Nótese que `T01`, `T12` y `T23` aquí tienen exactamente la misma forma que la matriz que
genera `t_matrix_gen(θ, tx, tz)` en la lección 6 — de hecho `T23` es traslación pura
(rotación identidad, solo `tx=l2`), igual que `T3_2` en el pipeline VHDL. Para usar este
script como referencia numérica de un caso de prueba del testbench `fk2r_tb.vhd`, basta con
convertir `theta1`, `theta2`, `l1`, `l2` a Q2.13 (`valor × 8192`) y comparar `T03(1,4)`,
`T03(2,4)` contra `px_out`/`py_out`.

---

## 4. `Verificador_DH.m` y `Prueba_Generalidad_DH.m` — vectores de prueba para el testbench VHDL

### `Verificador_DH.m`

Este es el script que genera **directamente** los valores usados en
`fk6r_tb.vhd` ([lección 7, sección 2](../7_Cinematica_Directa_DH/README.md#2-parámetros-dh-reales-del-robot-5-eslabones)).
Define una función anónima `dh(tz, tx, lz, lx)` que reproduce exactamente la fórmula DH:

```matlab
dh = @(tz, tx, lz, lx) [...
    cos(tz), -cos(tx)*sin(tz),  sin(tx)*sin(tz),  lx*cos(tz);
    sin(tz),  cos(tx)*cos(tz), -sin(tx)*cos(tz),  lx*sin(tz);
    0,        sin(tx),           cos(tx),           lz;
    0,        0,                 0,                 1];
```

Esta es **la misma fórmula, celda por celda**, que implementa `t_dh_gen.vhd` en hardware
(comparar con la sección 3 de la lección 7) — es la prueba de que el hardware calcula
exactamente lo que dice la teoría DH.

Corre 5 casos de prueba (los mismos ángulos que `fk6r_tb.vhd`, sección 6 de la lección 7) y,
para cada uno, imprime la posición del efector **en Q2.13**, listo para comparar contra la
simulación VHDL:

```matlab
T05_q = round(T05 * 8192);
fprintf('  Fila 0: [%6d  %6d  %6d  %6d]\n', T05_q(1,:));
```

También reporta la latencia esperada del pipeline como comentario informativo:

```
NOTA: tolerancia CORDIC aprox +-10 en Q2.13
PIPELINE: done_out llega al ciclo 23
  (15 ciclos t_dh + 4x2 ciclos mul = 23)
  Tiempo total a 50MHz: 460 ns
```

Estos números de latencia (15 + 4×2 = 23 ciclos) coinciden exactamente con el análisis de
[la lección 7, sección 8](../7_Cinematica_Directa_DH/README.md#8-comparación-dh-vs-matrices-homogéneas).

### `Prueba_Generalidad_DH.m`

Variante exploratoria del mismo cálculo: arma las 5 matrices DH del robot con la fórmula
expandida en línea (en vez de la función anónima `dh(...)`) y deja comentado un primer
intento con la descomposición en 4 matrices elementales (`RZ0`, `TZ0`, `RX1`, `TX1`, igual
que en `Metodo_DH_Peter.m`, sección 2) — es el "borrador" donde se probó la fórmula DH antes
de consolidarla en `Verificador_DH.m`. Útil como referencia de cómo se llegó a la fórmula
final, más que como fuente de vectores de prueba.

---

## 5. `Multiplicacion_Matrices.m` — utilidad didáctica

Script simple e interactivo (34 líneas): pide por consola los 16 valores de dos matrices
4×4 (`A` y `B`) y muestra el producto `C = A * B`:

```matlab
for i = 1:4
    for j = 1:4
        fprintf('A(%d,%d): ', i, j);
        A(i,j) = input('');
    end
end
```

No está atado a cinemática de ningún robot en particular — sirve como ejercicio de
introducción para quien se une al semillero y necesita practicar la mecánica de la
multiplicación matricial 4×4 antes de leer `mat4x4_mul.vhd` (lección 6, sección 4), que hace
lo mismo en hardware.

---

## 6. Flujo de trabajo recomendado: de MATLAB a VHDL

```
1. Definir/ajustar la geometría del robot en Metodo_DH_Peter.m
   -> visualizar con Robot.teach() para confirmar que la geometria es correcta
2. Calcular T05 de referencia con Verificador_DH.m para los angulos de prueba deseados
   -> convertir angulos y T05 esperado a Q2.13 (valor x 8192)
3. Cargar esos mismos valores en el testbench VHDL (fk6r_tb.vhd o fk2r_tb.vhd)
4. Simular en ModelSim/Questa (ver leccion 2) y comparar la matriz resultante
   contra la calculada en el paso 2, con tolerancia +-10 en Q2.13 (error acumulado del CORDIC)
```

---

*Ver también: [Cinemática 2R por matrices homogéneas (lección 6)](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) ·
[Cinemática DH del robot 6R (lección 7)](../7_Cinematica_Directa_DH/README.md)*
