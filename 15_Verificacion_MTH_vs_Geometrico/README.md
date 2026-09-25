# Clase 15: verificacion cruzada MATLAB vs FPGA

## Objetivo

Esta clase verifica la cinematica directa del robot de **6 grados de libertad** comparando dos caminos independientes:

- **MATLAB:** matrices de transformacion homogenea `T01*T12*T23*T34*T45*T56`.
- **FPGA:** resultados obtenidos en ModelSim con el nucleo VHDL de cinematica directa.

Toda la comparacion utiliza el formato **Q3.20 de 24 bits con signo**:

```text
valor_real = valor_Q3.20 / 2^20
valor_Q3.20 = round(valor_real * 2^20)
2^20 = 1048576
```

Las posiciones `X`, `Y` y `Z` se interpretan en metros. `Roll`, `Pitch` y `Yaw` se almacenan en radianes Q3.20 y se muestran tambien convertidos a grados.

Para hacer una comparacion justa, MATLAB usa los mismos angulos de entrada cuantizados en Q3.20 que recibe `tb.vhd`, no solamente los angulos nominales escritos en grados.

Script de verificacion: [`Verificacion_MTH_vs_Geometrico.m`](Verificacion_MTH_vs_Geometrico.m).

## Caso A

Entrada nominal: `theta = [0, 0, 0, 0, 0, 0] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | 440402 | 440405 | 0.420000 m | 0.420003 m | +3 LSB |
| Y | 0 | 1 | 0.000000 m | 0.000001 m | +1 LSB |
| Z | 68157 | 68159 | 0.065000 m | 0.065001 m | +2 LSB |
| Roll | -1647099 | -1647096 | -90.000000 deg | -89.999818 deg | +3 LSB |
| Pitch | 0 | 4 | 0.000000 deg | 0.000219 deg | +4 LSB |
| Yaw | -1647099 | -1647096 | -90.000000 deg | -89.999818 deg | +3 LSB |

## Caso B

Entrada nominal: `theta = [0, 0, 0, 0, 90, 90] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | 258998 | 259000 | 0.247000 m | 0.247002 m | +2 LSB |
| Y | 181404 | 181405 | 0.173000 m | 0.173001 m | +1 LSB |
| Z | 68157 | 68158 | 0.065000 m | 0.065001 m | +1 LSB |
| Roll | 3294199 | 3294199 | 180.000000 deg | 180.000019 deg | 0 LSB |
| Pitch | 1647099 | 1647084 | 89.999982 deg | 89.999162 deg | -15 LSB |
| Yaw | 0 | 0 | 0.000000 deg | 0.000000 deg | 0 LSB |

Este caso esta en singularidad. Con `Pitch` cercano a `90 grados`, el VHDL fija por convencion `Yaw=0` y `Roll=180 grados`.

## Caso C

Entrada nominal: `theta = [30, 20, -15, 45, 60, -70] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | 232279 | 232277 | 0.221518 m | 0.221517 m | -2 LSB |
| Y | 262378 | 262378 | 0.250223 m | 0.250223 m | 0 LSB |
| Z | 237895 | 237898 | 0.226874 m | 0.226877 m | +3 LSB |
| Roll | -685711 | -685696 | -37.468266 deg | -37.467467 deg | +15 LSB |
| Pitch | -632509 | -632504 | -34.561265 deg | -34.560976 deg | +5 LSB |
| Yaw | -777715 | -777726 | -42.495517 deg | -42.496126 deg | -11 LSB |

## Caso D

Entrada nominal: `theta = [137.6, 23.9, 168.2, 74.5, 109.3, 41.7] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | -69433 | -69434 | -0.066216 m | -0.066217 m | -1 LSB |
| Y | 1442 | 1441 | 0.001376 m | 0.001374 m | -1 LSB |
| Z | -65908 | -65907 | -0.062854 m | -0.062854 m | +1 LSB |
| Roll | 2689036 | 2689033 | 146.933005 deg | 146.932833 deg | -3 LSB |
| Pitch | 217998 | 217994 | 11.911734 deg | 11.911522 deg | -4 LSB |
| Yaw | -1622664 | -1622664 | -88.664800 deg | -88.664817 deg | 0 LSB |

## Caso E

Entrada nominal: `theta = [90, 90, 90, 90, 90, 90] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | 0 | 0 | 0.000000 m | 0.000000 m | 0 LSB |
| Y | -146801 | -146803 | -0.140000 m | -0.140002 m | -2 LSB |
| Z | -1048 | -1049 | -0.001000 m | -0.001000 m | -1 LSB |
| Roll | -3294198 | -3294191 | -179.999946 deg | -179.999582 deg | +7 LSB |
| Pitch | 0 | -4 | -0.000018 deg | -0.000219 deg | -4 LSB |
| Yaw | 3294199 | 3294199 | 180.000000 deg | 180.000019 deg | 0 LSB |

## Caso F

Entrada nominal: `theta = [0, 0, -90, -90, 90, 0] grados`.

| Valor | MATLAB Q3.20 | FPGA Q3.20 | MATLAB real | FPGA real | Error FPGA-MATLAB |
|---|---:|---:|---:|---:|---:|
| X | -69206 | -69208 | -0.066000 m | -0.066002 m | -2 LSB |
| Y | 0 | -1 | 0.000000 m | -0.000001 m | -1 LSB |
| Z | -78643 | -78645 | -0.075000 m | -0.075002 m | -2 LSB |
| Roll | 3294199 | 3294199 | 180.000000 deg | 180.000019 deg | 0 LSB |
| Pitch | 1647099 | 1647088 | 89.999964 deg | 89.999381 deg | -11 LSB |
| Yaw | 0 | 0 | 0.000000 deg | 0.000000 deg | 0 LSB |

Este caso tambien esta en singularidad. Se aplica la misma convencion `Yaw=0`, `Roll=180 grados`.

## Resultado general

Los seis casos coinciden dentro del error esperado por la aproximacion CORDIC y las operaciones de punto fijo:

- Error maximo de posicion: **3 LSB**, equivalente a `0.002861 mm`.
- Error maximo angular: **15 LSB**, equivalente a aproximadamente `0.000820 grados`.
- No se encontraron diferencias geometricas entre la cadena MTH de MATLAB y la implementacion de la FPGA.

## Como ejecutar

Abrir [`Verificacion_MTH_vs_Geometrico.m`](Verificacion_MTH_vs_Geometrico.m) en MATLAB y ejecutar el archivo completo. La consola imprime un bloque independiente para cada caso y una fila para cada valor `X`, `Y`, `Z`, `Roll`, `Pitch` y `Yaw`.
