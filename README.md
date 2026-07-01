# FPGA_Semillero

Curso paso a paso del semillero: cinemática de robots implementada en VHDL sobre FPGA
(Cyclone IV/V, Quartus), con verificación previa en MATLAB. Cada carpeta numerada es una
lección independiente, en español, con el código fuente completo incrustado y explicado
línea por línea.

## Índice de lecciones

| # | Lección | Tema | Nivel |
|---|---|---|---|
| 1 | [VHDL_Desde_Cero](1_VHDL_Desde_Cero/README.md) | Fundamentos del lenguaje VHDL: tipos, señales, procesos | Fundamento |
| 2 | [Configuracion_Quartus_y_Simulacion](2_Configuracion_Quartus_y_Simulacion/README.md) | Cómo compilar y simular testbenches en Quartus/ModelSim | Fundamento |
| 3 | [Modulo_PWM_50Hz](3_Modulo_PWM_50Hz/README.md) | Generador PWM para control de servomotores | FPGA |
| 4 | [Comunicacion_UART](4_Comunicacion_UART/README.md) | Transmisor/receptor UART con máquina de estados | FPGA |
| 5 | [2R_Inverse_Cinematic](5_2R_Inverse_Cinematic/Operadores.md) ([Atan2](5_2R_Inverse_Cinematic/Atan2.md)) | Operadores aritméticos Q2.13 y CORDIC `atan2` para cinemática inversa 2R | FPGA — histórico |
| 6 | [Cinematica_Directa_Matrices_Homogeneas](6_Cinematica_Directa_Matrices_Homogeneas/README.md) | Cinemática directa robot 2R por matrices homogéneas | FPGA — cinemática |
| 7 | [Cinematica_Directa_DH](7_Cinematica_Directa_DH/README.md) | Cinemática directa robot 5 eslabones por Denavit-Hartenberg | FPGA — cinemática |
| 8 | [MATLAB_Peter_Corke_Cinematica](8_MATLAB_Peter_Corke_Cinematica/README.md) | Simulación y verificación con el Robotics Toolbox de Peter Corke | MATLAB |
| 9 | [Control_Servos_UART](9_Control_Servos_UART/README.md) | De 1 a 6 servomotores controlados por UART (hardware + GUIs en Python) | FPGA — aplicación |
| 10 | [Diseno_CAD_SolidWorks](10_Diseno_CAD_SolidWorks/README.md) | Referencia del diseño mecánico del brazo físico | CAD — referencia |

### Orden de lectura recomendado

`1 → 2 → 3 → 4 → 5 (opcional/histórico) → 6 → 7 → 8 → 9 → 10`

Las lecciones 1-4 sientan las bases del lenguaje y las herramientas. La lección 5 documenta
un intento anterior de cinemática **inversa** (ya no es la arquitectura activa del robot 2R,
pero sus operadores aritméticos se reutilizan en las lecciones 6 y 7). Las lecciones 6-8 son
el núcleo del semillero: cinemática **directa** por los dos métodos clásicos, con su
contraparte de verificación en MATLAB. Las lecciones 9-10 documentan cómo ese cálculo se
traduce en movimiento físico del brazo.
