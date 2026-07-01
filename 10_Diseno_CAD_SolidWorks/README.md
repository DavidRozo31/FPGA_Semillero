# Diseño CAD del Brazo Robótico — SolidWorks

## Descripción General

Ficha de referencia (no técnica de VHDL) del diseño mecánico del brazo robótico físico que
controlan los proyectos de las lecciones [6](../6_Cinematica_Directa_Matrices_Homogeneas/README.md),
[7](../7_Cinematica_Directa_DH/README.md) y [9](../9_Control_Servos_UART/README.md). Los
archivos fuente viven en `SolidWorks/` (carpeta local de trabajo, no incluida en este
repositorio educativo por su peso — ver nota al final).

> **Universidad Militar Nueva Granada**

---

## Estructura del diseño

| Carpeta / archivo | Contenido |
|---|---|
| `Ensamble_Brazo/` | Ensamble propio del semillero: `Ensamble_Brazocompleto.SLDASM`, piezas propias (`Pieza_U_5grado.SLDPRT`, `Tubo_brazo.SLDPRT`, brackets para servos MG995/MG996R/DS3115/RDS3225) |
| `ma-6-dof-robot-*` | Modelo de referencia de un brazo 6 DOF completo (descargado, usado como guía de proporciones) |
| `robot-6-dof-with-gripper-*` | Modelo de referencia con gripper incluido |
| `gripper-with-servo-motor-*` | Modelo de gripper accionado por servo |
| `mg995-servo-motor-*`, `mg996r-*`, `rds-3225-*` | Modelos 3D de los servomotores reales usados en el brazo, para verificar encaje mecánico con los brackets propios |
| `servo-motor-u-bracket-*` | Soportes en U para los servos |

## Relación con los proyectos VHDL/MATLAB

Las longitudes de eslabón (`L1`, `L2`, `L3`...) y los parámetros DH (`d`, `a`) usados en
las lecciones 6-8 deben coincidir con las medidas reales del ensamble
`Ensamble_Brazocompleto.SLDASM` — si se modifica una pieza que cambia la distancia entre
ejes de rotación (por ejemplo `Tubo_brazo.SLDPRT`), hay que actualizar la constante
correspondiente en el testbench VHDL (`fk2r_tb.vhd` o `fk6r_tb.vhd`) y en los scripts
MATLAB (`Verificador_DH.m`, `Metodo_DH_Peter.m`) para que la cinemática simulada siga
correspondiendo al robot físico.

## Servos soportados (compatibilidad mecánica verificada en el CAD)

- MG995 / MG996R (torque estándar)
- DS3115
- RDS3225 (mayor torque, usado en las juntas de mayor carga)

## Nota sobre archivos pesados

Los modelos 3D descargados (`.zip`/`.STEP`, varios de 15-30 MB cada uno) no se suben a este
repositorio de documentación — el repo del semillero está pensado como material didáctico
en Markdown (ver el resto de las lecciones), no como almacén de binarios de CAD. Si se
necesita el archivo fuente de una pieza puntual, pedirlo directamente en el grupo de trabajo
del semillero o generarlo de nuevo desde el sitio de origen del modelo de referencia.

---

*Ver también: [Cinemática 2R (lección 6)](../6_Cinematica_Directa_Matrices_Homogeneas/README.md) ·
[Cinemática DH (lección 7)](../7_Cinematica_Directa_DH/README.md) ·
[Control de servos (lección 9)](../9_Control_Servos_UART/README.md)*
