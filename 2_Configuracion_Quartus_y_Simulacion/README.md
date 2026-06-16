# Configuración Quartus y Simulación

Lo primero que debemos de hacer es instalar Quartus desde el siguiente link (18.1) :
https://www.altera.com/downloads/fpga-development-tools/quartus-prime-lite-edition-design-software-version-18-1-windows

Posteriormente al abrir Quartus nos encontraremos con la interfaz de inicio, y buscaremos donde dice "New Proyect Wizard"

<img width="1679" height="993" alt="image" src="https://github.com/user-attachments/assets/004db07c-4e98-4653-af79-4bcc88dbfaf4" />

Posteriormente colocamos donde queremos que se ubique el archivo y el nombre del proyecto (Por default Quartus coloca al archivo top level el mismo nombre del proyecto, pero esto no es estrictamente necesario), Yo recomiendo crear una carpeta llamada como el proyecto y dentro de esta ubicarlo.

<img width="791" height="624" alt="image" src="https://github.com/user-attachments/assets/17621fbb-2624-4d59-9d1b-da355233eced" />

Damos "Next" 3 veces y en "Family, Device & Settings" colocamos nuestro modelo de FPGA Cyclone IV Core 8 (O la que tengamos nosotros) y la seleccionamos en la parte de abajo, como se muestra en la imagen con sombreado azul

<img width="789" height="624" alt="image" src="https://github.com/user-attachments/assets/f4d5c4dc-164e-4187-a96a-095928dc19f4" />

Luego en "EDA tools Settings" colocamos en Simulation "ModelSim - Altera" y "VHDL" como se evidencia en la imagen:

<img width="795" height="622" alt="image" src="https://github.com/user-attachments/assets/0bdacdef-10e4-4271-8889-2d6d5daddfcb" />

Damos click en Finish y ya tenemos nuestro proyecto creado en Quartus, si deseamos ver los archivos, vamos a la parte izquierda y vemos donde dice Hierarchy, y lo cambiamos por Files, adicionalmente le damos a la hoja blanca del panel superior para crear nuestro primer archivo VHDL

<img width="665" height="658" alt="image" src="https://github.com/user-attachments/assets/c3885c74-84a7-46ee-9400-3d09cc64ef92" />

En dicho archivo podemos colocar un ejemplo simple de detector de secuencia "101"s: 

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY SeqDetector IS
    PORT(
        clk, rst, x : IN  std_logic;
        detected    : OUT std_logic
    );
END SeqDetector;

ARCHITECTURE behavioral OF SeqDetector IS
    TYPE state_type IS (S0, S1, S2, S3);
    SIGNAL state, next_state : state_type;
BEGIN

    -- Registro de estado (secuencial)
    PROCESS(clk, rst)
    BEGIN
        IF rst = '1' THEN
            state <= S0;
        ELSIF rising_edge(clk) THEN
            state <= next_state;
        END IF;
    END PROCESS;

    -- Lógica de transición (combinacional)
    PROCESS(state, x)
    BEGIN
        CASE state IS
            WHEN S0 => IF x = '1' THEN next_state <= S1; ELSE next_state <= S0; END IF;
            WHEN S1 => IF x = '0' THEN next_state <= S2; ELSE next_state <= S1; END IF;
            WHEN S2 => IF x = '1' THEN next_state <= S3; ELSE next_state <= S0; END IF;
            WHEN S3 => IF x = '1' THEN next_state <= S1; ELSE next_state <= S0; END IF;
        END CASE;
    END PROCESS;

    -- Salida Moore (depende solo del estado)
    detected <= '1' WHEN state = S3 ELSE '0';

END behavioral;
```







