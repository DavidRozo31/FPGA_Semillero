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

En dicho archivo podemos colocar un ejemplo simple de detector de secuencia "101": 

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


> [!IMPORTANT]
> *CUIDADO*: Notese que la entidad se llama "SeqDetector" por lo que el nombre del archivo al ser guardado debe corresponder con el nombre de la entidad, de lo contrario no funcionara la compilación, asi que una vez guardado nuestro archivo con el nombre de la entidad, lo colocamos como "Top level entity" dando click derecho sobre el archivo en el panel de archivos y seleccionando la opcion "Set as Top level entity":


<img width="1285" height="666" alt="image" src="https://github.com/user-attachments/assets/95129a68-7ec1-4523-beec-5654225d4c05" />


Una vez hecho esto, compilamos el archivos presionando al boton de "play" azul que se encuentra en el panel superior:

<img width="846" height="223" alt="image" src="https://github.com/user-attachments/assets/0b429caf-c418-450e-8271-59e5044df21a" />

Una vez compilado, nos saldra el reporte de compilacion, que muestra la cantidad de elementos logicos usados para la construccion de nuestro hardware, asi como los pines y otras estadisticas de nuestra tarjeta:

<img width="594" height="604" alt="image" src="https://github.com/user-attachments/assets/3bf43e11-b09c-410e-b54a-dcfdbefd7e58" />

Ahora, para poder probar nuestro diseño usando un testbench, es necesario crearlo, por lo que vamos a hacer para ahorrar tiempo es lo siguiente, vamos a ir a una inteligencia artifical, y pedirle que nos realice un testbench para nuestro programa, y adicionalmente un simular.do para la visualizacion de nuestro programa en ModelSim, lo que nos daria lo siguiente:

> [!IMPORTANT]
> Ambos archivos deben guardarse en la carpeta del proyecto, el testbench puede ser creado desde el inicio como un vhdl normal o descargarlo directamente de la IA como vhdl, mientras que el .do lo mejor es descargarlo directamente de la IA.

El TestBench:

```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;

ENTITY SeqDetector_tb IS
END SeqDetector_tb;

ARCHITECTURE behavioral OF SeqDetector_tb IS

    -- Component declaration
    COMPONENT SeqDetector
        PORT(
            clk, rst, x : IN  std_logic;
            detected    : OUT std_logic
        );
    END COMPONENT;

    -- Signals
    SIGNAL clk      : std_logic := '0';
    SIGNAL rst      : std_logic := '0';
    SIGNAL x        : std_logic := '0';
    SIGNAL detected : std_logic;

    -- Clock period
    CONSTANT CLK_PERIOD : time := 20 ns;

BEGIN

    -- Unit Under Test
    UUT : SeqDetector
        PORT MAP (
            clk      => clk,
            rst      => rst,
            x        => x,
            detected => detected
        );

    -- Clock generation: 50 MHz
    CLK_PROC : PROCESS
    BEGIN
        clk <= '0';
        WAIT FOR CLK_PERIOD / 2;
        clk <= '1';
        WAIT FOR CLK_PERIOD / 2;
    END PROCESS;

    -- Stimulus process
    STIM_PROC : PROCESS
    BEGIN

        -- ------------------------------------------------
        -- Reset inicial
        -- ------------------------------------------------
        rst <= '1';
        x   <= '0';
        WAIT FOR CLK_PERIOD * 2;
        rst <= '0';
        WAIT FOR CLK_PERIOD;

        -- ------------------------------------------------
        -- TEST 1: Secuencia valida 1-0-1-1
        -- Se espera detected = '1' al final (estado S3)
        -- ------------------------------------------------
        REPORT "TEST 1: Secuencia 1011 - se espera detected=1";
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S0 -> S1
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S2 -> S3  (detected debe ser 1)
        x <= '1'; WAIT FOR CLK_PERIOD;  -- permanece en S3 un ciclo extra

        IF detected = '1' THEN
            REPORT "TEST 1 PASSED: detected = 1";
        ELSE
            REPORT "TEST 1 FAILED: detected = 0, se esperaba 1" SEVERITY ERROR;
        END IF;

        -- ------------------------------------------------
        -- Reset entre pruebas
        -- ------------------------------------------------
        rst <= '1';
        WAIT FOR CLK_PERIOD * 2;
        rst <= '0';
        WAIT FOR CLK_PERIOD;

        -- ------------------------------------------------
        -- TEST 2: Secuencia invalida 1-1-0-0
        -- Se espera detected = '0' en todo momento
        -- ------------------------------------------------
        REPORT "TEST 2: Secuencia 1100 - se espera detected=0";
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S0 -> S1
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S1 -> S1
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S2 -> S0

        IF detected = '0' THEN
            REPORT "TEST 2 PASSED: detected = 0";
        ELSE
            REPORT "TEST 2 FAILED: detected = 1, se esperaba 0" SEVERITY ERROR;
        END IF;

        -- ------------------------------------------------
        -- Reset entre pruebas
        -- ------------------------------------------------
        rst <= '1';
        WAIT FOR CLK_PERIOD * 2;
        rst <= '0';
        WAIT FOR CLK_PERIOD;

        -- ------------------------------------------------
        -- TEST 3: Secuencia solapada 1-0-1-1-0-1-1
        -- Primera deteccion en ciclo 4, segunda en ciclo 7
        -- Tras el primer 1011, el ultimo '1' mueve a S1,
        -- luego 0->S2, 1->S3 (segunda deteccion)
        -- ------------------------------------------------
        REPORT "TEST 3: Secuencia solapada 1011011 - dos detecciones";
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S0 -> S1
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S2 -> S3  (detected = 1)
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S3 -> S1  (detected = 0)
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S2 -> S3  (detected = 1)
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S3 -> S0

        IF detected = '0' THEN
            REPORT "TEST 3 PASSED: regresa a 0 correctamente";
        ELSE
            REPORT "TEST 3 FAILED: detected no regreso a 0" SEVERITY ERROR;
        END IF;

        -- ------------------------------------------------
        -- Reset entre pruebas
        -- ------------------------------------------------
        rst <= '1';
        WAIT FOR CLK_PERIOD * 2;
        rst <= '0';
        WAIT FOR CLK_PERIOD;

        -- ------------------------------------------------
        -- TEST 4: Entrada continua en 0 desde S0
        -- Maquina debe quedarse en S0, detected = 0
        -- ------------------------------------------------
        REPORT "TEST 4: Entrada siempre 0 - maquina permanece en S0";
        x <= '0'; WAIT FOR CLK_PERIOD * 5;

        IF detected = '0' THEN
            REPORT "TEST 4 PASSED: detected = 0 con entrada siempre 0";
        ELSE
            REPORT "TEST 4 FAILED: detected = 1 inesperado" SEVERITY ERROR;
        END IF;

        -- ------------------------------------------------
        -- TEST 5: Reset asincronico durante secuencia activa
        -- ------------------------------------------------
        REPORT "TEST 5: Reset asincronico durante secuencia";
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S0 -> S1
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        rst <= '1';                      -- Reset en medio de la secuencia
        WAIT FOR CLK_PERIOD;
        rst <= '0';
        x <= '1'; WAIT FOR CLK_PERIOD;  -- Debe reiniciar desde S0 -> S1
        x <= '0'; WAIT FOR CLK_PERIOD;  -- S1 -> S2
        x <= '1'; WAIT FOR CLK_PERIOD;  -- S2 -> S3

        IF detected = '1' THEN
            REPORT "TEST 5 PASSED: deteccion correcta luego del reset";
        ELSE
            REPORT "TEST 5 FAILED: no detecto luego del reset" SEVERITY ERROR;
        END IF;

        WAIT FOR CLK_PERIOD * 3;
        REPORT "Simulacion finalizada";
        WAIT;

    END PROCESS;

END behavioral;
```

El simular.do:


> [!IMPORTANT]
> *CUIDADO*: Notese que este simular .do referencia directamente archivos con ubicaciones dentro de mi computadora, cosas que variaran segun como hallas guardado tu proyecto y en el lugar en que lo hayas hecho, debes modificar las direcciones si no corresponden con las de tus archivos:


```vhdl
# ==============================================================
# Script de simulacion para SeqDetector
# Proyecto: C:\intelFPGA_lite\18.1\Projects\Tutorial
# ==============================================================

# Crear y mapear la libreria de trabajo
vlib work
vmap work work

# Compilar el diseno original
vcom -93 -work work "C:/intelFPGA_lite/18.1/Projects/Tutorial/SeqDetector.vhd"

# Compilar el testbench
vcom -93 -work work "C:/intelFPGA_lite/18.1/Projects/Tutorial/SeqDetector_tb.vhd"

# Iniciar la simulacion
vsim -t ns work.SeqDetector_tb

# Agregar seniales al visor de ondas
add wave -divider "Entradas"
add wave -color Yellow  -label "clk"      /SeqDetector_tb/clk
add wave -color Cyan    -label "rst"      /SeqDetector_tb/rst
add wave -color Green   -label "x"        /SeqDetector_tb/x

add wave -divider "Salida"
add wave -color Orange  -label "detected" /SeqDetector_tb/detected

add wave -divider "Estado interno"
add wave -color White   -label "state"      /SeqDetector_tb/UUT/state
add wave -color Magenta -label "next_state" /SeqDetector_tb/UUT/next_state

# Configurar escala de tiempo
configure wave -timelineunits ns

# Correr la simulacion completa
run -all

# Ajustar el zoom para ver todas las seniales
wave zoom full

```

Una vez hayamos guardado ambos archivos, lo primero es colocar el archivo del testbench como un "Testbench" dentro de quartus, para ello damos click derecho en el archivo y le damos a "Properties" y seleccionamos el tipo de archivo como "VHDL testbench file"

<img width="648" height="467" alt="image" src="https://github.com/user-attachments/assets/ed418146-cb15-4712-901c-0b7a93500eaa" />

Luego, debemos decirle a Quartus que ese es el testbench que queremos ejecutar y simular, por lo que vamos a "Assignments", luego a "Settings" y a la opcion de "Simulation"

<img width="935" height="823" alt="image" src="https://github.com/user-attachments/assets/50bb9bf3-d005-4fc1-acd8-8ecdefc37768" />

Posteriormente, seleccionamos la opcion de "Compile Test Bench" y luego en "Test Benches..."

<img width="619" height="213" alt="image" src="https://github.com/user-attachments/assets/be230fb3-8160-4454-9d26-3fec8c5dc01e" />

Luego en "New"

<img width="711" height="320" alt="image" src="https://github.com/user-attachments/assets/06e3d688-d8e8-437c-89f4-1d790ff1331a" />

Y buscamos en los tres punticos nuestro Testbench y lo agregamos dandole "Add", y adicionalmente le colocamos el nombre en la parte superior:

<img width="518" height="691" alt="image" src="https://github.com/user-attachments/assets/30882dc8-fec2-4c0c-bfcb-35e1a6ebc8ba" />

Damos Ok y volvemos a la parte de "Settings"donde ahora toca indicarle el archivo .do a usar, por lo que seleccionamos la opcion que dice "use script so set up simulation"
y buscamos el .do y lo seleccionamos

<img width="637" height="315" alt="image" src="https://github.com/user-attachments/assets/10bea31e-c408-4ae3-9ea7-cf606a70e132" />

Le damos "Apply" y Ok, y ya tenemos todo listo para simular por lo que vamos a "Tools" - "Run Simulation" - "RTL Simulation" :

<img width="604" height="482" alt="image" src="https://github.com/user-attachments/assets/6f101c4e-3358-49d6-a80b-74f7be23db24" />

y ya tenemos simulacion:

<img width="1658" height="509" alt="image" src="https://github.com/user-attachments/assets/ea7c79a0-34b1-4947-b12f-e7003a9acddb" />













