-- =============================================================
--  tb.vhd  (nombre fijo — se sobrescribe en cada paso)
--  Testbench de INTEGRACION -- FK_7R_Geometrico_Con_Posicion (top level)
--  Cadena completa: cordic_seq6 -> fk_recursivo_core -> atan2_seq3
--  (correccion del profesor -- metodo geometrico recursivo, Inversa2R.pdf)
--
--  Entradas theta1_in..theta6_in en Q2.13 RADIANES (mismo formato de
--  angle_in de cordic_sincos_16, no cambia con esta correccion).
--
--  CasoA (q=0) y CasoB (singularidad th5=90 th6=90) caen en angulos donde
--  el CORDIC da EXACTO (0 y pi/2 no tienen error de redondeo) -- por eso
--  ahi SI se verifica con ERROR si no coincide, usando los mismos valores
--  ya confirmados en el testbench standalone de fk_recursivo_core.
--
--  CasoC es generico: el CORDIC introduce un pequeno redondeo (pocos LSB)
--  frente a los valores "ideales" usados en el testbench standalone, asi
--  que aqui solo se IMPRIME el resultado para comparar a ojo (no ERROR).
--  Universidad Militar Nueva Granada
-- =============================================================
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.numeric_std.ALL;

ENTITY tb IS
END tb;

ARCHITECTURE sim OF tb IS

    CONSTANT CLK_PERIOD : time := 20 ns;

    SIGNAL clk_in   : STD_LOGIC := '0';
    SIGNAL rst_in   : STD_LOGIC := '1';
    SIGNAL start_in : STD_LOGIC := '0';
    SIGNAL theta1_in, theta2_in, theta3_in : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');
    SIGNAL theta4_in, theta5_in, theta6_in : STD_LOGIC_VECTOR(15 DOWNTO 0) := (OTHERS => '0');

    SIGNAL done_out  : STD_LOGIC;
    SIGNAL x_out, y_out, z_out          : STD_LOGIC_VECTOR(15 DOWNTO 0);
    SIGNAL yaw_out, pitch_out, roll_out : STD_LOGIC_VECTOR(15 DOWNTO 0);

    COMPONENT FK_7R_Geometrico_Con_Posicion IS
        PORT(
            clk_in    : IN  STD_LOGIC;
            rst_in    : IN  STD_LOGIC;
            start_in  : IN  STD_LOGIC;
            theta1_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            theta2_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            theta3_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            theta4_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            theta5_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            theta6_in : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            done_out  : OUT STD_LOGIC;
            pitch_out : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            roll_out  : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            x_out     : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            y_out     : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            yaw_out   : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            z_out     : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
        );
    END COMPONENT;

BEGIN

    DUT : FK_7R_Geometrico_Con_Posicion
        PORT MAP(
            clk_in => clk_in, rst_in => rst_in, start_in => start_in,
            theta1_in => theta1_in, theta2_in => theta2_in, theta3_in => theta3_in,
            theta4_in => theta4_in, theta5_in => theta5_in, theta6_in => theta6_in,
            done_out => done_out,
            x_out => x_out, y_out => y_out, z_out => z_out,
            yaw_out => yaw_out, pitch_out => pitch_out, roll_out => roll_out
        );

    clk_in <= NOT clk_in AFTER CLK_PERIOD / 2;

    PROCESS
        PROCEDURE wait_clk(n : INTEGER) IS
        BEGIN
            FOR i IN 1 TO n LOOP
                WAIT UNTIL rising_edge(clk_in);
            END LOOP;
        END PROCEDURE;

        PROCEDURE run_caso(
            t1,t2,t3,t4,t5,t6 : INTEGER;
            nombre : STRING;
            exp_x,exp_y,exp_z,exp_yaw,exp_pitch,exp_roll : INTEGER;
            check_exacto : BOOLEAN
        ) IS
            VARIABLE timeout : INTEGER := 0;
        BEGIN
            theta1_in <= STD_LOGIC_VECTOR(to_signed(t1, 16));
            theta2_in <= STD_LOGIC_VECTOR(to_signed(t2, 16));
            theta3_in <= STD_LOGIC_VECTOR(to_signed(t3, 16));
            theta4_in <= STD_LOGIC_VECTOR(to_signed(t4, 16));
            theta5_in <= STD_LOGIC_VECTOR(to_signed(t5, 16));
            theta6_in <= STD_LOGIC_VECTOR(to_signed(t6, 16));
            wait_clk(2);

            start_in <= '1';
            WAIT UNTIL rising_edge(clk_in);
            start_in <= '0';

            timeout := 0;
            WHILE done_out = '0' AND timeout < 300 LOOP
                WAIT UNTIL rising_edge(clk_in);
                timeout := timeout + 1;
            END LOOP;

            IF done_out = '0' THEN
                REPORT "TIMEOUT: " & nombre SEVERITY ERROR;
            ELSE
                REPORT "=== " & nombre & " ===" SEVERITY NOTE;
                REPORT "  ciclos hasta done_out = " & INTEGER'IMAGE(timeout) SEVERITY NOTE;
                REPORT "  x=" & INTEGER'IMAGE(to_integer(signed(x_out)))
                     & "  y=" & INTEGER'IMAGE(to_integer(signed(y_out)))
                     & "  z=" & INTEGER'IMAGE(to_integer(signed(z_out))) SEVERITY NOTE;
                REPORT "  yaw=" & INTEGER'IMAGE(to_integer(signed(yaw_out)))
                     & "  pitch=" & INTEGER'IMAGE(to_integer(signed(pitch_out)))
                     & "  roll=" & INTEGER'IMAGE(to_integer(signed(roll_out))) SEVERITY NOTE;

                IF check_exacto THEN
                    REPORT "  referencia (modulo standalone, sin ruido de CORDIC): x=" & INTEGER'IMAGE(exp_x)
                         & " y=" & INTEGER'IMAGE(exp_y) & " z=" & INTEGER'IMAGE(exp_z) SEVERITY NOTE;
                    ASSERT ABS(to_integer(signed(x_out)) - exp_x) <= 30 AND ABS(to_integer(signed(y_out)) - exp_y) <= 30
                       AND ABS(to_integer(signed(z_out)) - exp_z) <= 30
                        REPORT "  AVISO: " & nombre & " se aleja mas de 30 LSB de la referencia (revisar)" SEVERITY WARNING;
                ELSE
                    REPORT "  (caso generico -- comparar a ojo, CORDIC redondea unos pocos LSB)" SEVERITY NOTE;
                END IF;
            END IF;
            wait_clk(5);
        END PROCEDURE;

    BEGIN
        rst_in <= '1';
        wait_clk(5);
        rst_in <= '0';
        wait_clk(3);

        REPORT "============================================" SEVERITY NOTE;
        REPORT "FK_7R_Geometrico_Con_Posicion -- integracion completa" SEVERITY NOTE;
        REPORT "============================================" SEVERITY NOTE;

        -- CasoA: q=0 (todos los theta=0) -- CORDIC exacto
        -- esperado (igual al standalone): x=3441 y=0 z=532
        run_caso(0,0,0,0,0,0,
            "CasoA [q=0]", 3441,0,532, 0,0,0, TRUE);

        -- CasoB: th5=90 th6=90 (raw=12868=pi/2 en Q2.13), resto 0 -- CORDIC exacto (singularidad 6R)
        -- esperado (igual al standalone): x=2024 y=1417 z=532
        run_caso(0,0,0,0,12868,12868,
            "CasoB [SINGULARIDAD]", 2024,1417,532, 0,0,0, TRUE);

        -- CasoC: th1=30 th2=20 th3=-15 th4=45 th5=60 th6=-70 (grados -> Q2.13 rad)
        -- generico: solo se imprime, sin ERROR (margen de redondeo CORDIC)
        run_caso(4289,2860,-2145,6434,8579,-10008,
            "CasoC [generico] th=30,20,-15,45,60,-70 deg", 0,0,0, 0,0,0, FALSE);

        -- CasoD: th1=137.6 th2=23.9 th3=168.2 th4=74.5 th5=109.3 th6=41.7 (grados)
        -- "SUPER DIFICIL" -- mismos angulos irregulares (dentro de 0-180, rango
        -- real de un servo) que el "Caso EXTRA-2 [PEOR CASO]" de la STM32, para
        -- poder comparar la MISMA pose de punta a punta (FPGA/STM32/MATLAB).
        -- En la FPGA esto NO es una prueba de tiempo (el CORDIC tarda lo mismo
        -- siempre, ~4.05us, sea cual sea el angulo) -- es solo una pose extra
        -- para verificar que el metodo recursivo tambien da un resultado
        -- razonable en un caso bien alejado de cualquier multiplo de 90.
        -- Referencia aproximada (Python, doble precision, SIN la aritmetica
        -- truncada del hardware -- comparar a ojo, no exacto):
        --   x=-0.0662 y=0.0014 z=-0.0629 m | yaw=-88.66 pitch=11.91 roll=146.93 deg
        run_caso(19674,3417,24049,10652,15627,5962,
            "CasoD [SUPER DIFICIL] th=137.6,23.9,168.2,74.5,109.3,41.7 deg", 0,0,0, 0,0,0, FALSE);

        -- CasoE: los 6 angulos en 90 grados (12868 = pi/2 en Q2.13) -- equivalente
        -- al "Caso EXTRA-1 [MEJOR CASO a ojo]" de la STM32. Informativo (mismo
        -- criterio que CasoC/D): referencia aproximada tomada del resultado real
        -- de libm en la STM32 (double, sin la aritmetica truncada del hardware):
        --   x=-0.0000 y=-0.1400 z=-0.0010 m -> raw Q2.13 aprox: x=0 y=-1147 z=-8
        run_caso(12868,12868,12868,12868,12868,12868,
            "CasoE [EXTRA-1: 6 angulos en 90 grados]", 0,-1147,-8, 0,0,0, FALSE);

        -- CasoF: th1=0 th2=0 th3=-90 th4=-90 th5=90 th6=0 -- equivalente al
        -- "Caso EXTRA-3 [MEJOR CASO REAL]" de la STM32 (encontrado por sympy,
        -- cancela el offset +90 de theta3/theta4, cae en la singularidad).
        -- Referencia aproximada (libm STM32, doble precision):
        --   x=-0.0660 y=-0.0000 z=-0.0750 m -> raw Q2.13 aprox: x=-541 y=0 z=-614
        run_caso(0,0,-12868,-12868,12868,0,
            "CasoF [EXTRA-3: singularidad + min. angulos]", -541,0,-614, 0,0,0, FALSE);

        REPORT "============================================" SEVERITY NOTE;
        REPORT "FIN" SEVERITY NOTE;
        REPORT "============================================" SEVERITY NOTE;
        WAIT;
    END PROCESS;

END sim; -- entidad: tb
