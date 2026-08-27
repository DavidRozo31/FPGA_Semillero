library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity tb_Cinematica_Inversa3R is
end tb_Cinematica_Inversa3R;

architecture sim of tb_Cinematica_Inversa3R is

    component Cinematica_Inversa3R
        PORT (
            clk         : IN  STD_LOGIC;
            reset       : IN  STD_LOGIC;
            Start       : IN  STD_LOGIC;
            Px_d        : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            Py_d        : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            Pz_d        : IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
            Ik3R_Ready  : OUT STD_LOGIC;
            tetha_one   : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            tetha_three : OUT STD_LOGIC_VECTOR(15 DOWNTO 0);
            tetha_two   : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
        );
    end component;

    signal clk_tb    : std_logic := '0';
    signal reset_tb  : std_logic := '1';
    signal start_tb  : std_logic := '0';
    signal px_tb     : std_logic_vector(15 downto 0) := (others => '0');
    signal py_tb     : std_logic_vector(15 downto 0) := (others => '0');
    signal pz_tb     : std_logic_vector(15 downto 0) := (others => '0');
    signal ready_tb  : std_logic;
    signal theta1_tb : std_logic_vector(15 downto 0);
    signal theta2_tb : std_logic_vector(15 downto 0);
    signal theta3_tb : std_logic_vector(15 downto 0);

    constant CLK_PERIOD : time := 20 ns;

    -- Caso 1: primer cuadrante
    constant TEST1_PX : std_logic_vector(15 downto 0) := x"0333"; -- 0.100 m
    constant TEST1_PY : std_logic_vector(15 downto 0) := x"019A"; -- 0.050 m
    constant TEST1_PZ : std_logic_vector(15 downto 0) := x"04CD"; -- 0.150 m

    -- Caso 2: theta1 en cuadrante II (Px<0, Py>0)
    constant TEST2_PX : std_logic_vector(15 downto 0) := std_logic_vector(to_signed(-655, 16));
    constant TEST2_PY : std_logic_vector(15 downto 0) := std_logic_vector(to_signed( 492, 16));
    constant TEST2_PZ : std_logic_vector(15 downto 0) := x"04CD"; -- 1229

    -- Caso 3: theta1 en cuadrante III (Px<0, Py<0)
    constant TEST3_PX : std_logic_vector(15 downto 0) := std_logic_vector(to_signed(-410, 16));
    constant TEST3_PY : std_logic_vector(15 downto 0) := std_logic_vector(to_signed(-737, 16));
    constant TEST3_PZ : std_logic_vector(15 downto 0) := x"0333"; -- 819

    -- Caso 4: theta1 en cuadrante IV (Px>0, Py<0)
    constant TEST4_PX : std_logic_vector(15 downto 0) := x"02E1"; -- 737
    constant TEST4_PY : std_logic_vector(15 downto 0) := std_logic_vector(to_signed(-328, 16));
    constant TEST4_PZ : std_logic_vector(15 downto 0) := x"05C3"; -- 1475

    -- Caso 5: brazo casi extendido
    constant TEST5_PX : std_logic_vector(15 downto 0) := x"07D7"; -- 2007
    constant TEST5_PY : std_logic_vector(15 downto 0) := x"0000"; -- 0
    constant TEST5_PZ : std_logic_vector(15 downto 0) := x"01EC"; -- 492

    -- Caso 6: brazo casi plegado
    constant TEST6_PX : std_logic_vector(15 downto 0) := x"011F"; -- 287
    constant TEST6_PY : std_logic_vector(15 downto 0) := x"0000"; -- 0
    constant TEST6_PZ : std_logic_vector(15 downto 0) := x"01EC"; -- 492

    -- Caso 7: theta1 = 45 grados exacto
    constant TEST7_PX : std_logic_vector(15 downto 0) := x"02E1"; -- 737
    constant TEST7_PY : std_logic_vector(15 downto 0) := x"02E1"; -- 737
    constant TEST7_PZ : std_logic_vector(15 downto 0) := x"04CD"; -- 1229

begin

    DUT : Cinematica_Inversa3R
        PORT MAP (
            clk         => clk_tb,
            reset       => reset_tb,
            Start       => start_tb,
            Px_d        => px_tb,
            Py_d        => py_tb,
            Pz_d        => pz_tb,
            Ik3R_Ready  => ready_tb,
            tetha_one   => theta1_tb,
            tetha_three => theta3_tb,
            tetha_two   => theta2_tb
        );

    -- Generacion de reloj
    CLK_GEN : process
    begin
        clk_tb <= '0';
        wait for CLK_PERIOD / 2;
        clk_tb <= '1';
        wait for CLK_PERIOD / 2;
    end process;

    -- Estimulos: corre los 7 casos en secuencia
    STIM : process
        procedure run_case(
            px_val : in std_logic_vector(15 downto 0);
            py_val : in std_logic_vector(15 downto 0);
            pz_val : in std_logic_vector(15 downto 0);
            nombre : in string
        ) is
        begin
            px_tb <= px_val;
            py_tb <= py_val;
            pz_tb <= pz_val;
            wait until rising_edge(clk_tb);
            start_tb <= '1';
            wait until rising_edge(clk_tb);
            start_tb <= '0';

            wait until ready_tb = '1';
            wait for CLK_PERIOD * 2;

            report nombre severity note;
            report "theta1 (raw) = " & integer'image(to_integer(signed(theta1_tb))) severity note;
            report "theta2 (raw) = " & integer'image(to_integer(signed(theta2_tb))) severity note;
            report "theta3 (raw) = " & integer'image(to_integer(signed(theta3_tb))) severity note;

            wait for CLK_PERIOD * 10;
        end procedure;
    begin
        -- Reset inicial
        reset_tb <= '1';
        start_tb <= '0';
        px_tb <= (others => '0');
        py_tb <= (others => '0');
        pz_tb <= (others => '0');
        wait for CLK_PERIOD * 5;
        reset_tb <= '0';
        wait for CLK_PERIOD * 2;

        run_case(TEST1_PX, TEST1_PY, TEST1_PZ, "Caso 1: primer cuadrante");
        run_case(TEST2_PX, TEST2_PY, TEST2_PZ, "Caso 2: theta1 cuadrante II");
        run_case(TEST3_PX, TEST3_PY, TEST3_PZ, "Caso 3: theta1 cuadrante III");
        run_case(TEST4_PX, TEST4_PY, TEST4_PZ, "Caso 4: theta1 cuadrante IV");
        run_case(TEST5_PX, TEST5_PY, TEST5_PZ, "Caso 5: brazo casi extendido");
        run_case(TEST6_PX, TEST6_PY, TEST6_PZ, "Caso 6: brazo casi plegado");
        run_case(TEST7_PX, TEST7_PY, TEST7_PZ, "Caso 7: theta1 = 45 grados");

        report "Fin de la simulacion - todos los casos ejecutados" severity note;
        wait;
    end process;

end sim;