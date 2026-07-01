library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Block2R_tb is
end Block2R_tb;

architecture sim of Block2R_tb is

    component Block2RTest
        port (
            clk    : in  std_logic;
            rst    : in  std_logic;
            Start  : in  std_logic;
            L1     : in  std_logic_vector(15 downto 0);
            L2     : in  std_logic_vector(15 downto 0);
            One    : in  std_logic_vector(15 downto 0);
            Px     : in  std_logic_vector(15 downto 0);
            Py     : in  std_logic_vector(15 downto 0);
            Two    : in  std_logic_vector(15 downto 0);
            tetha1 : out std_logic_vector(15 downto 0);
            thetha2: out std_logic_vector(15 downto 0)
        );
    end component;

    signal clk_s    : std_logic := '0';
    signal rst_s    : std_logic := '1';
    signal start_s  : std_logic := '0';
	 signal L1_s  : std_logic_vector(15 downto 0) := x"028F";  -- 0.08m × 8192 = 655
	 signal L2_s  : std_logic_vector(15 downto 0) := x"023D";  -- 0.07m × 8192 = 573
	 signal One_s : std_logic_vector(15 downto 0) := x"2000";  -- 1.0 × 8192 = 8192
	 signal Two_s : std_logic_vector(15 downto 0) := x"4000";  -- 2.0 × 8192 = 16384
    signal Px_s     : std_logic_vector(15 downto 0) := (others => '0');
    signal Py_s     : std_logic_vector(15 downto 0) := (others => '0');
    signal tetha1_s : std_logic_vector(15 downto 0);
    signal thetha2_s: std_logic_vector(15 downto 0);

    constant CLK_PERIOD : time := 10 ns;

    -- ================================================================
    -- Valores esperados Q2.13 (factor escala = 8192)
    -- ================================================================
    -- CASO 1: Px=0.5  Py=0.5
    --   cos(t2) = 0.2250  -> 1844
    --   sin(t2) = 0.9744  -> 7981
    --   theta2  = atan2(0.9744, 0.2250) = 1.3440 rad -> 11010
    --   L2*cos  = 0.090   ->  737
    --   L2*sin  = 0.3898  -> 3194
    --   k1      = L1 + L2*cos = 0.5 + 0.090 = 0.590 ->  4833
    --   angle_base = atan2(0.5, 0.5)  = 0.7854 rad  ->  6434
    --   angle_corr = atan2(0.3898, 0.590) = 0.5839 rad -> 4782
    --   theta1  = angle_base - angle_corr = 0.2015 rad -> 1651
    --
    -- CASO 2: Px=0.7  Py=0.0
    --   cos(t2) = 0.2000  -> 1638
    --   sin(t2) = 0.9798  -> 8029
    --   theta2  = atan2(0.9798, 0.2000) = 1.3694 rad -> 11215
    --   L2*cos  = 0.080   ->  655
    --   L2*sin  = 0.3919  -> 3211
    --   k1      = 0.5 + 0.080 = 0.580 ->  4751
    --   angle_base = atan2(0.0, 0.7)  = 0.0000 rad  ->     0
    --   angle_corr = atan2(0.3919, 0.580) = 0.5955 rad -> 4877
    --   theta1  = 0.0000 - 0.5955 = -0.5955 rad -> -4877
    --     (negativo es correcto: codo-arriba con Py=0)
    --
    -- CASO 3: Px=0.3  Py=0.6
    --   cos(t2) = 0.1000  ->  820
    --   sin(t2) = 0.9950  -> 8151
    --   theta2  = atan2(0.9950, 0.1000) = 1.4706 rad -> 12047
    --   L2*cos  = 0.040   ->  328
    --   L2*sin  = 0.3980  -> 3261
    --   k1      = 0.5 + 0.040 = 0.540 ->  4424
    --   angle_base = atan2(0.6, 0.3)  = 1.1071 rad  ->  9069
    --   angle_corr = atan2(0.3980, 0.540) = 0.6364 rad -> 5212
    --   theta1  = 1.1071 - 0.6364 = 0.4707 rad -> 3856
    -- ================================================================

begin

    DUT : Block2RTest
        port map (
            clk     => clk_s,
            rst     => rst_s,
            Start   => start_s,
            L1      => L1_s,
            L2      => L2_s,
            One     => One_s,
            Px      => Px_s,
            Py      => Py_s,
            Two     => Two_s,
            tetha1  => tetha1_s,
            thetha2 => thetha2_s
        );

    clk_s <= not clk_s after CLK_PERIOD / 2;

    stim_proc : process
    begin
        rst_s   <= '1';
        start_s <= '0';
        wait for 5 * CLK_PERIOD;
        rst_s   <= '0';
        wait for 2 * CLK_PERIOD;


        -- ============================================================
        -- CASO: Px=0.05  Py=0.05
        -- ============================================================
        report "=== CASO 3: Px=0.3 Py=0.6 ===" severity note;
        Px_s <= x"019A";  -- 0.05m × 8192 = 410
		  Py_s <= x"019A";  -- 0.05m × 8192 = 410
        start_s <= '1';
        wait for CLK_PERIOD;
        start_s <= '0';

        wait for 600 * CLK_PERIOD;

        report "CASO 3: thetha2=" &
               integer'image(to_integer(signed(thetha2_s))) &
               "  esperado ~ 12047" severity note;
        report "CASO 3: tetha1=" &
               integer'image(to_integer(signed(tetha1_s))) &
               "  esperado ~  3856" severity note;

        wait for 20 * CLK_PERIOD;
        report "=== SIMULACION COMPLETA ===" severity note;
        wait;
    end process;

end sim;