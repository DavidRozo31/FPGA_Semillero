-- =============================================================
--  tb_inverse_kinematics.vhd
--  Testbench simple - Robot 2R/3R Cinematica Inversa
--  UMNG - Mecatronica
--
--  Modulos probados:
--    SECCION 1 : fp_multiplier      (Q2.13 x Q2.13)
--    SECCION 2 : fp_adder           (Q2.13 +/- Q2.13)
--    SECCION 3 : sqrt_q13           (sqrt Q2.13)
--    SECCION 4 : cordic_atan2       (atan2 Q2.13)
--    SECCION 5 : cordic_sincos_16   (sin/cos Q2.13)
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;
use work.ik_pkg.ALL;

entity tb_inverse_kinematics is
end tb_inverse_kinematics;

architecture sim of tb_inverse_kinematics is

    constant CLK_PERIOD : time := 20 ns;

    -- Convierte real -> Q2.13
    function to_q13(x : real) return signed is
        variable raw : integer;
    begin
        raw := integer(x * 8192.0);
        if raw >  32767 then raw :=  32767; end if;
        if raw < -32768 then raw := -32768; end if;
        return to_signed(raw, 16);
    end function;

    -- Convierte Q2.13 -> real
    function from_q13(x : signed(15 downto 0)) return real is
    begin
        return real(to_integer(x)) / 8192.0;
    end function;

    -- Convierte radianes a grados
    function to_deg(r : real) return real is
    begin
        return r * 57.29577951;
    end function;

    -- ===========================================================
    --  Senales globales
    -- ===========================================================
    signal clk            : std_logic := '0';
    signal rst            : std_logic := '1';
    signal seccion_activa : integer   := 0;   -- visible en el wave

    -- ===========================================================
    --  SECCION 1: fp_multiplier
    -- ===========================================================
    signal mul_a  : signed(15 downto 0) := (others => '0');
    signal mul_b  : signed(15 downto 0) := (others => '0');
    signal mul_vi : std_logic := '0';
    signal mul_p  : signed(15 downto 0);
    signal mul_vo : std_logic;

    -- ===========================================================
    --  SECCION 2: fp_adder
    -- ===========================================================
    signal add_a  : signed(15 downto 0) := (others => '0');
    signal add_b  : signed(15 downto 0) := (others => '0');
    signal add_op : std_logic := '0';
    signal add_vi : std_logic := '0';
    signal add_p  : signed(15 downto 0);
    signal add_vo : std_logic;

    -- ===========================================================
    --  SECCION 3: sqrt_q13
    -- ===========================================================
    signal sq_x     : signed(15 downto 0) := (others => '0');
    signal sq_start : std_logic := '0';
    signal sq_y     : signed(15 downto 0);
    signal sq_done  : std_logic;

    -- ===========================================================
    --  SECCION 4: cordic_atan2
    -- ===========================================================
    signal at_x     : signed(15 downto 0) := (others => '0');
    signal at_y     : signed(15 downto 0) := (others => '0');
    signal at_start : std_logic := '0';
    signal at_angle : signed(15 downto 0);
    signal at_done  : std_logic;

    -- ===========================================================
    --  SECCION 5: cordic_sincos_16
    -- ===========================================================
    signal sc_angle : signed(15 downto 0) := (others => '0');
    signal sc_start : std_logic := '0';
    signal sc_sin   : signed(15 downto 0);
    signal sc_cos   : signed(15 downto 0);
    signal sc_done  : std_logic;

    -- ===========================================================
    --  Componentes
    -- ===========================================================
    component fp_multiplier is
        Port (
            clk     : in  std_logic;
            rst     : in  std_logic;
            a_in    : in  signed(15 downto 0);
            b_in    : in  signed(15 downto 0);
            valid_i : in  std_logic;
            p_out   : out signed(15 downto 0);
            valid_o : out std_logic
        );
    end component;

    component fp_adder is
        Port (
            clk     : in  std_logic;
            rst     : in  std_logic;
            a_in    : in  signed(15 downto 0);
            b_in    : in  signed(15 downto 0);
            op      : in  std_logic;
            valid_i : in  std_logic;
            p_out   : out signed(15 downto 0);
            valid_o : out std_logic
        );
    end component;

    component sqrt_q13 is
        Port (
            clk   : in  std_logic;
            rst   : in  std_logic;
            start : in  std_logic;
            x_in  : in  signed(15 downto 0);
            y_out : out signed(15 downto 0);
            done  : out std_logic
        );
    end component;

    component cordic_atan2 is
        Port (
            clk   : in  std_logic;
            rst   : in  std_logic;
            start : in  std_logic;
            x_in  : in  signed(15 downto 0);
            y_in  : in  signed(15 downto 0);
            angle : out signed(15 downto 0);
            done  : out std_logic
        );
    end component;

    component cordic_sincos_16 is
        Port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            start    : in  std_logic;
            angle_in : in  signed(15 downto 0);
            sin_out  : out signed(15 downto 0);
            cos_out  : out signed(15 downto 0);
            done     : out std_logic
        );
    end component;

begin

    clk <= not clk after CLK_PERIOD / 2;

    -- ===========================================================
    --  Instancias
    -- ===========================================================
    DUT_MUL : fp_multiplier
        port map (clk=>clk, rst=>rst,
                  a_in=>mul_a, b_in=>mul_b, valid_i=>mul_vi,
                  p_out=>mul_p, valid_o=>mul_vo);

    DUT_ADD : fp_adder
        port map (clk=>clk, rst=>rst,
                  a_in=>add_a, b_in=>add_b, op=>add_op, valid_i=>add_vi,
                  p_out=>add_p, valid_o=>add_vo);

    DUT_SQRT : sqrt_q13
        port map (clk=>clk, rst=>rst,
                  start=>sq_start, x_in=>sq_x,
                  y_out=>sq_y, done=>sq_done);

    DUT_ATAN2 : cordic_atan2
        port map (clk=>clk, rst=>rst,
                  start=>at_start, x_in=>at_x, y_in=>at_y,
                  angle=>at_angle, done=>at_done);

    DUT_SINCOS : cordic_sincos_16
        port map (clk=>clk, rst=>rst,
                  start=>sc_start, angle_in=>sc_angle,
                  sin_out=>sc_sin, cos_out=>sc_cos, done=>sc_done);

    -- ===========================================================
    --  Proceso principal
    -- ===========================================================
    process

        -- -------------------------------------------------------
        --  SECCION 1 - MULTIPLICADOR
        --  Cambia A_MUL y B_MUL
        -- -------------------------------------------------------
        constant A_MUL : real := 0.6;
        constant B_MUL : real := 0.6;

        -- -------------------------------------------------------
        --  SECCION 2 - SUMADOR / RESTADOR
        --  OP_ADD: '0'=suma  '1'=resta
        -- -------------------------------------------------------
        constant A_ADD  : real      := 0.36;
        constant B_ADD  : real      := 0.16;
        constant OP_ADD : std_logic := '0';

        -- -------------------------------------------------------
        --  SECCION 3 - SQRT
        --  X_SQRT debe ser >= 0.0
        -- -------------------------------------------------------
        constant X_SQRT : real := 0.52;

        -- -------------------------------------------------------
        --  SECCION 4 - ATAN2
        --  Calcula atan2(Y_AT, X_AT) en radianes
        --  Ejemplo: atan2(0.4, 0.6) ~ 33.69 deg ~ 0.5880 rad
        -- -------------------------------------------------------
        constant X_AT : real := 0.6;
        constant Y_AT : real := 0.4;

        -- -------------------------------------------------------
        --  SECCION 5 - SIN / COS
        --  ANG_SC en radianes
        --  Ejemplo: 0.5880 rad -> sin~0.5547  cos~0.8321
        -- -------------------------------------------------------
        constant ANG_SC : real := 0.5880;

    begin
        -- Reset
        rst            <= '1';
        seccion_activa <= 0;
        wait for CLK_PERIOD * 3;
        rst <= '0';
        wait for CLK_PERIOD;

        -- =======================================================
        --  SECCION 1: fp_multiplier
        -- =======================================================
        seccion_activa <= 1;
        wait for 1 ns;
        report "------ SECCION 1: fp_multiplier ------" severity note;

        mul_a  <= to_q13(A_MUL);
        mul_b  <= to_q13(B_MUL);
        mul_vi <= '1';
        wait until rising_edge(clk);
        mul_vi <= '0';
        wait until rising_edge(clk);

        report "MUL  A        = " & real'image(A_MUL)           severity note;
        report "MUL  B        = " & real'image(B_MUL)           severity note;
        report "MUL  Resultado= " & real'image(from_q13(mul_p)) severity note;
        report "MUL  Esperado = " & real'image(A_MUL * B_MUL)   severity note;

        wait for CLK_PERIOD * 3;

        -- =======================================================
        --  SECCION 2: fp_adder
        -- =======================================================
        seccion_activa <= 2;
        wait for 1 ns;
        report "------ SECCION 2: fp_adder ------" severity note;

        add_a  <= to_q13(A_ADD);
        add_b  <= to_q13(B_ADD);
        add_op <= OP_ADD;
        add_vi <= '1';
        wait until rising_edge(clk);
        add_vi <= '0';
        wait until rising_edge(clk);

        if OP_ADD = '0' then
            report "ADD  Operacion = SUMA  (op=0)"                  severity note;
            report "ADD  Resultado= " & real'image(from_q13(add_p)) severity note;
            report "ADD  Esperado = " & real'image(A_ADD + B_ADD)   severity note;
        else
            report "ADD  Operacion = RESTA (op=1)"                  severity note;
            report "ADD  Resultado= " & real'image(from_q13(add_p)) severity note;
            report "ADD  Esperado = " & real'image(A_ADD - B_ADD)   severity note;
        end if;

        wait for CLK_PERIOD * 3;

        -- =======================================================
        --  SECCION 3: sqrt_q13
        -- =======================================================
        seccion_activa <= 3;
        wait for 1 ns;
        report "------ SECCION 3: sqrt_q13 ------" severity note;

        sq_x     <= to_q13(X_SQRT);
        sq_start <= '1';
        wait until rising_edge(clk);
        sq_start <= '0';

        for i in 0 to 30 loop
            if sq_done = '1' then exit; end if;
            wait until rising_edge(clk);
        end loop;

        if sq_done /= '1' then
            report "SQRT ERROR: done nunca llego en 30 ciclos" severity error;
        else
            report "SQRT X        = " & real'image(X_SQRT)         severity note;
            report "SQRT Resultado= " & real'image(from_q13(sq_y)) severity note;
        end if;

        wait for CLK_PERIOD * 3;

        -- =======================================================
        --  SECCION 4: cordic_atan2
        -- =======================================================
        seccion_activa <= 4;
        wait for 1 ns;
        report "------ SECCION 4: cordic_atan2 ------" severity note;

        at_x     <= to_q13(X_AT);
        at_y     <= to_q13(Y_AT);
        at_start <= '1';
        wait until rising_edge(clk);
        at_start <= '0';

        for i in 0 to 30 loop
            if at_done = '1' then exit; end if;
            wait until rising_edge(clk);
        end loop;

        if at_done /= '1' then
            report "ATAN2 ERROR: done nunca llego en 30 ciclos" severity error;
        else
            report "ATAN2  X        = " & real'image(X_AT)                         severity note;
            report "ATAN2  Y        = " & real'image(Y_AT)                          severity note;
            report "ATAN2  Resultado= " & real'image(from_q13(at_angle)) & " rad"   severity note;
            report "ATAN2  En grados= " & real'image(to_deg(from_q13(at_angle)))    severity note;
        end if;

        wait for CLK_PERIOD * 3;

        -- =======================================================
        --  SECCION 5: cordic_sincos_16
        -- =======================================================
        seccion_activa <= 5;
        wait for 1 ns;
        report "------ SECCION 5: cordic_sincos_16 ------" severity note;

        sc_angle <= to_q13(ANG_SC);
        sc_start <= '1';
        wait until rising_edge(clk);
        sc_start <= '0';

        report "SINCOS iniciado, esperando done..." severity note;

        for i in 0 to 30 loop
            if sc_done = '1' then exit; end if;
            wait until rising_edge(clk);
        end loop;

        if sc_done /= '1' then
            report "SINCOS ERROR: done nunca llego en 30 ciclos" severity error;
            report "SINCOS Verifica: N_ITER en cordic_pkg = 12?" severity error;
        else
            report "SINCOS  Angulo  = " & real'image(ANG_SC) & " rad"  severity note;
            report "SINCOS  sin     = " & real'image(from_q13(sc_sin)) severity note;
            report "SINCOS  cos     = " & real'image(from_q13(sc_cos)) severity note;
        end if;

        wait for CLK_PERIOD * 5;
        wait;
    end process;

end sim;