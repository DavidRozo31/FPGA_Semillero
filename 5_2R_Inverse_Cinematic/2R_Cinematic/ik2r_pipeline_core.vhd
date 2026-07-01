-- =============================================================
--  ik2r_pipeline_core.vhd
--  Cinemática Inversa Robot 2R — Núcleo completamente pipelined
--
--  CORRECCIONES APLICADAS (v2):
--    BUG 1: error_latch ahora se captura en atan3_done='1'
--           en vez de done_r='1' (evita captura del valor anterior)
--    BUG 2: Shift registers indexados con (0) en vez de (L_DELAY/COS_DELAY)
--           El índice 0 contiene el valor más antiguo = el correcto
--           cuando se accede COS_DELAY/L_DELAY ciclos después de escribirlo
--    BUG 3: k2_latch ahora usa k2_delayed (1 ciclo extra de registro)
--           para alinearse correctamente con k1_v y k1_out
--
--  Pipeline temporal:
--  T+0  : start — x, y, L1, L2 disponibles
--  T+1  : fp_multiplier×5 PARALELO + cordic_atan2① arranca (atan2(y,x))
--           x²,  y²,  L1²,  L2²,  2·L1·L2
--  T+2  : fp_adder×2 PARALELO  r²=x²+y²  |  Σ=L1²+L2²
--  T+3  : fp_adder              num=r²−Σ
--  T+4..21: fp_divider (18 ciclos) cosθ₂=num/(2·L1·L2)
--  T+22 : fp_mult  cos²θ₂ = cosθ₂×cosθ₂
--  T+23 : fp_adder 1−cos²θ₂
--  T+24..41: sqrt_q13 (18 ciclos)  sinθ₂=√(1−cos²θ₂)
--  T+42..54: cordic_atan2② (13 ciclos) θ₂=atan2(sinθ₂,cosθ₂)
--            PARALELO: fp_mult k2=L2·sinθ₂,  fp_mult+adder k1=L1+L2·cosθ₂
--  T+43..55: cordic_atan2③ (13 ciclos) α=atan2(k2,k1)
--  T+56 : θ₁=atan2(y,x)−α   DONE
--
--  Formato: Q2.13 (16 bits signed), 1 LSB ≈ 0.000122 rad
--  Precisión: ~0.007 rad (≈0.4°)
--
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;
use work.cordic_pkg.ALL;

entity ik2r_pipeline_core is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;
        -- Entradas Q2.13
        px_in      : in  signed(15 downto 0);
        py_in      : in  signed(15 downto 0);
        l1_in      : in  signed(15 downto 0);
        l2_in      : in  signed(15 downto 0);
        -- Salidas Q2.13 (radianes), solución codo arriba
        theta1_out : out signed(15 downto 0);
        theta2_out : out signed(15 downto 0);
        done       : out std_logic;
        error_out  : out std_logic   -- '1' si |cosθ₂| > 1
    );
end ik2r_pipeline_core;

architecture rtl of ik2r_pipeline_core is

    -- =========================================================
    --  Declaraciones de componentes
    -- =========================================================
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

    component fp_divider is
        Port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            start    : in  std_logic;
            num_in   : in  signed(15 downto 0);
            den_in   : in  signed(15 downto 0);
            quot_out : out signed(15 downto 0);
            done     : out std_logic
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

    -- =========================================================
    --  CONSTANTE: ONE en Q2.13 = 8192
    -- =========================================================
    constant ONE_16 : signed(15 downto 0) := to_signed(8192, 16);

    -- =========================================================
    --  ETAPA 0 → ETAPA 1: registros de entrada
    -- =========================================================
    signal px_r, py_r   : signed(15 downto 0) := (others => '0');
    signal l1_r, l2_r   : signed(15 downto 0) := (others => '0');
    signal start_r      : std_logic := '0';

    -- =========================================================
    --  ETAPA 1: multiplicadores paralelos (todos 1 ciclo)
    --  M_PX2 = Px², M_PY2 = Py², M_L1SQ = L1²,
    --  M_L2SQ = L2², M_2L1L2 = 2·L1·L2
    --  + arranque de atan2①  atan2(Py, Px)
    -- =========================================================
    signal mul_px2_out   : signed(15 downto 0);
    signal mul_py2_out   : signed(15 downto 0);
    signal mul_l1sq_out  : signed(15 downto 0);
    signal mul_l2sq_out  : signed(15 downto 0);
    signal mul_l1l2_out  : signed(15 downto 0);   -- L1·L2 (luego ×2)
    signal mul_v1        : std_logic;

    -- 2·L1·L2 calculado combinatoriamente en etapa 2
    signal two_l1l2      : signed(15 downto 0);

    -- =========================================================
    --  ETAPA 2: adders paralelos
    --  ADD_R2  = Px² + Py²        (r²)
    --  ADD_SUM = L1² + L2²        (Σ)
    -- =========================================================
    signal add_r2_out    : signed(15 downto 0);
    signal add_sum_out   : signed(15 downto 0);
    signal add2_v        : std_logic;

    -- Shift register para 2·L1·L2 (necesita llegar a div start = T+4)
    -- T+1: mul_l1l2_out válido → T+2: ×2 → T+3: registrado → T+4: start div
    signal two_l1l2_d1   : signed(15 downto 0) := (others => '0');
    signal two_l1l2_d2   : signed(15 downto 0) := (others => '0');

    -- =========================================================
    --  ETAPA 3: num = r² − (L1²+L2²)
    -- =========================================================
    signal add_num_out   : signed(15 downto 0);
    signal add3_v        : std_logic;

    -- =========================================================
    --  ETAPA 4..21: divisor  cosθ₂ = num / (2·L1·L2)
    --  Latencia: 18 ciclos (señal start en T+3, done en T+21)
    -- =========================================================
    signal div_start_s   : std_logic := '0';
    signal div_num_s     : signed(15 downto 0) := (others => '0');
    signal div_den_s     : signed(15 downto 0) := (others => '0');
    signal cos_t2        : signed(15 downto 0);
    signal div_done      : std_logic;

    -- =========================================================
    --  ETAPA 22: cos²θ₂ = cosθ₂ × cosθ₂  (1 ciclo)
    -- =========================================================
    signal cos_t2_latch  : signed(15 downto 0) := (others => '0');
    signal mul_cos2_out  : signed(15 downto 0);
    signal mul_cos2_v    : std_logic;

    -- =========================================================
    --  ETAPA 23: 1 − cos²θ₂  (1 ciclo)
    -- =========================================================
    signal add_1mc2_out  : signed(15 downto 0);
    signal add_1mc2_v    : std_logic;

    -- =========================================================
    --  ETAPA 24..41: sqrt  sinθ₂ = √(1−cos²θ₂)  (18 ciclos)
    -- =========================================================
    signal sqrt_start_s  : std_logic := '0';
    signal sqrt_x_s      : signed(15 downto 0) := (others => '0');
    signal sin_t2        : signed(15 downto 0);
    signal sqrt_done     : std_logic;

    -- =========================================================
    --  FIX BUG 2: Shift register para cos_t2
    --  cos_t2_latch válido en T+21 (div_done)
    --  sin_t2 válido en T+41 (sqrt_done, ~20 ciclos después)
    --  El índice (0) contiene el valor escrito hace COS_DELAY ciclos
    --  = el que necesitamos cuando sqrt_done llega
    -- =========================================================
    constant COS_DELAY : integer := 20;
    type cos_sr_t is array (0 to COS_DELAY) of signed(15 downto 0);
    signal cos_t2_sr     : cos_sr_t := (others => (others => '0'));

    -- =========================================================
    --  ETAPA 42..54: cordic_atan2② θ₂ = atan2(sinθ₂, cosθ₂)
    -- =========================================================
    signal atan2_t2_start : std_logic := '0';
    signal atan2_t2_x     : signed(15 downto 0) := (others => '0');
    signal atan2_t2_y     : signed(15 downto 0) := (others => '0');
    signal theta2_raw     : signed(15 downto 0);
    signal atan2_t2_done  : std_logic;

    -- =========================================================
    --  PARALELO a θ₂: k1 = L1 + L2·cosθ₂  y  k2 = L2·sinθ₂
    -- =========================================================
    signal mul_k2_out    : signed(15 downto 0);   -- L2·sinθ₂
    signal mul_k2_v      : std_logic;
    signal mul_l2c_out   : signed(15 downto 0);   -- L2·cosθ₂
    signal mul_l2c_v     : std_logic;

    -- =========================================================
    --  FIX BUG 2: Shift registers para L1, L2
    --  L1, L2 disponibles en T+0
    --  Necesitan estar en T+42 cuando sqrt_done llega
    --  Índice (0) = valor escrito hace L_DELAY ciclos = correcto
    -- =========================================================
    constant L_DELAY : integer := 42;
    type l_sr_t is array (0 to L_DELAY) of signed(15 downto 0);
    signal l1_sr         : l_sr_t := (others => (others => '0'));
    signal l2_sr         : l_sr_t := (others => (others => '0'));

    signal k1_out        : signed(15 downto 0);   -- L1 + L2·cosθ₂
    signal k1_v          : std_logic;

    -- =========================================================
    --  FIX BUG 3: k2_delayed — 1 ciclo extra de delay para alinear
    --  k2 (mul_k2_out) válido en T+43 (1 ciclo después de sqrt_done)
    --  k1 (k1_out)     válido en T+44 (2 ciclos después de sqrt_done)
    --  → k2 necesita 1 ciclo de delay para coincidir con k1_v
    -- =========================================================
    signal k2_delayed    : signed(15 downto 0) := (others => '0');
    signal k1_latch      : signed(15 downto 0) := (others => '0');
    signal k2_latch      : signed(15 downto 0) := (others => '0');

    -- =========================================================
    --  ETAPA 43..55: cordic_atan2③  α = atan2(k2, k1)
    -- =========================================================
    signal atan3_start   : std_logic := '0';
    signal atan3_x       : signed(15 downto 0) := (others => '0');
    signal atan3_y       : signed(15 downto 0) := (others => '0');
    signal alpha_out     : signed(15 downto 0);
    signal atan3_done    : std_logic;

    -- =========================================================
    --  cordic_atan2① atan2(Py, Px) — arranca en T+1
    --  Latencia 13 ciclos → resultado en T+14
    --  Latched hasta que α esté disponible (~T+56)
    -- =========================================================
    signal atan1_start   : std_logic := '0';
    signal atan1_x       : signed(15 downto 0) := (others => '0');
    signal atan1_y       : signed(15 downto 0) := (others => '0');
    signal atan1_raw     : signed(15 downto 0);
    signal atan1_done    : std_logic;
    signal atan1_latch   : signed(15 downto 0) := (others => '0');

    -- =========================================================
    --  ETAPA FINAL: θ₁ = atan2(Py,Px) − α
    -- =========================================================
    signal theta1_raw    : signed(15 downto 0);
    signal theta1_v      : std_logic;

    -- =========================================================
    --  FIX BUG 1: error_r y error_latch
    --  error_latch se captura en atan3_done='1', no en done_r='1'
    --  Así error_r ya está estable cuando se latchea
    -- =========================================================
    signal error_r       : std_logic := '0';
    signal error_latch   : std_logic := '0';

    -- =========================================================
    --  Salidas latched
    -- =========================================================
    signal t1_latch      : signed(15 downto 0) := (others => '0');
    signal t2_latch      : signed(15 downto 0) := (others => '0');
    signal done_r        : std_logic := '0';

begin

    -- =========================================================
    --  REGISTRO DE ENTRADA (T+0 → T+1)
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            px_r    <= (others => '0');
            py_r    <= (others => '0');
            l1_r    <= (others => '0');
            l2_r    <= (others => '0');
            start_r <= '0';
        elsif rising_edge(clk) then
            px_r    <= px_in;
            py_r    <= py_in;
            l1_r    <= l1_in;
            l2_r    <= l2_in;
            start_r <= start;
        end if;
    end process;

    -- =========================================================
    --  ETAPA 1: MULTIPLICADORES PARALELOS × 5
    --  + arranque atan2①
    -- =========================================================

    -- Px²
    U_MUL_PX2 : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => px_r, b_in => px_r,
        valid_i => start_r,
        p_out => mul_px2_out, valid_o => mul_v1
    );

    -- Py²
    U_MUL_PY2 : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => py_r, b_in => py_r,
        valid_i => start_r,
        p_out => mul_py2_out, valid_o => open
    );

    -- L1²
    U_MUL_L1SQ : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => l1_r, b_in => l1_r,
        valid_i => start_r,
        p_out => mul_l1sq_out, valid_o => open
    );

    -- L2²
    U_MUL_L2SQ : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => l2_r, b_in => l2_r,
        valid_i => start_r,
        p_out => mul_l2sq_out, valid_o => open
    );

    -- L1·L2 (luego ×2 con shift_left)
    U_MUL_L1L2 : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => l1_r, b_in => l2_r,
        valid_i => start_r,
        p_out => mul_l1l2_out, valid_o => open
    );

    -- atan2① arranca en T+1 con los operandos registrados en T+0
    process(clk, rst)
    begin
        if rst = '1' then
            atan1_start <= '0';
            atan1_x     <= (others => '0');
            atan1_y     <= (others => '0');
        elsif rising_edge(clk) then
            atan1_start <= start_r;
            atan1_x     <= px_r;
            atan1_y     <= py_r;
        end if;
    end process;

    U_ATAN2_1 : cordic_atan2 port map (
        clk   => clk, rst => rst,
        start => atan1_start,
        x_in  => atan1_x,
        y_in  => atan1_y,
        angle => atan1_raw,
        done  => atan1_done
    );

    -- Latch de atan2① cuando llega done (~T+14)
    -- Permanece estable hasta que atan3_done llega (~T+56)
    process(clk, rst)
    begin
        if rst = '1' then
            atan1_latch <= (others => '0');
        elsif rising_edge(clk) then
            if atan1_done = '1' then
                atan1_latch <= atan1_raw;
            end if;
        end if;
    end process;

    -- =========================================================
    --  ETAPA 2: ADDERS PARALELOS
    --  r² = Px² + Py²
    --  Σ  = L1² + L2²
    --  2·L1·L2: shift_left(L1·L2, 1)
    -- =========================================================
    two_l1l2 <= shift_left(mul_l1l2_out, 1);

    U_ADD_R2 : fp_adder port map (
        clk => clk, rst => rst,
        a_in => mul_px2_out, b_in => mul_py2_out,
        op => '0', valid_i => mul_v1,
        p_out => add_r2_out, valid_o => add2_v
    );

    U_ADD_SUM : fp_adder port map (
        clk => clk, rst => rst,
        a_in => mul_l1sq_out, b_in => mul_l2sq_out,
        op => '0', valid_i => mul_v1,
        p_out => add_sum_out, valid_o => open
    );

    -- 2·L1·L2 necesita llegar en T+3 (junto con num para el divisor)
    -- T+1: mul_l1l2 válido → T+2: two_l1l2 (combinatorio) → T+3: registrado
    process(clk, rst)
    begin
        if rst = '1' then
            two_l1l2_d1 <= (others => '0');
            two_l1l2_d2 <= (others => '0');
        elsif rising_edge(clk) then
            two_l1l2_d1 <= two_l1l2;    -- T+2
            two_l1l2_d2 <= two_l1l2_d1; -- T+3
        end if;
    end process;

    -- =========================================================
    --  ETAPA 3: num = r² − Σ
    -- =========================================================
    U_ADD_NUM : fp_adder port map (
        clk => clk, rst => rst,
        a_in => add_r2_out, b_in => add_sum_out,
        op => '1', valid_i => add2_v,
        p_out => add_num_out, valid_o => add3_v
    );

    -- =========================================================
    --  ETAPA 4..21: DIVISOR  cosθ₂ = num / (2·L1·L2)
    --  start cuando add3_v = '1'
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            div_start_s <= '0';
            div_num_s   <= (others => '0');
            div_den_s   <= (others => '0');
        elsif rising_edge(clk) then
            div_start_s <= add3_v;
            div_num_s   <= add_num_out;
            div_den_s   <= two_l1l2_d2;   -- alineado con num en T+3
        end if;
    end process;

    U_DIV : fp_divider port map (
        clk      => clk, rst => rst,
        start    => div_start_s,
        num_in   => div_num_s,
        den_in   => div_den_s,
        quot_out => cos_t2,
        done     => div_done
    );

    -- =========================================================
    --  FIX BUG 1: Detección de error y latch de cos_t2
    --  error_r se setea cuando div_done='1' y |cosθ₂| > 1
    --  error_latch se captura en atan3_done='1' (no en done_r)
    --  para garantizar que error_r ya está estable
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            error_r      <= '0';
            error_latch  <= '0';
            cos_t2_latch <= (others => '0');
        elsif rising_edge(clk) then
            if div_done = '1' then
                cos_t2_latch <= cos_t2;
                -- |cosθ₂| > 1 significa punto fuera del workspace
                if abs_q13(cos_t2) > ONE_16 then
                    error_r <= '1';
                else
                    error_r <= '0';
                end if;
            end if;
            -- FIX: Capturar error en atan3_done, no en done_r
            -- En este punto error_r lleva ~35 ciclos estable = confiable
            if atan3_done = '1' then
                error_latch <= error_r;
            end if;
        end if;
    end process;

    -- =========================================================
    --  ETAPA 22: cos²θ₂ = cosθ₂ × cosθ₂
    -- =========================================================
    U_MUL_COS2 : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in => cos_t2_latch, b_in => cos_t2_latch,
        valid_i => div_done,
        p_out => mul_cos2_out, valid_o => mul_cos2_v
    );

    -- =========================================================
    --  ETAPA 23: 1 − cos²θ₂
    -- =========================================================
    U_ADD_1MC2 : fp_adder port map (
        clk => clk, rst => rst,
        a_in => ONE_16, b_in => mul_cos2_out,
        op => '1', valid_i => mul_cos2_v,
        p_out => add_1mc2_out, valid_o => add_1mc2_v
    );

    -- =========================================================
    --  ETAPA 24..41: SQRT  sinθ₂ = √(1 − cos²θ₂)
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            sqrt_start_s <= '0';
            sqrt_x_s     <= (others => '0');
        elsif rising_edge(clk) then
            sqrt_start_s <= add_1mc2_v;
            sqrt_x_s     <= add_1mc2_out;
        end if;
    end process;

    U_SQRT : sqrt_q13 port map (
        clk   => clk, rst => rst,
        start => sqrt_start_s,
        x_in  => sqrt_x_s,
        y_out => sin_t2,
        done  => sqrt_done
    );

    -- =========================================================
    --  FIX BUG 2: Shift register para cos_t2
    --  Escritura: cos_t2_sr(COS_DELAY) <= ... (posición más alta = nueva)
    --             cos_t2_sr(k-1) <= cos_t2_sr(k) (desplaza hacia abajo)
    --  Lectura:   cos_t2_sr(0) = valor escrito hace COS_DELAY ciclos
    --
    --  Cuando sqrt_done llega (COS_DELAY ciclos después de div_done),
    --  cos_t2_sr(0) contiene el cosθ₂ correcto.
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            cos_t2_sr <= (others => (others => '0'));
        elsif rising_edge(clk) then
            cos_t2_sr(COS_DELAY) <= cos_t2_latch;          -- entrada nueva
            for k in COS_DELAY downto 1 loop
                cos_t2_sr(k-1) <= cos_t2_sr(k);            -- desplazar
            end loop;
        end if;
    end process;

    -- =========================================================
    --  FIX BUG 2: Shift registers para L1, L2
    --  Misma lógica: escritura en (L_DELAY), lectura en (0)
    --  Cuando sqrt_done llega (T+42), l1_sr(0) y l2_sr(0)
    --  contienen los valores de L1 y L2 del cálculo en curso.
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            l1_sr <= (others => (others => '0'));
            l2_sr <= (others => (others => '0'));
        elsif rising_edge(clk) then
            l1_sr(L_DELAY) <= l1_in;                        -- entrada nueva
            l2_sr(L_DELAY) <= l2_in;
            for k in L_DELAY downto 1 loop
                l1_sr(k-1) <= l1_sr(k);                     -- desplazar
                l2_sr(k-1) <= l2_sr(k);
            end loop;
        end if;
    end process;

    -- =========================================================
    --  ETAPA 42: atan2② y multiplicadores k1, k2 en paralelo
    --  Cuando sqrt_done='1':
    --    - cordic_atan2② arranca: θ₂ = atan2(sinθ₂, cosθ₂)
    --    - fp_mult: k2 = L2·sinθ₂
    --    - fp_mult: L2·cosθ₂  (para k1)
    --
    --  FIX BUG 2: usar (0) para acceder al valor correcto del SR
    -- =========================================================

    -- atan2② para θ₂
    process(clk, rst)
    begin
        if rst = '1' then
            atan2_t2_start <= '0';
            atan2_t2_x     <= (others => '0');
            atan2_t2_y     <= (others => '0');
        elsif rising_edge(clk) then
            atan2_t2_start <= sqrt_done;
            atan2_t2_x     <= cos_t2_sr(0);   -- FIX: índice (0), no (COS_DELAY)
            atan2_t2_y     <= sin_t2;
        end if;
    end process;

    U_ATAN2_T2 : cordic_atan2 port map (
        clk   => clk, rst => rst,
        start => atan2_t2_start,
        x_in  => atan2_t2_x,
        y_in  => atan2_t2_y,
        angle => theta2_raw,
        done  => atan2_t2_done
    );

    -- k2 = L2 · sinθ₂
    -- FIX BUG 2: usar l2_sr(0)
    U_MUL_K2 : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in    => l2_sr(0),    -- FIX: índice (0), no (L_DELAY)
        b_in    => sin_t2,
        valid_i => sqrt_done,
        p_out   => mul_k2_out,
        valid_o => mul_k2_v
    );

    -- L2 · cosθ₂ (componente de k1)
    -- FIX BUG 2: usar l2_sr(0) y cos_t2_sr(0)
    U_MUL_L2C : fp_multiplier port map (
        clk => clk, rst => rst,
        a_in    => l2_sr(0),        -- FIX: índice (0)
        b_in    => cos_t2_sr(0),    -- FIX: índice (0)
        valid_i => sqrt_done,
        p_out   => mul_l2c_out,
        valid_o => mul_l2c_v
    );

    -- k1 = L1 + L2·cosθ₂  (adder, 1 ciclo)
    -- FIX BUG 2: usar l1_sr(0)
    -- NOTA: l1_sr(0) y mul_l2c_out llegan en el mismo ciclo (T+43)
    --       porque el multiplicador tiene 1 ciclo de latencia desde sqrt_done
    U_ADD_K1 : fp_adder port map (
        clk => clk, rst => rst,
        a_in    => l1_sr(0),    -- FIX: índice (0)
        b_in    => mul_l2c_out,
        op      => '0',
        valid_i => mul_l2c_v,
        p_out   => k1_out,
        valid_o => k1_v
    );

    -- =========================================================
    --  FIX BUG 3: Alinear k1 y k2 para atan2③
    --
    --  ANTES (bug):
    --    mul_k2_out válido en T+43 (mul_k2_v='1')
    --    k1_out     válido en T+44 (k1_v='1')
    --    → Al latchear en k1_v, mul_k2_out ya tiene el valor de T+44
    --      (un ciclo adelantado = incorrecto o basura)
    --
    --  AHORA (fix):
    --    k2_delayed <= mul_k2_out  (registrado 1 ciclo)
    --    → k2_delayed válido en T+44 = mismo ciclo que k1_out
    --    → Al latchear en k1_v, k2_delayed tiene el valor correcto de T+43
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            k2_delayed <= (others => '0');
            k1_latch   <= (others => '0');
            k2_latch   <= (others => '0');
        elsif rising_edge(clk) then
            -- Delay de 1 ciclo para k2
            k2_delayed <= mul_k2_out;   -- FIX BUG 3: delay explícito

            -- Latch cuando k1 está listo (T+44)
            if k1_v = '1' then
                k1_latch <= k1_out;
                k2_latch <= k2_delayed;   -- FIX BUG 3: usar k2_delayed
            end if;
        end if;
    end process;

    -- =========================================================
    --  ETAPA 43..55: cordic_atan2③  α = atan2(k2, k1)
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            atan3_start <= '0';
            atan3_x     <= (others => '0');
            atan3_y     <= (others => '0');
        elsif rising_edge(clk) then
            atan3_start <= k1_v;
            atan3_x     <= k1_latch;
            atan3_y     <= k2_latch;
        end if;
    end process;

    U_ATAN2_3 : cordic_atan2 port map (
        clk   => clk, rst => rst,
        start => atan3_start,
        x_in  => atan3_x,
        y_in  => atan3_y,
        angle => alpha_out,
        done  => atan3_done
    );

    -- =========================================================
    --  ETAPA FINAL: θ₁ = atan2(Py,Px) − α
    --  atan1_latch tiene atan2(Py,Px) latcheado desde T+14
    --  alpha_out llega cuando atan3_done='1' (~T+56)
    --
    --  FIX BUG 1: error_latch se captura aquí también (en atan3_done)
    --  para que esté disponible cuando done_r='1' al ciclo siguiente
    -- =========================================================
    process(clk, rst)
    begin
        if rst = '1' then
            t1_latch <= (others => '0');
            t2_latch <= (others => '0');
            done_r   <= '0';
        elsif rising_edge(clk) then
            done_r <= '0';
            -- Latch θ₂ cuando atan2② termina
            if atan2_t2_done = '1' then
                t2_latch <= theta2_raw;
            end if;
            -- Calcular y latchear θ₁ cuando α está listo; activar done
            if atan3_done = '1' then
                t1_latch <= atan1_latch - alpha_out;
                done_r   <= '1';
            end if;
        end if;
    end process;

    -- =========================================================
    --  SALIDAS
    -- =========================================================
    theta1_out <= t1_latch;
    theta2_out <= t2_latch;
    done       <= done_r;
    error_out  <= error_latch;

end rtl;