-- =============================================================
--  sqrt_q13.vhd  (Opcion A: puertos STD_LOGIC_VECTOR)
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity sqrt_q13 is
    Port (
        clk   : in  std_logic;
        rst   : in  std_logic;
        start : in  std_logic;
        x_in  : in  std_logic_vector(15 downto 0);
        y_out : out std_logic_vector(15 downto 0);
        done  : out std_logic
    );
end sqrt_q13;

architecture rtl of sqrt_q13 is
    constant N_ITER : integer := 16;
    type state_t is (S_IDLE, S_RUN, S_FINISH);
    signal state  : state_t := S_IDLE;
    signal rem_s  : unsigned(31 downto 0) := (others => '0');
    signal root_s : unsigned(15 downto 0) := (others => '0');
    signal cnt    : integer range 0 to N_ITER := 0;
    signal result : signed(15 downto 0) := (others => '0');
    signal done_r : std_logic := '0';
begin
    process(clk, rst)
        variable bit_v : unsigned(15 downto 0);
        variable trial : unsigned(31 downto 0);
    begin
        if rst = '1' then
            state  <= S_IDLE;
            rem_s  <= (others => '0');
            root_s <= (others => '0');
            cnt    <= 0;
            result <= (others => '0');
            done_r <= '0';
        elsif rising_edge(clk) then
            done_r <= '0';
            case state is
                when S_IDLE =>
                    if start = '1' then
                        -- Conversion SLV -> unsigned para escalar
                        rem_s  <= shift_left(
                                    resize(unsigned(x_in), 32), 13);
                        root_s <= (others => '0');
                        cnt    <= 0;
                        state  <= S_RUN;
                    end if;

                when S_RUN =>
                    bit_v := shift_left(to_unsigned(1, 16),
                                        N_ITER - 1 - cnt);
                    trial := resize(
                                (root_s or bit_v) * (root_s or bit_v),
                             32);
                    if trial <= rem_s then
                        root_s <= root_s or bit_v;
                    end if;
                    if cnt = N_ITER - 1 then
                        state <= S_FINISH;
                    else
                        cnt <= cnt + 1;
                    end if;

                when S_FINISH =>
                    result <= signed(resize(root_s, 16));
                    done_r <= '1';
                    state  <= S_IDLE;
            end case;
        end if;
    end process;

    -- Conversion signed -> SLV en la salida
    y_out <= std_logic_vector(result);
    done  <= done_r;

end rtl;