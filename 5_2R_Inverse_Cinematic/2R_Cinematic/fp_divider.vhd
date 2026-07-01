library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fp_divider is
    Port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        num_in   : in  std_logic_vector(15 downto 0);
        den_in   : in  std_logic_vector(15 downto 0);
        quot_out : out std_logic_vector(15 downto 0);
        done     : out std_logic
    );
end fp_divider;

architecture rtl of fp_divider is
    constant N_DIV : integer := 13;
    type state_t is (S_IDLE, S_RUN, S_FINISH);
    signal state  : state_t := S_IDLE;
    signal rem_s  : unsigned(31 downto 0) := (others => '0');
    signal den_s  : unsigned(31 downto 0) := (others => '0');
    signal quot_s : unsigned(31 downto 0) := (others => '0');
    signal neg_s  : std_logic := '0';
    signal cnt    : integer range 0 to N_DIV := 0;
    signal result : signed(15 downto 0) := (others => '0');
    signal done_r : std_logic := '0';
    signal flag   : std_logic := '0';
    signal num_s  : signed(15 downto 0);
    signal den_sig: signed(15 downto 0);
begin
    num_s   <= signed(num_in);
    den_sig <= signed(den_in);

    process(clk)
        variable rem_v   : unsigned(31 downto 0);
        variable q16     : signed(15 downto 0);
        variable abs_num : unsigned(15 downto 0);
        variable abs_den : unsigned(15 downto 0);
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state  <= S_IDLE;
                flag   <= '0';
                rem_s  <= (others => '0');
                den_s  <= (others => '0');
                quot_s <= (others => '0');
                neg_s  <= '0';
                cnt    <= 0;
                result <= (others => '0');
                done_r <= '0';
            else
                case state is
                    when S_IDLE =>
                        if start = '1' or flag = '1' then
                            flag  <= '1';
                            neg_s <= num_s(15) xor den_sig(15);

                            if num_s(15) = '1' then
                                abs_num := unsigned(-num_s);
                            else
                                abs_num := unsigned(num_s);
                            end if;

                            if den_sig(15) = '1' then
                                abs_den := unsigned(-den_sig);
                            else
                                abs_den := unsigned(den_sig);
                            end if;

                            -- Sin escalar, 13 iteraciones dan resultado en Q2.13
                            rem_s  <= resize(abs_num, 32);
                            den_s  <= resize(abs_den, 32);
                            quot_s <= (others => '0');
                            cnt    <= 0;
                            state  <= S_RUN;
                        end if;

                    when S_RUN =>
                        rem_v := shift_left(rem_s, 1);
                        if rem_v >= den_s then
                            rem_s  <= rem_v - den_s;
                            quot_s <= shift_left(quot_s, 1) or to_unsigned(1, 32);
                        else
                            rem_s  <= rem_v;
                            quot_s <= shift_left(quot_s, 1);
                        end if;
                        if cnt = N_DIV - 1 then
                            state <= S_FINISH;
                        else
                            cnt <= cnt + 1;
                        end if;

                    when S_FINISH =>
                        -- 13 iteraciones acumulan resultado en bits (12 downto 0)
                        q16 := signed(resize(quot_s(12 downto 0), 16));
                        if neg_s = '1' then
                            result <= -q16;
                        else
                            result <= q16;
                        end if;
                        done_r <= '1';
                        flag   <= '0';
                        state  <= S_IDLE;
                end case;
            end if;
        end if;
    end process;

    quot_out <= std_logic_vector(result);
    done     <= done_r;
end rtl;