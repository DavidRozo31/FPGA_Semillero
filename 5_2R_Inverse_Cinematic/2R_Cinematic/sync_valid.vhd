-- =============================================================
--  sync_valid.vhd
--  Sincronizador de dos senales valid con latencias distintas.
--  Retiene cada valid hasta que ambos esten activos,
--  luego emite un pulso de 1 ciclo en valid_out y se autoreset.
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity sync_valid is
    Port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        valid_a   : in  std_logic;
        valid_b   : in  std_logic;
        valid_out : out std_logic
    );
end sync_valid;

architecture rtl of sync_valid is
    signal latch_a : std_logic := '0';
    signal latch_b : std_logic := '0';
begin
    process(clk, rst)
    begin
        if rst = '1' then
            latch_a   <= '0';
            latch_b   <= '0';
            valid_out <= '0';
        elsif rising_edge(clk) then
            -- Retener cada valid cuando llega
            if valid_a = '1' then
                latch_a <= '1';
            end if;
            if valid_b = '1' then
                latch_b <= '1';
            end if;

            -- Cuando ambos estan retenidos emitir pulso y limpiar
            if (latch_a = '1' or valid_a = '1') and
               (latch_b = '1' or valid_b = '1') then
                valid_out <= '1';
                latch_a   <= '0';
                latch_b   <= '0';
            else
                valid_out <= '0';
            end if;
        end if;
    end process;
end rtl;