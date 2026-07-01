-- =============================================================
--  cordic_atan2.vhd  (Opcion A: puertos STD_LOGIC_VECTOR)
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;

entity cordic_atan2 is
    Port (
        clk   : in  std_logic;
        rst   : in  std_logic;
        start : in  std_logic;
        x_in  : in  std_logic_vector(15 downto 0);
        y_in  : in  std_logic_vector(15 downto 0);
        angle : out std_logic_vector(15 downto 0);
        done  : out std_logic
    );
end cordic_atan2;

architecture vectoring of cordic_atan2 is
    type data_array is array (0 to N_ITER) of signed(15 downto 0);
    signal x_pipe     : data_array;
    signal y_pipe     : data_array;
    signal z_pipe     : data_array;
    signal valid_pipe : std_logic_vector(N_ITER downto 0);
    signal quad_corr  : signed(15 downto 0);

    constant PI_POS : signed(15 downto 0) := to_signed(-25736, 16);
    constant PI_NEG : signed(15 downto 0) := to_signed( 25736, 16);

    -- Senales internas signed para operar
    signal x_in_s : signed(15 downto 0);
    signal y_in_s : signed(15 downto 0);
begin

    -- Conversion de entradas SLV -> signed
    x_in_s <= signed(x_in);
    y_in_s <= signed(y_in);

    process(clk, rst)
        variable x_scaled : signed(15 downto 0);
        variable y_scaled : signed(15 downto 0);
    begin
        if rst = '1' then
            x_pipe(0)     <= (others => '0');
            y_pipe(0)     <= (others => '0');
            z_pipe(0)     <= (others => '0');
            valid_pipe(0) <= '0';
            quad_corr     <= (others => '0');
        elsif rising_edge(clk) then
		  
				if start = '1' then
					 valid_pipe(0) <= '1';
					 z_pipe(0)     <= (others => '0');
					 x_scaled := shift_right(x_in_s, 1);
					 y_scaled := shift_right(y_in_s, 1);

					 if x_in_s >= 0 then
						  -- Cuadrantes I y IV: sin corrección
						  x_pipe(0) <= x_scaled;
						  y_pipe(0) <= y_scaled;
						  quad_corr <= to_signed(0, 16);

					 elsif y_in_s >= 0 then
						  -- Cuadrante II: x<0, y>=0 → sumar +π
						  x_pipe(0) <= -x_scaled;
						  y_pipe(0) <=  y_scaled;   -- mantener signo de y
						  quad_corr <= to_signed(25736, 16);   -- +π

					 else
						  -- Cuadrante III: x<0, y<0 → restar -π
						  x_pipe(0) <= -x_scaled;
						  y_pipe(0) <=  y_scaled;   -- mantener signo de y (negativo)
						  quad_corr <= to_signed(-25736, 16);  -- -π
					 end if;

				else
					 valid_pipe(0) <= '0';
				end if;
				
        end if;
    end process;

    GEN_VEC: for i in 0 to N_ITER-1 generate
        process(clk, rst)
            variable x_sh : signed(15 downto 0);
            variable y_sh : signed(15 downto 0);
        begin
            if rst = '1' then
                x_pipe(i+1)     <= (others => '0');
                y_pipe(i+1)     <= (others => '0');
                z_pipe(i+1)     <= (others => '0');
                valid_pipe(i+1) <= '0';
            elsif rising_edge(clk) then
                valid_pipe(i+1) <= valid_pipe(i);
                x_sh := shift_right(x_pipe(i), i);
                y_sh := shift_right(y_pipe(i), i);
                if y_pipe(i) >= 0 then
                    x_pipe(i+1) <= x_pipe(i) + y_sh;
                    y_pipe(i+1) <= y_pipe(i) - x_sh;
                    z_pipe(i+1) <= z_pipe(i) + ATAN_TABLE(i);
                else
                    x_pipe(i+1) <= x_pipe(i) - y_sh;
                    y_pipe(i+1) <= y_pipe(i) + x_sh;
                    z_pipe(i+1) <= z_pipe(i) - ATAN_TABLE(i);
                end if;
            end if;
        end process;
    end generate GEN_VEC;

    -- Conversion signed -> SLV en la salida
    angle <= std_logic_vector(z_pipe(N_ITER) + quad_corr);
    done  <= valid_pipe(N_ITER);

end vectoring;