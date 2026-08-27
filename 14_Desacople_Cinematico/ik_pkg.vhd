-- =============================================================
--  ik_pkg.vhd
--  Paquete compartido — Cinemática Inversa 2R
--  Formato Q2.13:  valor_real = raw / 8192
--  Rango:  ±3.9999 (útil: ±1.0 para ángulos en radianes/π)
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

-- Re-exportar cordic_pkg para que ik_pkg sea el único use
use work.cordic_pkg.ALL;

package ik_pkg is

    -- --------------------------------------------------------
    --  Parámetros globales (heredados de cordic_pkg)
    -- --------------------------------------------------------
    constant IK_BITS : integer := 16;   -- ancho de palabra Q2.13
    constant IK_FRAC : integer := 13;   -- bits fraccionarios

    -- --------------------------------------------------------
    --  Constantes matemáticas en Q2.13
    --    Q2.13 = round(valor_real * 2^13)
    -- --------------------------------------------------------
    -- PI = 3.14159...  → 3.14159 * 8192 = 25736
    constant PI_Q13      : signed(15 downto 0) := to_signed( 25736, 16);
    -- PI/2 = 1.5708    → 1.5708  * 8192 = 12868
    constant PI2_Q13     : signed(15 downto 0) := to_signed( 12868, 16);
    -- 2·π  = 6.2832    → no cabe en Q2.13 — usar con cuidado (overflow)
    -- 1.0              → 8192
    constant ONE_Q13     : signed(15 downto 0) := to_signed(  8192, 16);
    -- 0.5              → 4096
    constant HALF_Q13    : signed(15 downto 0) := to_signed(  4096, 16);

    -- --------------------------------------------------------
    --  Función auxiliar: producto Q2.13 × Q2.13 → Q2.13
    --  Multiplica dos valores Q2.13 y devuelve Q2.13
    --  (truncamiento aritmético, sin redondeo)
    -- --------------------------------------------------------
    function mul_q13(a, b : signed(15 downto 0)) return signed;

    -- --------------------------------------------------------
    --  Función auxiliar: valor absoluto de signed(15:0)
    -- --------------------------------------------------------
    function abs_q13(a : signed(15 downto 0)) return signed;

end package ik_pkg;

package body ik_pkg is

    function mul_q13(a, b : signed(15 downto 0)) return signed is
        variable p : signed(31 downto 0);
    begin
        p := a * b;                         -- producto 32 bits Q4.26
        return p(28 downto 13);             -- tomar bits [28:13] → Q2.13
    end function;

    function abs_q13(a : signed(15 downto 0)) return signed is
    begin
        if a(15) = '1' then
            return -a;
        else
            return a;
        end if;
    end function;

end package body ik_pkg;
