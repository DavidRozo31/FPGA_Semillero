-- =============================================================
--  geom_pkg.vhd
--  Paso 0 — Cinematica Directa por METODO GEOMETRICO (brazo de 6 GDL)
--  Paquete de constantes de eslabon + funcion de reduccion angular
--
--  ACTUALIZADO: brazo de 6 GDL real (tabla DH corregida por el
--  profesor, "regla 4" -- ver leccion 12). Ya NO es el brazo de
--  5 articulaciones + roll de gripper de la leccion 11.
--
--  Reutiliza cordic_pkg (tabla atan, K) e ik_pkg (mul_q13, PI_Q13)
--  Formato Q2.13:  valor_real = raw / 8192
--
--  Longitudes del brazo (segun tabla DH nueva):
--    L1 = 6.5cm = 0.065 m -> raw = round(0.065 * 8192) =  532
--    L2 = 10.7cm = 0.107 m -> raw = round(0.107 * 8192) =  877
--    Ld4 = L3+L4 = 14.0cm = 0.140 m -> raw = round(0.140 * 8192) = 1147
--    Ld6 = L5+L6 = 17.3cm = 0.173 m -> raw = round(0.173 * 8192) = 1417
--    (Ld4 y Ld6 absorben las distancias que los sistemas 3 y 5 ya no
--     cargan, porque coinciden con los sistemas 2 y 4 -- ver leccion 12)
--
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

use work.cordic_pkg.ALL;
use work.ik_pkg.ALL;

package geom_pkg is

    -- ---------------------------------------------------------
    --  Longitudes de eslabon en Q2.13 (metros) -- brazo de 6 GDL
    -- ---------------------------------------------------------
    constant L1_Q13  : signed(15 downto 0) := to_signed(  532, 16);  -- 0.065 m
    constant L2_Q13  : signed(15 downto 0) := to_signed(  877, 16);  -- 0.107 m
    constant LD4_Q13 : signed(15 downto 0) := to_signed( 1147, 16);  -- Ld4 = L3+L4 = 0.140 m
    constant LD6_Q13 : signed(15 downto 0) := to_signed( 1417, 16);  -- Ld6 = L5+L6 = 0.173 m

    -- 2*pi en Q2.13 (no cabe en 16 bits signed, se usa en 18 bits)
    constant TWO_PI_Q13 : integer := 51472;  -- round(2*pi*8192)

    -- ---------------------------------------------------------
    --  wrap_to_pi: reduce un angulo (acumulado en 18 bits, para
    --  evitar overflow al sumar hasta 3 thetas) al rango [-pi, pi]
    --  requerido por la entrada del CORDIC.
    -- ---------------------------------------------------------
    function wrap_to_pi(a : signed(17 downto 0)) return signed;

end package geom_pkg;

package body geom_pkg is

    function wrap_to_pi(a : signed(17 downto 0)) return signed is
        variable r       : signed(17 downto 0);
        constant PI18    : signed(17 downto 0) := resize(PI_Q13, 18);
        constant TWOPI18 : signed(17 downto 0) := to_signed(TWO_PI_Q13, 18);
    begin
        r := a;
        if r > PI18 then
            r := r - TWOPI18;
        elsif r < -PI18 then
            r := r + TWOPI18;
        end if;
        return r(15 downto 0);
    end function;

end package body geom_pkg;