-- =============================================================
--  geom_pkg.vhd
--  Paso 0 — Cinematica Directa por METODO GEOMETRICO
--  Paquete de constantes de eslabon + funcion de reduccion angular
--
--  Reutiliza cordic_pkg (tabla atan, K) e ik_pkg (mul_q13, PI_Q13)
--  Formato Q2.13:  valor_real = raw / 8192
--
--  Longitudes del brazo (segun plano de ejes):
--    L1 = 5   cm = 0.05  m -> raw = round(0.05  * 8192) =  410
--    L2 = 10.7cm = 0.107 m -> raw = round(0.107 * 8192) =  877
--    L3 = 13  cm = 0.13  m -> raw = round(0.13  * 8192) = 1065
--    L4+L5 = 18cm = 0.18  m -> raw = round(0.18  * 8192) = 1475
--    (L4 y L5 se suman porque theta5 no cambia la posicion,
--     solo el roll del gripper — ver justificacion geometrica)
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
    --  Longitudes de eslabon en Q2.13 (metros)
    -- ---------------------------------------------------------
    constant L1_Q13  : signed(15 downto 0) := to_signed(  410, 16);  -- 0.05  m
    constant L2_Q13  : signed(15 downto 0) := to_signed(  877, 16);  -- 0.107 m
    constant L3_Q13  : signed(15 downto 0) := to_signed( 1065, 16);  -- 0.13  m
    constant L45_Q13 : signed(15 downto 0) := to_signed( 1475, 16);  -- L4+L5 = 0.18 m

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