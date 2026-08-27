library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity Wrist_Center is
    Port (
        clk, rst, start : in  STD_LOGIC;
        dx, dy, dz       : in  STD_LOGIC_VECTOR(15 downto 0);
        rz_x, rz_y, rz_z : in  STD_LOGIC_VECTOR(15 downto 0);
        done : out STD_LOGIC;
        Px_d, Py_d, Pz_d : out STD_LOGIC_VECTOR(15 downto 0)
    );
end Wrist_Center;

architecture struct of Wrist_Center is
    component fp_multiplier
        Port(clk,rst,valid_i: in STD_LOGIC; a_in,b_in: in STD_LOGIC_VECTOR(15 downto 0);
             valid_o: out STD_LOGIC; p_out: out STD_LOGIC_VECTOR(15 downto 0));
    end component;
    component fp_adder
        Port(clk,rst,op,valid_i: in STD_LOGIC; a_in,b_in: in STD_LOGIC_VECTOR(15 downto 0);
             valid_o: out STD_LOGIC; p_out: out STD_LOGIC_VECTOR(15 downto 0));
    end component;

    signal sx, sy, sz : STD_LOGIC_VECTOR(15 downto 0);
    signal ds_x, ds_y, ds_z : STD_LOGIC;
    signal don_x, don_y, don_z : STD_LOGIC;
	 signal l5_plus_l6 : STD_LOGIC_VECTOR(15 downto 0);
begin
    l5_plus_l6 <= x"0589";  -- l5+l6 = 0.173 m = 1417 (l5=0.07, l6=0.103)
    MX: fp_multiplier port map(clk,rst,start,l5_plus_l6,rz_x,ds_x,sx);
    MY: fp_multiplier port map(clk,rst,start,l5_plus_l6,rz_y,ds_y,sy);
    MZ: fp_multiplier port map(clk,rst,start,l5_plus_l6,rz_z,ds_z,sz);

    SUB_X: fp_adder port map(clk,rst,'1',ds_x,dx,sx,don_x,Px_d);
    SUB_Y: fp_adder port map(clk,rst,'1',ds_y,dy,sy,don_y,Py_d);
    SUB_Z: fp_adder port map(clk,rst,'1',ds_z,dz,sz,don_z,Pz_d);

    done <= don_x and don_y and don_z;
end struct;