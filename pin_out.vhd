----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 07/12/2021 12:40:57 PM
-- Design Name: 
-- Module Name: high_pin - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity high_pin is
    Port ( clk_in : in STD_LOGIC;
           pin_out : out STD_LOGIC := '0');
end high_pin;

architecture Behavioral of high_pin is

    signal pin_out_signal    : STD_LOGIC :=  '0';

begin

process(clk_in)
    variable counter : integer := 0;
begin
    if rising_edge(clk_in) then
        if counter < 100000000 then --500000
            counter := counter + 1;
        else
            pin_out_signal <= NOT pin_out_signal;
            counter := 0;
        end if;
    end if;
end process;

pin_out <= pin_out_signal;

end Behavioral;