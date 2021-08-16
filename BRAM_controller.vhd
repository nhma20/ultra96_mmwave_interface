----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 08/03/2021 12:03:56 PM
-- Design Name: 
-- Module Name: BRAM_controller - Behavioral
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
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity BRAM_controller is
        port (
        ----  STD ports:
        --clk     	:   in  STD_LOGIC;
        --rst     	:   in  STD_LOGIC;

        --  Data in ports:
        data_in     :   in  STD_LOGIC_VECTOR(127 downto 0);
        --channel_in  :   in  STD_LOGIC_VECTOR(1 downto 0);
        irq_in      :   in  STD_LOGIC;

        --  BRAM ports:
        bram_addr   :   out STD_LOGIC_VECTOR(31 downto 0);
        bram_dout   :   out STD_LOGIC_VECTOR(31 downto 0);
        bram_en     :   out STD_LOGIC;
        bram_wr     :   out STD_LOGIC_VECTOR(3 downto 0)

        --  Debug ports:
        
        );
end BRAM_controller;

architecture Behavioral of BRAM_controller is
begin
    bram_addr(31 downto 4)  <=  (30 => '1', others => '0');
    bram_addr(3 downto 0)   <=  "0000"; --channel_in & "00";
    bram_en                 <=  '1';
    bram_dout               <=  data_in(127 downto 96); 
    bram_wr                 <=  irq_in & irq_in & irq_in & irq_in;

end Behavioral;
