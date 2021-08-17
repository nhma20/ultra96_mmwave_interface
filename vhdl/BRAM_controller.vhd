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
        i_Clk     	:   in  STD_LOGIC;
        --rst     	:   in  STD_LOGIC;

        --  Data in ports:
        data_in     :   in  STD_LOGIC_VECTOR(127 downto 0);
        irq_in      :   in  STD_LOGIC;
        i_num_points:   in std_logic_vector(4 downto 0);
        o_address   :   out std_logic_vector(4 downto 0);
        
        --  BRAM ports:
        bram_addr   :   out STD_LOGIC_VECTOR(31 downto 0);
        bram_dout   :   out STD_LOGIC_VECTOR(31 downto 0);
        bram_en     :   out STD_LOGIC;
        bram_wr     :   out STD_LOGIC_VECTOR(3 downto 0)

        --  Debug ports:
        
        );
end BRAM_controller;

architecture Behavioral of BRAM_controller is
    signal ena_shift_reg    :   std_logic_vector(1 downto 0) := "00";
    signal s_address_out    :   std_logic_vector(4 downto 0) := "00000";
    signal s_bram_wr        :   std_logic_vector(3 downto 0) := "0000";
    signal s_data_out       :   std_logic_vector(127 downto 0) := (others => '0');
begin

    ------------------------------------------------------------------------------
    fill_BRAM: process(i_Clk, irq_in, data_in)
    ------------------------------------------------------------------------------
    -- Reads data from RAM and stores it in BRAM.
    -- Only x-values are stored since BRAM is limited to 32 bit data width
    ------------------------------------------------------------------------------
     constant rising    :   std_logic_vector(1 downto 0) := "01";
     variable address_var:  std_logic_vector(4 downto 0) := "00000";
     constant one       :   std_logic_vector(4 downto 0) := "00001";
     variable reading   :   std_logic := '0';
     variable read_cnt  :   integer range 0 to 32;
     variable wait_cnt  :   integer range 0 to 20;
    ------------------------------------------------------------------------------
    begin
        if rising_edge(i_Clk) then
            if ena_shift_reg = rising then
                reading := '1';
                address_var := "00000";
            end if;
            if reading = '1' then
                if wait_cnt = 3 then
                    if read_cnt < to_integer(unsigned(i_num_points)) then
                        s_data_out <= data_in;
                    else 
                        s_data_out <= (others => '0');
                    end if;
                    wait_cnt := wait_cnt + 1;
                elsif wait_cnt = 4 then
                    s_bram_wr <= "1111";
                    wait_cnt := wait_cnt + 1;
                elsif wait_cnt = 6 then
                    s_bram_wr <= "0000";
                    wait_cnt := wait_cnt + 1;
                elsif wait_cnt = 7 then
                    wait_cnt := 0;
                    read_cnt := read_cnt + 1;
                    address_var := std_logic_vector(unsigned(address_var) + unsigned(one));
                else
                    wait_cnt := wait_cnt + 1;
                end if;
                if read_cnt = 32 then
                    reading := '0';
                    read_cnt := 0;
                end if;
            end if;
            s_address_out <= address_var;
        end if;
    ------------------------------------------------------------------------------
    end process fill_BRAM;
    ------------------------------------------------------------------------------



    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(irq_in, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of irq_in and i_new_set.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            ena_shift_reg <= ena_shift_reg(0) & irq_in;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------


    o_address               <=  s_address_out;
    bram_addr(31 downto 7)  <=  (30 => '1', others => '0');
    bram_addr(6 downto 0)   <=  s_address_out & "00";
    bram_en                 <=  '1';
    bram_dout               <=  s_data_out(127 downto 96); 
    bram_wr                 <=  s_bram_wr;

end Behavioral;
