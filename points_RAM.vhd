----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 08/11/2021 10:59:41 AM
-- Design Name: 
-- Module Name: points_RAM - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: stores detected mmWave points
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

entity points_RAM is

    GENERIC(
        d_width  : INTEGER := 128;    --width of each data word
        size     : INTEGER := 32);  --number of data words the memory can store
        
    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in STD_LOGIC;
           i_data_in    : in STD_LOGIC_VECTOR (127 downto 0);
           i_point_addr : in STD_LOGIC_VECTOR(4 downto 0); -- select one of 32 stored points          
           i_write_ena  : in STD_LOGIC;     
           i_new_set    : in STD_LOGIC;   -- foung magic word, indicates new batch of points   
           o_data_rdy   : out STD_LOGIC;
           o_num_points : out STD_LOGIC_VECTOR(4 downto 0);
           o_data_out   : out STD_LOGIC_VECTOR(127 downto 0)
          );
end points_RAM;

architecture Behavioral of points_RAM is
    TYPE memory IS ARRAY(size-1 DOWNTO 0) OF STD_LOGIC_VECTOR(d_width-1 DOWNTO 0); --data type for memory
    signal ram_0        : memory;                             --memory array
    signal ram_1        : memory;                             --memory array
    signal addr_out     : INTEGER RANGE 0 TO size-1;          --internal address register
    signal ena_shift_reg: std_logic_vector (1 downto 0) := "00";
    signal new_set_shift_reg: std_logic_vector (1 downto 0) := "00";
    signal ram_selector : std_logic := '1';                   -- selects which ram to read from
    type    STATE_TYPE      is  (s_rst, s_ram_0, s_ram_1);    --  add states here
    signal current_state:   STATE_TYPE  :=  s_rst;
    signal s_data_rdy   : std_logic := '0';                   -- 1 clk high when ram can be read after storing
    signal num_points_out: integer := 0;
    

begin

    ------------------------------------------------------------------------------
    ram_in    :   process(current_state, i_write_ena, i_Clk, ena_shift_reg, new_set_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Handles writes to input RAM
    ------------------------------------------------------------------------------
    variable num_points     : integer := 0;
    variable points_stored  : integer := 0;
    variable addr_in        : integer := 0;
    constant ena_rising     : std_logic_vector := "01";
    
    begin
    
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                if new_set_shift_reg = ena_rising then
                    current_state <= s_ram_0;
                else 
                    current_state  <=  s_rst;
                end if;
    ---------------------------------------WRITE RAM 0---------------------------                
            when s_ram_0 =>
                s_data_rdy <= '0';
                if new_set_shift_reg = ena_rising then
                    s_data_rdy <= '1';
                    num_points_out <= points_stored; 
                    points_stored := 0; -- @@@@@@@@@@ does this 'ruin' previous assignment?
                    ram_selector <= '1';
                    current_state <= s_ram_1;
                elsif ena_shift_reg = ena_rising then
                    if points_stored < size-1 then
                        ram_0(points_stored) <= i_data_in;
                        points_stored := points_stored + 1;
                    end if;
                else
                    current_state <= s_ram_0;
                end if;
    ---------------------------------------WRITE RAM 1---------------------------                    
            when s_ram_1 =>
                s_data_rdy <= '0';
                if new_set_shift_reg = ena_rising then
                    s_data_rdy <= '1';
                    num_points_out <= points_stored; 
                    points_stored := 0; -- @@@@@@@@@@ does this 'ruin' previous assignment?
                    ram_selector <= '0';
                    current_state <= s_ram_0;
                elsif ena_shift_reg = ena_rising then
                    if points_stored < size-1 then
                        ram_1(points_stored) <= i_data_in;
                        points_stored := points_stored + 1;
                    end if;
                else
                    current_state <= s_ram_1;
                end if;
    ---------------------------------------OTHER---------------------------------  
            when others =>
                null;
        end case;
    ---------------------------------------W-------------------------------------
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
            s_data_rdy <= '0';
            points_stored := 0;
            num_points := 0;

        else 
            null;
        end if;
    
    end if;
    ------------------------------------------------------------------------------
    end process ram_in;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_write_ena, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_write_ena.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            ena_shift_reg <= ena_shift_reg(0) & i_write_ena;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    new_set_shift_reg_process :   process(i_new_set, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_new_set.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            new_set_shift_reg <= new_set_shift_reg(0) & i_new_set;
        end if;
    ------------------------------------------------------------------------------
    end process new_set_shift_reg_process;
    ------------------------------------------------------------------------------
    
    
    o_data_out <= ram_0(to_integer(unsigned(i_point_addr))) when ram_selector = '0' else ram_1(to_integer(unsigned(i_point_addr)));
    o_num_points <= std_logic_vector(to_unsigned(num_points_out, o_num_points'length));
    o_data_rdy <= s_data_rdy;
    
end Behavioral;
