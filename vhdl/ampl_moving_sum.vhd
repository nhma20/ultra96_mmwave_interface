----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/12/2021 07:02:47 PM
-- Design Name: 
-- Module Name: ampl_moving_sum - Behavioral
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

entity ampl_moving_sum is

    GENERIC(
       --log_2_amplitude_min1   : INTEGER := ;  --2^(log_2_num_samples_min1 +1) = number of samples pr period
       --sum_size                 : INTEGER := 16; -- 12 + roof(log2(window_size))
       window_size              : INTEGER := 10 -- numbers to average over
       );

    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in STD_LOGIC;
           i_ena        : in STD_LOGIC;
           i_ampl       : in std_logic_vector(11 downto 0);
           o_sum        : out STD_LOGIC_VECTOR (31 downto 0)
           );
end ampl_moving_sum;

architecture Behavioral of ampl_moving_sum is
    type   STATE_TYPE       is  (s_rst, s_data_in, s_wait);    --  add states here
    signal current_state    : STATE_TYPE  :=  s_rst;
    signal next_shift_reg   : std_logic_vector(1 downto 0) := "00";
    constant rising         : std_logic_vector := "01";
    signal data             : std_logic_vector((window_size*i_ampl'length)-1 downto 0) := (others => '0');
    signal data_rdy         : std_logic := '0';

begin

    ------------------------------------------------------------------------------
    extrema_detector  :   process(current_state, i_ena, i_Clk, next_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Finds extrema for one period, until cleared
    ------------------------------------------------------------------------------  
    variable sum    : integer := 0;
    
    begin    
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------CLEAR--------------------------------          
            when s_rst =>
                data <= (others => '0');
                data_rdy <= '0';
                current_state <= s_data_in;
    ----------------------------------------DATA IN-------------------------------                
            when s_data_in =>
                data_rdy <= '0';
                if next_shift_reg = rising then
                    data <= i_ampl & data((window_size*i_ampl'length)-1 downto i_ampl'length); 
                    for i in 0 to window_size-1 loop
                        sum := sum + to_integer(unsigned(data((i_ampl'length-1) + i_ampl'length*i downto i_ampl'length*i)));
                    end loop; 
                    o_sum <= std_logic_vector(to_unsigned(sum,o_sum'length));
                    sum := 0;
                    current_state <= s_wait;
                else
                    current_state <= s_data_in;
                end if;   
    -----------------------------------------CLEAR--------------------------------          
            when s_wait =>
                data_rdy <= '1';
                current_state <= s_data_in;
    ---------------------------------------OTHER---------------------------------  
            when others =>
                null;
        end case;
    -----------------------------------------------------------------------------   
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
        end if;
    -----------------------------------------------------------------------------      
    end if;
    ------------------------------------------------------------------------------
    end process extrema_detector;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_ena, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_next_sample.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            next_shift_reg <= next_shift_reg(0) & i_ena;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------
    
    
    
    
    
end Behavioral;
