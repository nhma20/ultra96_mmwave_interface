----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/12/2021 01:50:23 PM
-- Design Name: 
-- Module Name: phase_amp_extract - Behavioral
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

entity extrema_finder_ctrl is

    GENERIC(
       log_2_num_samples_min1   : INTEGER := 8;  --2^(log_2_num_samples_min1 +1) = number of samples pr period
       SAMPLE_CLK_DIV           : INTEGER := 1000; -- clks per sample
       NUM_SAMPLES              : INTEGER := 500 -- samples pr period
       );

    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in std_logic;
           o_sample_n   : out STD_LOGIC_VECTOR (log_2_num_samples_min1 downto 0);
           o_next_sample: out STD_LOGIC;
           o_clr        : out STD_LOGIC
           );
end extrema_finder_ctrl;


architecture Behavioral of extrema_finder_ctrl is
    type   STATE_TYPE       is  (s_rst, s_clr_extrema_finder, s_wait, s_sample, s_finished, s_shift_ram, s_wait_EF);    --  add states here
    signal current_state    : STATE_TYPE  :=  s_rst;
    signal clk_div_cnt      : integer := 0;
    signal next_sample      : std_logic := '1';
    signal sample_cnt       : integer := 0;
    signal shift_RAM        : std_logic := '0';
    signal clr              : std_logic := '0';
    constant rising         : std_logic_vector(1 downto 0) := "01";

begin

    ------------------------------------------------------------------------------
    extrema_detector_ctrl  :   process(current_state, i_Clk)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Controls extrema finder module(s)
    ------------------------------------------------------------------------------  
    begin
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                clk_div_cnt <= 0;
                next_sample <= '0';
                sample_cnt <= 0;
                shift_RAM <= '0';
                clr <= '0';                
                if i_Rst = '0' then
                    current_state <= s_wait;
                else
                    current_state <= s_rst;
                end if;
    -------------------------------------WAITING-------------------------------                
            when s_wait =>
                if clk_div_cnt < SAMPLE_CLK_DIV-1 then
                    clk_div_cnt <= clk_div_cnt + 1;
                    next_sample <= '0';
                    current_state <= s_wait;
                else 
                    next_sample <= '1';
                    current_state <= s_sample;
                end if;     
    -------------------------------------SAMPLING-------------------------------                
            when s_sample =>
                next_sample <= '0';
                if sample_cnt = NUM_SAMPLES-1 then
                    current_state <= s_wait_EF;
                else
                    clk_div_cnt <= 0;
                    sample_cnt <= sample_cnt + 1;
                    current_state <= s_wait;
                end if;
    ---------------------------------WAIT FOR EF--------------------------------                
            when s_wait_EF =>
                -- do nothing, give EF 1 clk cycle before 
                -- writing their output to shift RAM
                current_state <= s_shift_ram; 
    -------------------------------------SHIFT RAM------------------------------                
            when s_shift_ram =>
                shift_RAM <= '1';
                current_state <= s_clr_extrema_finder;
    ----------------------------CLEAR EXTREMA FINDER----------------------------                
            when s_clr_extrema_finder =>
                clr <= '1';
                current_state <= s_rst;   
    ---------------------------------------OTHER--------------------------------  
            when others =>
                null;
        end case;
    ----------------------------------------------------------------------------   
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
        end if;
    ----------------------------------------------------------------------------
    end if;
    ----------------------------------------------------------------------------
    end process extrema_detector_ctrl;
    ----------------------------------------------------------------------------
    
    
o_sample_n <= std_logic_vector(to_unsigned(sample_cnt, o_sample_n'length));
o_clr <= clr;
o_next_sample <= next_sample;


end Behavioral;


