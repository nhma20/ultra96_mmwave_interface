----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/12/2021 11:24:27 AM
-- Design Name: 
-- Module Name: extrema_finder - Behavioral
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

entity extrema_finder is

    GENERIC(
       log_2_num_samples_min1   : INTEGER := 8);  --2^(log_2_num_samples_min1 +1) = number of samples pr period

    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in STD_LOGIC;
           i_clr        : in STD_LOGIC;
           i_data_in    : in STD_LOGIC_VECTOR (11 downto 0);
           i_sample_n   : in STD_LOGIC_VECTOR (log_2_num_samples_min1 downto 0);
           i_next_sample: in STD_LOGIC;
           o_largest    : out STD_LOGIC_VECTOR (11 downto 0); 
           o_largest_n  : out STD_LOGIC_VECTOR (log_2_num_samples_min1 downto 0);
           o_smallest   : out STD_LOGIC_VECTOR (11 downto 0); 
           o_smallest_n : out STD_LOGIC_VECTOR (log_2_num_samples_min1 downto 0)
           );
end extrema_finder;


architecture Behavioral of extrema_finder is
    type   STATE_TYPE       is  (s_clr, s_data_in);    --  add states here
    signal current_state    : STATE_TYPE  :=  s_clr;
    signal next_shift_reg   : std_logic_vector(1 downto 0) := "00";
    signal clr_shift_reg    : std_logic_vector(1 downto 0) := "00";
    constant rising         : std_logic_vector := "01";
    signal sig_largest_n    : std_logic_vector(log_2_num_samples_min1 downto 0) := (others => '0');
    signal sig_largest      : unsigned(11 downto 0) := (others => '0');
    signal sig_smallest_n   : std_logic_vector(log_2_num_samples_min1 downto 0) := (others => '0');
    signal sig_smallest     : unsigned(11 downto 0) := (others => '1');

begin

    ------------------------------------------------------------------------------
    extrema_detector  :   process(current_state, i_next_sample, i_Clk, i_clr, next_shift_reg, clr_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Finds extrema for one period, until cleared
    ------------------------------------------------------------------------------  
    begin
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------CLEAR--------------------------------          
            when s_clr =>
                sig_largest_n <= (others => '0');
                sig_largest <= (others => '0');
                sig_smallest_n <= (others => '0');
                sig_smallest <= (others => '1');
                current_state <= s_data_in;
    ----------------------------------------DATA IN-------------------------------                
            when s_data_in =>
                if clr_shift_reg = rising then
                    current_state <= s_clr;
                elsif next_shift_reg = rising then
                    if unsigned(i_data_in) > sig_largest then
                        sig_largest <= unsigned(i_data_in);
                        sig_largest_n <= i_sample_n;
                    end if;
                    if unsigned(i_data_in) < sig_smallest then
                        sig_smallest <= unsigned(i_data_in);
                        sig_smallest_n <= i_sample_n;
                    end if;        
                else
                    current_state <= s_data_in;
                end if;     
    ---------------------------------------OTHER---------------------------------  
            when others =>
                null;
        end case;
    -----------------------------------------------------------------------------   
        if (i_Rst = '1') then
            current_state   <=  s_clr;          -- Reset state
        end if;
    -----------------------------------------------------------------------------      
    end if;
    ------------------------------------------------------------------------------
    end process extrema_detector;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_next_sample, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_next_sample.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            next_shift_reg <= next_shift_reg(0) & i_next_sample;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------
    
    ------------------------------------------------------------------------------
    clr_shift_reg_process :   process(i_Clr, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_Clr
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            clr_shift_reg <= clr_shift_reg(0) & i_clr;
        end if;
    ------------------------------------------------------------------------------
    end process clr_shift_reg_process;
    ------------------------------------------------------------------------------
    
    
    
o_largest <= std_logic_vector(sig_largest);
o_largest_n <= sig_largest_n;

o_smallest <= std_logic_vector(sig_smallest);
o_smallest_n <= sig_smallest_n;

end Behavioral;


