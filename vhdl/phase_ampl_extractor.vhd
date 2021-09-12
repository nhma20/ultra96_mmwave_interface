----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/12/2021 04:27:57 PM
-- Design Name: 
-- Module Name: phase_ampl_extractor - Behavioral
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

entity phase_ampl_extractor is

    GENERIC(
       log_2_num_samples_min1   : INTEGER := 9;  --2^(log_2_num_samples_min1 +1) = number of samples pr period
       NUM_SAMPLES              : INTEGER := 500 -- samples pr period
       );

    Port ( -- channel 0
           i_ch0_largest    : in STD_LOGIC_VECTOR (11 downto 0);
           i_ch0_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch0_smallest   : in STD_LOGIC_VECTOR (11 downto 0);
           i_ch0_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 1
           i_ch1_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch1_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch1_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch1_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 2
           i_ch2_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch2_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch2_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch2_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 3
           i_ch3_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch3_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch3_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch3_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 4
           i_ch4_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch4_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch4_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch4_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 5
           i_ch5_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch5_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch5_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch5_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 6
           i_ch6_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch6_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch6_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch6_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 7
           i_ch7_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch7_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch7_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch7_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 8
           i_ch8_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch8_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch8_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch8_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 9
           i_ch9_largest    : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch9_largest_n  : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch9_smallest   : in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch9_smallest_n : in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 10
           i_ch10_largest    :in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch10_largest_n  :in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch10_smallest   :in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch10_smallest_n :in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           -- channel 11
           i_ch11_largest    :in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch11_largest_n  :in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           i_ch11_smallest   :in STD_LOGIC_VECTOR (11 downto 0);                    
           i_ch11_smallest_n :in STD_LOGIC_VECTOR (log_2_num_samples_min1-1 downto 0);
           
           -- outputs
           o_ch0_ampl        : out std_logic_vector(11 downto 0);
           o_ch0_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch1_ampl        : out std_logic_vector(11 downto 0);
           o_ch1_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch2_ampl        : out std_logic_vector(11 downto 0);
           o_ch2_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch3_ampl        : out std_logic_vector(11 downto 0);
           o_ch3_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch4_ampl        : out std_logic_vector(11 downto 0);
           o_ch4_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch5_ampl        : out std_logic_vector(11 downto 0);
           o_ch5_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch6_ampl        : out std_logic_vector(11 downto 0);
           o_ch6_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch7_ampl        : out std_logic_vector(11 downto 0);
           o_ch7_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch8_ampl        : out std_logic_vector(11 downto 0);
           o_ch8_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch9_ampl        : out std_logic_vector(11 downto 0);
           o_ch9_phase       : out std_logic_vector(log_2_num_samples_min1 downto 0); 
           
           o_ch10_ampl       : out std_logic_vector(11 downto 0);
           o_ch10_phase      : out std_logic_vector(log_2_num_samples_min1 downto 0); 
          
           o_ch11_ampl       : out std_logic_vector(11 downto 0);
           o_ch11_phase      : out std_logic_vector(log_2_num_samples_min1 downto 0)
           );
           
end phase_ampl_extractor;

architecture Behavioral of phase_ampl_extractor is
    
    signal ch0_ampl    : std_logic_vector(11 downto 0);
    signal ch1_ampl    : std_logic_vector(11 downto 0);
    signal ch2_ampl    : std_logic_vector(11 downto 0);
    signal ch3_ampl    : std_logic_vector(11 downto 0);
    signal ch4_ampl    : std_logic_vector(11 downto 0);
    signal ch5_ampl    : std_logic_vector(11 downto 0);
    signal ch6_ampl    : std_logic_vector(11 downto 0);
    signal ch7_ampl    : std_logic_vector(11 downto 0);
    signal ch8_ampl    : std_logic_vector(11 downto 0);
    signal ch9_ampl    : std_logic_vector(11 downto 0);
    signal ch10_ampl   : std_logic_vector(11 downto 0);
    signal ch11_ampl   : std_logic_vector(11 downto 0);

begin
    
    ch0_ampl <= std_logic_vector(unsigned(unsigned(i_ch0_largest) - unsigned(i_ch0_smallest)));
    o_ch0_ampl  <= '0' & ch0_ampl(11 downto 1);
    
    ch1_ampl <= std_logic_vector(unsigned(unsigned(i_ch1_largest) - unsigned(i_ch1_smallest)));
    o_ch1_ampl  <= '0' & ch1_ampl(11 downto 1); 
    
    ch2_ampl <= std_logic_vector(unsigned(unsigned(i_ch2_largest) - unsigned(i_ch2_smallest)));
    o_ch2_ampl  <= '0' & ch2_ampl(11 downto 1);
    
    ch3_ampl <= std_logic_vector(unsigned(unsigned(i_ch3_largest) - unsigned(i_ch3_smallest)));
    o_ch3_ampl  <= '0' & ch3_ampl(11 downto 1);
    
    ch4_ampl <= std_logic_vector(unsigned(unsigned(i_ch4_largest) - unsigned(i_ch4_smallest)));
    o_ch4_ampl  <= '0' & ch4_ampl(11 downto 1);
    
    ch5_ampl <= std_logic_vector(unsigned(unsigned(i_ch5_largest) - unsigned(i_ch5_smallest)));
    o_ch5_ampl  <= '0' & ch5_ampl(11 downto 1);
    
    ch6_ampl <= std_logic_vector(unsigned(unsigned(i_ch6_largest) - unsigned(i_ch6_smallest)));
    o_ch6_ampl  <= '0' & ch6_ampl(11 downto 1);
    
    ch7_ampl <= std_logic_vector(unsigned(unsigned(i_ch7_largest) - unsigned(i_ch7_smallest)));
    o_ch7_ampl  <= '0' & ch7_ampl(11 downto 1);
    
    ch8_ampl <= std_logic_vector(unsigned(unsigned(i_ch8_largest) - unsigned(i_ch8_smallest)));
    o_ch8_ampl  <= '0' & ch8_ampl(11 downto 1);
    
    ch9_ampl <= std_logic_vector(unsigned(unsigned(i_ch9_largest) - unsigned(i_ch9_smallest)));
    o_ch9_ampl  <= '0' & ch9_ampl(11 downto 1);
    
    ch10_ampl <= std_logic_vector(unsigned(unsigned(i_ch10_largest) - unsigned(i_ch10_smallest)));
    o_ch10_ampl <= '0' & ch10_ampl(11 downto 1);
    
    ch11_ampl <= std_logic_vector(unsigned(unsigned(i_ch11_largest) - unsigned(i_ch11_smallest)));
    o_ch11_ampl <= '0' & ch11_ampl(11 downto 1);
    
    o_ch0_phase  <= (others => '0'); -- reference phase
    o_ch1_phase  <= std_logic_vector(unsigned('0' & i_ch1_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch1_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch1_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch2_phase  <= std_logic_vector(unsigned('0' & i_ch2_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch2_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch2_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch3_phase  <= std_logic_vector(unsigned('0' & i_ch3_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch3_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch3_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch4_phase  <= std_logic_vector(unsigned('0' & i_ch4_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch4_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch4_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch5_phase  <= std_logic_vector(unsigned('0' & i_ch5_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch5_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch5_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch6_phase  <= std_logic_vector(unsigned('0' & i_ch6_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch6_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch6_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch7_phase  <= std_logic_vector(unsigned('0' & i_ch7_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch7_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch7_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch8_phase  <= std_logic_vector(unsigned('0' & i_ch8_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch8_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch8_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch9_phase  <= std_logic_vector(unsigned('0' & i_ch9_largest_n)  - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch9_largest_n)  >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch9_largest_n)  + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch10_phase <= std_logic_vector(unsigned('0' & i_ch10_largest_n) - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch10_largest_n) >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch10_largest_n) + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    o_ch11_phase <= std_logic_vector(unsigned('0' & i_ch11_largest_n) - unsigned('0' & i_ch0_largest_n)) when (unsigned(i_ch11_largest_n) >= unsigned(i_ch0_largest_n)) else std_logic_vector((unsigned('0' & i_ch11_largest_n) + unsigned(to_unsigned(NUM_SAMPLES,log_2_num_samples_min1))) - unsigned('0' & i_ch0_largest_n));
    
end Behavioral;
