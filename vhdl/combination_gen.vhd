----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 09/06/2021 03:14:17 PM
-- Design Name: 
-- Module Name: combination_gen - Behavioral
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

entity combination_gen is

    GENERIC(
        max_num_points  : INTEGER := 7);  --number of data words the memory can store

    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in STD_LOGIC;
           i_num_points : in STD_LOGIC_VECTOR (4 downto 0);
           i_next       : in STD_LOGIC;
           o_not_used   : out STD_LOGIC_VECTOR (max_num_points downto 0);
           o_s2_phase_0    : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 0 deg
           o_s3_phase_0    : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 3-phase 0 deg
           o_s3_phase_120  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 120 deg
           o_s2_phase_180  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 180 deg
           o_s3_phase_240  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 3-phase 240 deg
           o_data_rdy   : out std_logic
           );
end combination_gen;


architecture Behavioral of combination_gen is
    type   STATE_TYPE       is  (s_rst, s_inc_comb, s_comb_check, s_mask_0, s_mask_1);    --  add states here
    type   mask_0           is  array(max_num_points downto 0) of integer range 0 to 1; -- 0=0deg, 1=180deg
    type   mask_1           is  array(max_num_points downto 0) of integer range 0 to 2; -- 0=0deg, 1=120deg, 2=240deg
    signal current_state    : STATE_TYPE  :=  s_rst;
    signal sig_mask_0       : mask_0 := (others => 0);
    signal sig_mask_1       : mask_1 := (others => 0);
    signal s_data_rdy       : std_logic := '0';  
    --signal one              : std_logic_vector(4 downto 0) := "00001";
    signal next_shift_reg   : std_logic_vector(1 downto 0) := "00";
    signal s_not_used       : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_0        : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_120      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_180      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_240      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal sig_comb_mask    : std_logic_vector(max_num_points downto 0) := (others => '0'); -- '1' use index in ram, '0' do not use index in ram
    --signal sig_mask_0       : std_logic_vector(max_num_points downto 0) := (others => '0');
    --signal sig_mask_1       : std_logic_vector(max_num_points downto 0) := (others => '0');

begin

    ------------------------------------------------------------------------------
    combination_generator  :   process(current_state, i_next, i_num_points, i_Clk, next_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Handles writes to input RAM
    ------------------------------------------------------------------------------
    variable points_stored  : integer := 0;
    variable addr_in        : integer := 0;
    constant ena_rising     : std_logic_vector := "01";
    constant one            : std_logic_vector(max_num_points downto 0) := (0 => '1', others => '0');
    constant fully_incremtd : std_logic_vector(max_num_points downto 0) := (others => '1');
    variable high_cnt       : integer := 0;
    variable comb_mask_sum  : integer := 0;
    variable comb_mask_idx  : integer := 0;

    
    begin
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                if i_Rst = '0' then
                    sig_mask_0 <= (others => 0);
                    sig_mask_1 <= (others => 0);
                    sig_comb_mask <= (0 => '1', others => '0'); -- populate LSB, one before first valid comb
                    current_state <= s_inc_comb;
                else 
                    current_state  <=  s_rst;
                end if;
    ----------------------------------Increment comb_mask-------------------------                
            when s_inc_comb =>
                -- increment mask, or reset if fully incremented
                if sig_comb_mask = fully_incremtd then
                    -- something reset here
                    -- tell search all combs done
                    -- some state transition
                else
                    sig_comb_mask <= std_logic_vector(unsigned(sig_comb_mask) + unsigned(one)); -- increment comb_mask
                    current_state <= s_comb_check;
                end if;          
    ------------------------------------Check comb_mask--------------------------                
            when s_comb_check => 
                if sig_comb_mask(comb_mask_idx) = '1' then -- get bits set in mask
                    comb_mask_sum := comb_mask_sum + 1;
                end if;
                
                comb_mask_idx := comb_mask_idx + 1;
                   
                if comb_mask_idx = max_num_points+1 then
                    if comb_mask_sum mod 2 = 0 then     -- goto 2-phase
                        current_state <= s_mask_0;
                    elsif comb_mask_sum mod 3 = 0 then  -- goto 3-phase
                        current_state <= s_mask_1;
                    else                                -- goto increment s_comb_mask
                        current_state <= s_inc_comb;
                    end if;
                    comb_mask_sum := 0;
                    comb_mask_idx := 0;
                else
                    current_state <= s_comb_check;
                end if;               
    ------------------------------------Increment mask 0--------------------------                
            when s_mask_0 =>
                if next_shift_reg = ena_rising then
                    
                else
                    current_state <= s_mask_0;
                end if;
    ------------------------------------Increment mask 1--------------------------                    
            when s_mask_1 =>
                if next_shift_reg = ena_rising then
                    
                else
                    current_state <= s_mask_1;
                end if;
    ---------------------------------------OTHER---------------------------------  
            when others =>
                null;
        end case;
    -----------------------------------------------------------------------------
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
            points_stored := 0;
            s_data_rdy <= '0';
        end if;
        
    -----------------------------------------------------------------------------
      
        if s_data_rdy = '1' then
            high_cnt := high_cnt + 1;
        end if;
        if high_cnt = 2 then -- find good value?
            s_data_rdy <= '0';
            high_cnt := 0;
        end if;

    end if;
    ------------------------------------------------------------------------------
    end process combination_generator;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_next, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_next and i_new_set.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            next_shift_reg <= next_shift_reg(0) & i_next;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------
    
    
    -- mask outputs, hardcoded to 8 for now, use generate to match generic 'max_num_points'
o_s2_phase_0(0) <= '1' when sig_mask_0(0) = 0 and sig_comb_mask(0) = '1' else '0'; -- 2-phase 0 deg output
o_s2_phase_0(1) <= '1' when sig_mask_0(1) = 0 and sig_comb_mask(1) = '1' else '0';
o_s2_phase_0(2) <= '1' when sig_mask_0(2) = 0 and sig_comb_mask(2) = '1' else '0';
o_s2_phase_0(3) <= '1' when sig_mask_0(3) = 0 and sig_comb_mask(3) = '1' else '0';
o_s2_phase_0(4) <= '1' when sig_mask_0(4) = 0 and sig_comb_mask(4) = '1' else '0';
o_s2_phase_0(5) <= '1' when sig_mask_0(5) = 0 and sig_comb_mask(5) = '1' else '0';
o_s2_phase_0(6) <= '1' when sig_mask_0(6) = 0 and sig_comb_mask(6) = '1' else '0';
o_s2_phase_0(7) <= '1' when sig_mask_0(7) = 0 and sig_comb_mask(7) = '1' else '0';

o_s2_phase_180(0) <= '1' when sig_mask_0(0) = 1 and sig_comb_mask(0) = '1' else '0'; -- 2-phase 180 deg output
o_s2_phase_180(1) <= '1' when sig_mask_0(1) = 1 and sig_comb_mask(1) = '1' else '0';
o_s2_phase_180(2) <= '1' when sig_mask_0(2) = 1 and sig_comb_mask(2) = '1' else '0';
o_s2_phase_180(3) <= '1' when sig_mask_0(3) = 1 and sig_comb_mask(3) = '1' else '0';
o_s2_phase_180(4) <= '1' when sig_mask_0(4) = 1 and sig_comb_mask(4) = '1' else '0';
o_s2_phase_180(5) <= '1' when sig_mask_0(5) = 1 and sig_comb_mask(5) = '1' else '0';
o_s2_phase_180(6) <= '1' when sig_mask_0(6) = 1 and sig_comb_mask(6) = '1' else '0';
o_s2_phase_180(7) <= '1' when sig_mask_0(7) = 1 and sig_comb_mask(7) = '1' else '0';


o_s3_phase_0(0) <= '1' when sig_mask_1(0) = 0 and sig_comb_mask(0) = '1' else '0'; -- 3-phase 0 deg output
o_s3_phase_0(1) <= '1' when sig_mask_1(1) = 0 and sig_comb_mask(1) = '1' else '0';
o_s3_phase_0(2) <= '1' when sig_mask_1(2) = 0 and sig_comb_mask(2) = '1' else '0';
o_s3_phase_0(3) <= '1' when sig_mask_1(3) = 0 and sig_comb_mask(3) = '1' else '0';
o_s3_phase_0(4) <= '1' when sig_mask_1(4) = 0 and sig_comb_mask(4) = '1' else '0';
o_s3_phase_0(5) <= '1' when sig_mask_1(5) = 0 and sig_comb_mask(5) = '1' else '0';
o_s3_phase_0(6) <= '1' when sig_mask_1(6) = 0 and sig_comb_mask(6) = '1' else '0';
o_s3_phase_0(7) <= '1' when sig_mask_1(7) = 0 and sig_comb_mask(7) = '1' else '0';

o_s3_phase_120(0) <= '1' when sig_mask_1(0) = 1 and sig_comb_mask(0) = '1' else '0'; -- 3-phase 120 deg output
o_s3_phase_120(1) <= '1' when sig_mask_1(1) = 1 and sig_comb_mask(1) = '1' else '0';
o_s3_phase_120(2) <= '1' when sig_mask_1(2) = 1 and sig_comb_mask(2) = '1' else '0';
o_s3_phase_120(3) <= '1' when sig_mask_1(3) = 1 and sig_comb_mask(3) = '1' else '0';
o_s3_phase_120(4) <= '1' when sig_mask_1(4) = 1 and sig_comb_mask(4) = '1' else '0';
o_s3_phase_120(5) <= '1' when sig_mask_1(5) = 1 and sig_comb_mask(5) = '1' else '0';
o_s3_phase_120(6) <= '1' when sig_mask_1(6) = 1 and sig_comb_mask(6) = '1' else '0';
o_s3_phase_120(7) <= '1' when sig_mask_1(7) = 1 and sig_comb_mask(7) = '1' else '0';

o_s3_phase_240(0) <= '1' when sig_mask_1(0) = 2 and sig_comb_mask(0) = '1' else '0'; -- 3-phase 240 deg output
o_s3_phase_240(1) <= '1' when sig_mask_1(1) = 2 and sig_comb_mask(1) = '1' else '0';
o_s3_phase_240(2) <= '1' when sig_mask_1(2) = 2 and sig_comb_mask(2) = '1' else '0';
o_s3_phase_240(3) <= '1' when sig_mask_1(3) = 2 and sig_comb_mask(3) = '1' else '0';
o_s3_phase_240(4) <= '1' when sig_mask_1(4) = 2 and sig_comb_mask(4) = '1' else '0';
o_s3_phase_240(5) <= '1' when sig_mask_1(5) = 2 and sig_comb_mask(5) = '1' else '0';
o_s3_phase_240(6) <= '1' when sig_mask_1(6) = 2 and sig_comb_mask(6) = '1' else '0';
o_s3_phase_240(7) <= '1' when sig_mask_1(7) = 2 and sig_comb_mask(7) = '1' else '0';


    
end Behavioral;
