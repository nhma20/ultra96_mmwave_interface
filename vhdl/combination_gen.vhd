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
-- Description: Generates all combinations of points and phases
--              to be used in the subsequent model-measurement fit
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
           o_s2_phase_0    : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 0 deg
           o_s3_phase_0    : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 3-phase 0 deg
           o_s3_phase_120  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 120 deg
           o_s2_phase_180  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 2-phase 180 deg
           o_s3_phase_240  : out STD_LOGIC_VECTOR (max_num_points downto 0); -- 3-phase 240 deg
           o_set_done      : out std_logic;
           o_data_rdy      : out std_logic
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
    signal next_shift_reg   : std_logic_vector(1 downto 0) := "00";
    signal s_not_used       : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_0        : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_120      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_180      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal s_phase_240      : std_logic_vector(max_num_points downto 0) := (others => '0');
    signal sig_comb_mask    : std_logic_vector(max_num_points downto 0) := (others => '0'); -- '1' use index in ram, '0' do not use index in ram
    signal phase_to_out     : std_logic := '0'; -- 0 = output 2-phase, 1 = output 3-phase
    signal set_done         : std_logic := '0'; -- indicates when all combs of current set of points have been generated

begin

    ------------------------------------------------------------------------------
    combination_generator  :   process(current_state, i_next, i_num_points, i_Clk, next_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Handles writes to input RAM
    ------------------------------------------------------------------------------
    constant ena_rising     : std_logic_vector := "01";
    constant one            : std_logic_vector(max_num_points downto 0) := (0 => '1', others => '0');
    constant fully_incremtd : std_logic_vector(max_num_points downto 0) := (others => '1');
    variable rdy_high_cnt   : integer := 0;
    variable set_high_cnt   : integer := 0;
    variable comb_mask_sum  : integer := 0;
    variable carry_1        : std_logic := '0';
    variable point_cnt      : integer range 0 to max_num_points := 0;
    variable mask_sum       : integer := 0;

    
    begin
    if (rising_edge(i_Clk)) then
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                if i_Rst = '0' then
                    sig_mask_0 <= (others => 0);
                    sig_mask_1 <= (others => 0);
                    sig_comb_mask <= (1 => '1', others => '0'); -- populate LSB+1, one comb before first valid comb
                    current_state <= s_inc_comb;
                else 
                    current_state  <=  s_rst;
                end if;
    ----------------------------------Increment comb_mask-------------------------                
            when s_inc_comb =>
                -- increment mask, or reset if fully incremented
                comb_mask_sum := 0; 
                if sig_comb_mask = fully_incremtd then
                    set_done <= '1';
                    current_state <= s_rst;
                else
                    sig_comb_mask <= std_logic_vector(unsigned(sig_comb_mask) + unsigned(one)); -- increment comb_mask
                    current_state <= s_comb_check;
                end if;          
    ------------------------------------Check comb_mask--------------------------                
            when s_comb_check => 
            
                for k in 0 to max_num_points loop
                    if sig_comb_mask(k) = '1' then -- get bits set in mask
                        comb_mask_sum := comb_mask_sum + 1;
                    end if;
                end loop;
                                      
                if comb_mask_sum mod 2 = 0 then     -- goto 2-phase
                    current_state <= s_mask_0;
                elsif comb_mask_sum mod 3 = 0 then  -- goto 3-phase
                    current_state <= s_mask_1;
                else                                -- goto increment s_comb_mask
                    current_state <= s_inc_comb;
                end if;           
    ------------------------------------Increment mask 0--------------------------                
            when s_mask_0 =>
                if next_shift_reg = ena_rising then -- next comb pls
                    phase_to_out <= '0'; -- output 2-phase
                    for i in 0 to max_num_points loop -- check status of all indeces
                        if sig_comb_mask(i) = '1' then -- only check index "assigned" by comb mask
                            if point_cnt = 0 then -- always increment LSB
                                if sig_mask_0(i) = 1 then -- carry over
                                    sig_mask_0(i) <= 0;
                                    carry_1 := '1';
                                else
                                    sig_mask_0(i) <= sig_mask_0(i) + 1; -- else increment
                                end if; 
                            else
                                if sig_mask_0(i) < 1 then -- if increment on carry
                                    if carry_1 = '1' then
                                        sig_mask_0(i) <= sig_mask_0(i) + 1;
                                        carry_1 := '0';
                                    end if;
                                elsif sig_mask_0(i) = 1 then -- else reset and keep carry for next it
                                    if carry_1 = '1' then
                                        sig_mask_0(i) <= 0;
                                        carry_1 := '1';
                                    end if;
                                end if;
                            end if;
                            point_cnt := point_cnt + 1;
                        end if;
                        mask_sum := mask_sum + sig_mask_0(i);
                    end loop;
                    
                    if mask_sum = comb_mask_sum then -- check if last comb for current comb mask. +1 because sig_mask_0 signal won't be assigned last increment until process ends
                        s_data_rdy <= '0';
                        if comb_mask_sum mod 3 = 0 then  -- goto 3-phase if mod3=0
                            sig_mask_0 <= (others => 0);
                            sig_mask_1 <= (others => 0);
                            current_state <= s_mask_1;
                        else 
                            sig_mask_0 <= (others => 0); -- else goto increment comb mask
                            current_state <= s_inc_comb;
                        end if;
                    else
                        s_data_rdy <= '1';
                    end if;
                    mask_sum := 0;
                    point_cnt := 0;
                    carry_1 := '0';
                else
                    current_state <= s_mask_0;
                end if;
    ------------------------------------Increment mask 1--------------------------                    
            when s_mask_1 =>
                if next_shift_reg = ena_rising then
                    phase_to_out <= '1';
                    for j in 0 to max_num_points loop -- check status of all indeces
                        if sig_comb_mask(j) = '1' then -- only check index "assigned" by comb mask
                            if point_cnt = 0 then -- always increment LSB
                                if sig_mask_1(j) = 2 then -- carry over
                                    sig_mask_1(j) <= 0;
                                    carry_1 := '1';
                                else
                                    sig_mask_1(j) <= sig_mask_1(j) + 1; -- else increment
                                end if; 
                            else
                                if sig_mask_1(j) < 2 then -- if increment on carry
                                    if carry_1 = '1' then
                                        sig_mask_1(j) <= sig_mask_1(j) + 1;
                                        carry_1 := '0';
                                    end if;
                                elsif sig_mask_1(j) = 2 then -- else reset and keep carry for next it
                                    if carry_1 = '1' then
                                        sig_mask_1(j) <= 0;
                                        carry_1 := '1';
                                    end if;
                                end if;
                            end if;
                            point_cnt := point_cnt + 1;
                        end if;
                        mask_sum := mask_sum + sig_mask_1(j);
                    end loop;
                    
                    if mask_sum = comb_mask_sum*2 then -- check if last comb for current comb mask. +1 because sig_mask_0 signal won't be assigned last increment until process ends
                        s_data_rdy <= '0';
                        sig_mask_0 <= (others => 0); -- else goto increment comb mask
                        sig_mask_1 <= (others => 0); -- else goto increment comb mask
                        current_state <= s_inc_comb;
                    else 
                        s_data_rdy <= '1';
                    end if;
                    mask_sum := 0;
                    point_cnt := 0;
                    carry_1 := '0';
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
            comb_mask_sum := 0;
            sig_comb_mask <= (others => '0');
            sig_mask_0 <= (others => 0);
            sig_mask_1 <= (others => 0);
            phase_to_out <= '0';
            carry_1 := '0';
            point_cnt := 0;
            s_data_rdy <= '0';
        end if;
        
    -----------------------------------------------------------------------------
      
        if s_data_rdy = '1' then
            rdy_high_cnt := rdy_high_cnt + 1;
        end if;
        if rdy_high_cnt = 2 then -- find good value?
            s_data_rdy <= '0';
            rdy_high_cnt := 0;
        end if;

        if set_done = '1' then
            set_high_cnt := set_high_cnt + 1;
        end if;
        if set_high_cnt = 2 then -- find good value?
            set_done <= '0';
            set_high_cnt := 0;
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
    
    
    
o_set_done <= set_done;
o_data_rdy <= s_data_rdy;


GEN_P2_D0: for i in 0 to max_num_points generate
    o_s2_phase_0(i) <= '1' when sig_mask_0(i) = 0 and sig_comb_mask(i) = '1' and phase_to_out = '0' else '0'; -- 2-phase 0 deg output
end generate GEN_P2_D0;

GEN_P2_D180: for i in 0 to max_num_points generate
    o_s2_phase_180(i) <= '1' when sig_mask_0(i) = 1 and sig_comb_mask(i) = '1' and phase_to_out = '0' else '0'; -- 2-phase 180 deg output
end generate GEN_P2_D180;

GEN_P3_D0: for i in 0 to max_num_points generate
    o_s3_phase_0(i) <= '1' when sig_mask_1(i) = 0 and sig_comb_mask(i) = '1' and phase_to_out = '1' else '0'; -- 3-phase 0 deg output
end generate GEN_P3_D0;

GEN_P3_D120: for i in 0 to max_num_points generate
    o_s3_phase_120(i) <= '1' when sig_mask_1(i) = 1 and sig_comb_mask(i) = '1' and phase_to_out = '1' else '0'; -- 3-phase 120 deg output
end generate GEN_P3_D120;

GEN_P3_D240: for i in 0 to max_num_points generate
    o_s3_phase_240(i) <= '1' when sig_mask_1(i) = 2 and sig_comb_mask(i) = '1' and phase_to_out = '1' else '0'; -- 3-phase 240 deg output
end generate GEN_P3_D240;


end Behavioral;


