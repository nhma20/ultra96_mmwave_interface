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
-- In: Points get stored sequentially in first RAM. When new set of points, switch 
-- to storing in second RAM and make first RAM the one to read from. Switch RAM every new set.
-- Points are stored as 128 bits, each sub-data is float32 (single precision):
-- | 32 bits X | 32 bits Y | 32 bits Z | 32 bits Doppler |
--
-- Out: Point to address of point of interest, and concurrent logic makes it available at output.
-- Cycle through all wanted points by repeatedly changing address and reading output. 
-- Number of points stored in current RAM can be read at designated port (o_num_points).
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


entity points_RAM is

    GENERIC(
        d_width  : INTEGER := 128;    --width of each data word
        size     : INTEGER := 32);  --number of data words the memory can store
        
    Port ( i_Clk        : in STD_LOGIC;
           i_Rst        : in STD_LOGIC;
           i_data_in    : in STD_LOGIC_VECTOR (127 downto 0);
           i_point_addr : in STD_LOGIC_VECTOR(4 downto 0); -- select one of 32 stored points          
           i_set_and_rdy: in std_logic_vector(1 downto 0);
           o_data_rdy   : out STD_LOGIC;
           o_num_points : out STD_LOGIC_VECTOR(4 downto 0);
           --o_test       : out std_logic_vector(7 downto 0);
           o_data_out   : out STD_LOGIC_VECTOR(127 downto 0)
          );
end points_RAM;

architecture Behavioral of points_RAM is
    TYPE memory IS ARRAY(size-1 DOWNTO 0) OF STD_LOGIC_VECTOR(d_width-1 DOWNTO 0); --data type for memory
    signal ram_0             : memory;                             --memory array
    signal ram_1             : memory;                             --memory array
    signal addr_out          : INTEGER RANGE 0 TO size-1;          --internal address register
    signal ena_shift_reg     : std_logic_vector (1 downto 0) := "00";
    signal new_set_shift_reg : std_logic_vector (1 downto 0) := "00";
    signal ram_selector      : std_logic := '1';                   -- selects which ram to read from
    type    STATE_TYPE      is  (s_rst, s_ram_0, s_ram_1);    --  add states here
    signal current_state     : STATE_TYPE  :=  s_rst;
    signal s_data_rdy        : std_logic := '0';                   -- 1 clk high when ram can be read after storing
    signal num_points_out    : integer := 0;
    
    signal s_point_addr     : std_logic_vector(4 downto 0) := "00000"; -- @@@@@@@@@@@ testing
    signal test             : std_logic_vector(1 downto 0) := "00";
    signal one              : std_logic_vector(4 downto 0) := "00001";

begin

    ------------------------------------------------------------------------------
    ram_in    :   process(current_state, i_set_and_rdy, i_Clk, ena_shift_reg, new_set_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Handles writes to input RAM
    ------------------------------------------------------------------------------
    variable points_stored  : integer := 0;
    variable addr_in        : integer := 0;
    constant ena_rising     : std_logic_vector := "01";
    variable high_cnt : integer := 0;

    
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
                if ena_shift_reg = ena_rising then
                    if points_stored < size-1 then
                        ram_0(points_stored) <= i_data_in;
                        points_stored := points_stored + 1;
                    end if;
                end if;
                if new_set_shift_reg = ena_rising then
                    s_data_rdy <= '1';
                    num_points_out <= points_stored; 
                    points_stored := 0;
                    ram_selector <= '0';
                    ram_1 <= (others => (others => '0')); -- new, clears ram_1 before assigning
                    current_state <= s_ram_1;
                else
                    current_state <= s_ram_0;
                end if;
    ---------------------------------------WRITE RAM 1---------------------------                    
            when s_ram_1 =>
                if ena_shift_reg = ena_rising then
                    if points_stored < size-1 then
                        ram_1(points_stored) <= i_data_in;
                        points_stored := points_stored + 1;
                    end if;
                end if;
                if new_set_shift_reg = ena_rising then
                    s_data_rdy <= '1';
                    num_points_out <= points_stored;
                    points_stored := 0;
                    ram_selector <= '1';
                    ram_0 <= (others => (others => '0')); -- new, clears ram_0 before assigning
                    current_state <= s_ram_0;
                else
                    current_state <= s_ram_1;
                end if;
    ---------------------------------------OTHER---------------------------------  
            when others =>
                null;
        end case;
    -----------------------------------------------------------------------------
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
            points_stored := 0;
            num_points_out <= 0;
            ram_0 <= (others => (others => '0')); 
            ram_1 <= (others => (others => '0'));  
            s_data_rdy <= '1'; -- overwrite BRAM with 0's
        end if;
        
    -----------------------------------------------------------------------------
      
        if s_data_rdy = '1' then
            high_cnt := high_cnt + 1;
        end if;
        if high_cnt = 2000 then -- find good value?
            s_data_rdy <= '0';
            high_cnt := 0;
        end if;

    end if;
    ------------------------------------------------------------------------------
    end process ram_in;
    ------------------------------------------------------------------------------
    
    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_set_and_rdy, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_set_and_rdy and i_new_set.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            ena_shift_reg <= ena_shift_reg(0) & i_set_and_rdy(0);
            new_set_shift_reg <= new_set_shift_reg(0) & i_set_and_rdy(1);
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------




   
    o_data_out <= ram_0(to_integer(unsigned(i_point_addr))) when ram_selector = '0' else ram_1(to_integer(unsigned(i_point_addr)));
    o_num_points <= std_logic_vector(to_unsigned(num_points_out, o_num_points'length));
    o_data_rdy <= s_data_rdy;
    
--    RAM_val_0 <= ram_0(0) when ram_selector = '0' else ram_1(0);
--    RAM_val_1 <= ram_0(1) when ram_selector = '0' else ram_1(1);
--    RAM_val_2 <= ram_0(2) when ram_selector = '0' else ram_1(2);
--    RAM_val_3 <= ram_0(3) when ram_selector = '0' else ram_1(3);
--    RAM_val_4 <= ram_0(4) when ram_selector = '0' else ram_1(4);
--    RAM_val_5 <= ram_0(5) when ram_selector = '0' else ram_1(5);
--    RAM_val_6 <= ram_0(6) when ram_selector = '0' else ram_1(6);
--    RAM_val_7 <= ram_0(7) when ram_selector = '0' else ram_1(7);
    
end Behavioral;
