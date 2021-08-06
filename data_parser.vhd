----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 03/09/2021 10:18:33 AM
-- Design Name: 
-- Module Name: data_parser - Behavioral
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


entity data_parser is
    Port ( i_RX_Byte    : in std_logic_vector(7 downto 0);
           i_Clk        : in STD_LOGIC;
           i_Ena        : in STD_LOGIC;
           i_Rst        : in std_logic ;
           o_Error      : out std_logic;
           o_Found_Magic : out STD_LOGIC;
           o_Write_BRAM : out STD_LOGIC_VECTOR (3 downto 0)       
    );
end data_parser;

architecture Behavioral of data_parser is
    type    STATE_TYPE      is  (s_rst, s_magic_word, 
    s_frame_hdr, s_tlv_hdr, s_tlv_points, s_tlv_other, s_error);    --  add states here
    signal  current_state   :   STATE_TYPE  :=  s_rst;
    --signal  next_state      :   STATE_TYPE  :=  s_rst;
    
    signal found_magic_s       :   std_logic := '0';
    signal magic_word_buff  :   std_logic_vector(63 downto 0) := (others => '0');
    --signal hdr_cnt          :   integer range 0 to 32;
    signal packet_size      :   std_logic_vector(31 downto 0) := (others => '0');
    signal num_points       :   std_logic_vector(31 downto 0) := (others => '0');
    --signal rxd_points       :   std_logic_vector(31 downto 0);
    signal tlv_hdr          :   std_logic_vector(63 downto 0) := (others => '0');
    signal skip_length      :   integer := 0; --std_logic_vector(31 downto 0);
    --signal skip_cnt         :   std_logic_vector(31 downto 0);
    --signal x_cnt            :   std_logic_vector(31 downto 0);
    signal x_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    --signal y_cnt            :   std_logic_vector(31 downto 0);
    signal y_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    --signal z_cnt            :   std_logic_vector(31 downto 0);
    signal z_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    --signal dp_cnt           :   std_logic_vector(31 downto 0);
    signal dp_arr           :   std_logic_vector(31 downto 0) := (others => '0');
    signal r_data_rdy       :   std_logic := '0';
    signal r_error          :   std_logic := '0';
    signal ena_shift_reg    :   std_logic_vector(1 downto 0) := "00";
    signal tlv_hdr_cnt_s    :   integer := 0;
    signal rxd_points_s     :   integer := 0;

begin

    ------------------------------------------------------------------------------
    next_state_logic    :   process(current_state, i_Ena, i_Clk, ena_shift_reg)  -- Add input signals to sensitivity list
    ------------------------------------------------------------------------------
    -- Next state logic process. Here goes state transition conditions. 
    -- Sensitive to state change and input signals.
    ------------------------------------------------------------------------------
    variable magic_word_buff_var:   std_logic_vector(63 downto 0) := (others => '0');
    constant magic_word : std_logic_vector(63 downto 0) := "0000001000000001000001000000001100000110000001010000100000000111";
    variable hdr_cnt    : integer range 0 to 32 := 0;
    variable tlv_hdr_cnt: integer range 0 to 32 := 0;
    variable skip_cnt   : integer := 0; --std_logic_vector(31 downto 0);
    variable x_cnt      : integer range 0 to 32 := 0;
    variable y_cnt      : integer range 0 to 32 := 0;
    variable z_cnt      : integer range 0 to 32 := 0;
    variable dp_cnt     : integer range 0 to 32 := 0;
    variable rxd_points : integer := 0; --std_logic_vector(31 downto 0);
    constant int_one    : integer := 1; 
    constant std_vec_one: std_logic_vector(31 downto 0) := "00000000000000000000000000000001";
    constant ena_rising : std_logic_vector(1 downto 0) := "01";
    
    begin
    
    if (rising_edge(i_Clk)) then
    
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
            magic_word_buff <= (others => '0');
            --hdr_cnt <= 0;
            packet_size <= (others => '0');
            num_points <= (others => '0');
            --rxd_points <= (others => '0');
            tlv_hdr <= (others => '0');
            skip_length <= 0;--(others => '0');
            --skip_cnt <= (others => '0');
            --x_cnt <= (others => '0');
            x_arr <= (others => '0');
            --y_cnt <= (others => '0');
            y_arr <= (others => '0');
            --z_cnt <= (others => '0');
            z_arr <= (others => '0');
            --dp_cnt <= (others => '0');
            dp_arr <= (others => '0');
            r_data_rdy <= '0';
            r_error <= '0';
        end if;
    
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    magic_word_buff_var(15 downto 8) := i_RX_Byte; -- lowest byte + leftshift
                    magic_word_buff(15 downto 8) <= i_RX_Byte; -- lowest byte + leftshiftg
                    current_state <= s_magic_word;
                else 
                    current_state  <=  s_rst;
                end if;

    ---------------------------------------MAGIC WORD-----------------------------                
            when s_magic_word =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    --magic_word_buff_var := magic_word; -- i_RX_Byte; -- testing
                    magic_word_buff_var(7 downto 0) := i_RX_Byte;
                    --if magic_word_buff_var = magic_word then -- magic word found
                    if magic_word_buff_var = magic_word then -- testing
                        --found_magic_s <= '1';
                        hdr_cnt := 0;
                        current_state <= s_frame_hdr;
                    else -- left shift 1 byte
                        magic_word_buff_var := magic_word_buff_var(55 downto 0) & "00000000";
                        found_magic_s <= not found_magic_s;
                    end if;
                    magic_word_buff <= magic_word_buff_var;
                else
                    current_state <= s_magic_word;
                end if;
                
    --------------------------------------FRAME HEADER----------------------------                  
            when s_frame_hdr =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    if hdr_cnt < 4 then
                        hdr_cnt := hdr_cnt + 1;
                    elsif hdr_cnt < 8 then
                        packet_size(7+(hdr_cnt-4)*8 downto (hdr_cnt-4)*8) <= i_RX_Byte;
                        hdr_cnt := hdr_cnt + 1;
                    elsif hdr_cnt < 20 then
                        hdr_cnt := hdr_cnt + 1;
                    elsif hdr_cnt < 24 then
                        num_points(7+(hdr_cnt-20)*8 downto (hdr_cnt-20)*8) <= i_RX_Byte;
                        hdr_cnt := hdr_cnt + 1;
                    elsif hdr_cnt < 32 then
                        hdr_cnt := hdr_cnt + 1;
                    elsif hdr_cnt = 32 then
                        tlv_hdr_cnt := 1;
                        tlv_hdr(7 downto 0) <= i_RX_Byte;
                        current_state <= s_tlv_hdr;                       
                    else 
                        current_state <= s_error;                       
                    end if;
                else
                    current_state  <=  s_frame_hdr;
                end if;
 
    ----------------------------------------TLV HEADER----------------------------                 
            when s_tlv_hdr =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    if tlv_hdr_cnt < 8 then
                        tlv_hdr(7+tlv_hdr_cnt*8 downto tlv_hdr_cnt*8) <= i_RX_Byte;
                        tlv_hdr_cnt := tlv_hdr_cnt + 1;
                    elsif tlv_hdr_cnt = 8 then
                        if tlv_hdr(63 downto 32) = "00000001000000000000000000000000" then -- 0x01000000
                            x_arr(7 downto 0) <= i_RX_Byte;
                            x_cnt := 1;
                            y_cnt := 0;
                            z_cnt := 0;
                            dp_cnt := 0;
                            current_state <= s_tlv_points;
                        else
                            skip_length <= to_integer(unsigned(tlv_hdr(31 downto 0)));
                            skip_cnt := 0; 
                            current_state <= s_tlv_other;
                        end if;                       
                    else 
                        current_state <= s_error;
                    end if;
                    tlv_hdr_cnt_s <= tlv_hdr_cnt;
                else
                    current_state  <=  s_tlv_hdr;
                end if;
                
    ----------------------------------------TLV POINTS----------------------------  
            when s_tlv_points =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    if to_integer(unsigned(num_points)) = 0 then
                        current_state <= s_magic_word;
                    elsif x_cnt < 4 then
                        x_arr(7+x_cnt*8 downto x_cnt*8) <= i_RX_Byte;
                        x_cnt := x_cnt + 1;
                    elsif x_cnt = 4 and y_cnt <4 then
                        y_arr(7+y_cnt*8 downto y_cnt*8) <= i_RX_Byte;
                        y_cnt := y_cnt  + 1;
                    elsif y_cnt = 4 and z_cnt < 4 then
                        z_arr(7+z_cnt*8 downto z_cnt*8) <= i_RX_Byte;
                        z_cnt := z_cnt  + 1;
                    elsif z_cnt = 4 and dp_cnt < 4 then
                        dp_arr(7+dp_cnt*8 downto dp_cnt*8) <= i_RX_Byte;
                        dp_cnt := dp_cnt  + 1;
                    elsif dp_cnt = 4 then 
                        if dp_cnt = 4 then
                            rxd_points := rxd_points + int_one;
                            if rxd_points < to_integer(unsigned(num_points)) then
                                x_arr(7 downto 0) <= i_RX_Byte;
                                x_cnt := 1;
                                y_cnt := 0;
                                z_cnt := 0;
                                dp_cnt := 0;
                            elsif rxd_points = to_integer(unsigned(num_points)) then
                                r_data_rdy <= '0';
                                magic_word_buff_var(15 downto 8) := i_RX_Byte; -- lowest byte + leftshift
                                current_state <= s_magic_word;
                            else 
                                current_state <= s_error;
                            end if;
                        else 
                            current_state <= s_error;
                        end if;
                    end if;
                    rxd_points_s <= rxd_points;
                else
                    current_state  <=  s_tlv_points;
                end if;
 
    ----------------------------------------TLV OTHER-----------------------------                
            when s_tlv_other =>
                --if rising_edge(i_Ena) then
                --if i_Ena'EVENT and i_Ena = '1' then
                if ena_shift_reg = ena_rising then
                    if skip_cnt < skip_length then
                        skip_cnt := skip_cnt  + 1;
                    elsif skip_cnt = skip_length then
                        tlv_hdr(7 downto 0) <= i_RX_Byte;
                        tlv_hdr_cnt := 1;
                        current_state <= s_tlv_hdr;
                    else 
                        current_state <= s_error;
                    end if;
                else
                    current_state  <=  s_tlv_other;
                end if;
                
            when s_error =>    
                r_error <= '1';
                
            when others =>
                null;
        end case;
    
    end if;
        
    ------------------------------------------------------------------------------
    end process next_state_logic;
    ------------------------------------------------------------------------------

    
    ------------------------------------------------------------------------------
    ena_shift_reg_process :   process(i_Ena, i_Clk)  
    ------------------------------------------------------------------------------
    -- Holds the edge state of i_Ena.
    ------------------------------------------------------------------------------
    begin  
    ------------------------------------------------------------------------------
        if rising_edge(i_Clk) then
            ena_shift_reg <= ena_shift_reg(0) & i_Ena;
        end if;
    ------------------------------------------------------------------------------
    end process ena_shift_reg_process;
    ------------------------------------------------------------------------------
            
    
    
    
o_Error <= '1' when current_state = s_error else '0';
--o_Error <= r_error;

end Behavioral;
