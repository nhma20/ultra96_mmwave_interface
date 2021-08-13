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
-- Description: Parses mmWave data from received UART packets
-- https://dev.ti.com/tirex/explore/content/mmwave_industrial_toolbox_4_8_0/labs/out_of_box_demo/common/docs/understanding_oob_uart_data.html
--
-- In: Bytes from UART get parsed to find points in data.
--
-- Out: Once a point is found, data_rdy is high and data is on o_data_out. 
-- Once all points have been sent to RAM, issue new-set-of-points flag.
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
           --o_Error      : out std_logic;
           --o_Found_Magic: out STD_LOGIC;
           o_set_done   : out STD_LOGIC;
           o_Data_rdy   : out STD_LOGIC;
           o_data_out   : out std_logic_vector(127 downto 0) := (others => '0');
           o_Debug      : out std_logic_vector(7 downto 0) := (others => '0')  
    );
end data_parser;

architecture Behavioral of data_parser is
    type    STATE_TYPE      is  (s_rst, s_magic_word, 
    s_frame_hdr, s_tlv_hdr, s_tlv_points, s_tlv_other, s_error);    --  add states here
    signal  current_state   :   STATE_TYPE  :=  s_rst;
    signal found_magic_s    :   std_logic := '0';
    signal magic_word_buff  :   std_logic_vector(63 downto 0) := (others => '0');
    signal packet_size      :   std_logic_vector(31 downto 0) := (others => '0');
    signal num_points       :   std_logic_vector(31 downto 0) := (others => '0');
    signal tlv_hdr          :   std_logic_vector(63 downto 0) := (others => '0');
    signal skip_length      :   integer := 0;
    signal x_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    signal y_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    signal z_arr            :   std_logic_vector(31 downto 0) := (others => '0');
    signal dp_arr           :   std_logic_vector(31 downto 0) := (others => '0');
    signal r_data_rdy       :   std_logic := '0';
    signal r_error          :   std_logic := '0';
    signal ena_shift_reg    :   std_logic_vector(1 downto 0) := "00";
    signal all_points_sent  :   std_logic := '0';

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
    variable skip_cnt   : integer := 0; 
    variable x_cnt      : integer range 0 to 32 := 0;
    variable y_cnt      : integer range 0 to 32 := 0;
    variable z_cnt      : integer range 0 to 32 := 0;
    variable dp_cnt     : integer range 0 to 32 := 0;
    variable rxd_points : integer := 0; 
    constant ena_rising : std_logic_vector(1 downto 0) := "01";
    variable packets_rxd: integer := 0;
    
    begin
    
    if (rising_edge(i_Clk)) then
     
    ------------------------------------------------------------------------------
        case current_state is
    -----------------------------------------RESET--------------------------------          
            when s_rst =>
                if ena_shift_reg = ena_rising then
                    magic_word_buff_var(15 downto 8) := i_RX_Byte; -- lowest byte + leftshift
                    magic_word_buff(15 downto 8) <= i_RX_Byte; -- lowest byte + leftshiftg
                    current_state <= s_magic_word;
                else 
                    current_state  <=  s_rst;
                end if;

    ---------------------------------------MAGIC WORD-----------------------------                
            when s_magic_word =>
--                r_data_rdy <= '0';
--                all_points_sent <= '0';
                if ena_shift_reg = ena_rising then
                    r_data_rdy <= '0';--------------------------
                    all_points_sent <= '0';----------------------
                    magic_word_buff_var(7 downto 0) := i_RX_Byte;
                    if magic_word_buff_var = magic_word then
                        found_magic_s <= '1';
                        hdr_cnt := 0;
                        packets_rxd := 8;
                        current_state <= s_frame_hdr;
                    else -- left shift 1 byte
                        magic_word_buff_var := magic_word_buff_var(55 downto 0) & "00000000";
                    end if;
                    magic_word_buff <= magic_word_buff_var;
                else
                    current_state <= s_magic_word;
                end if;
                
    --------------------------------------FRAME HEADER----------------------------                  
            when s_frame_hdr =>
                found_magic_s <= '0';
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
                    packets_rxd := packets_rxd + 1;
                    if packets_rxd = to_integer(unsigned(packet_size)) then
                        current_state <= s_magic_word;
                    end if;
                else
                    current_state  <=  s_frame_hdr;
                end if;
 
    ----------------------------------------TLV HEADER----------------------------                 
            when s_tlv_hdr =>
                if ena_shift_reg = ena_rising then
                    if tlv_hdr_cnt < 8 then
                        tlv_hdr(7+tlv_hdr_cnt*8 downto tlv_hdr_cnt*8) <= i_RX_Byte;
                        tlv_hdr_cnt := tlv_hdr_cnt + 1;
                    elsif tlv_hdr_cnt = 8 then
                        if tlv_hdr(31 downto 0) = "00000000000000000000000000000001" then -- 0x00000001
                            x_arr(7 downto 0) <= i_RX_Byte;
                            x_cnt := 1;
                            y_cnt := 0;
                            z_cnt := 0;
                            dp_cnt := 0;
                            current_state <= s_tlv_points;
                        else
                            skip_length <= to_integer(unsigned(tlv_hdr(63 downto 32)));
                            skip_cnt := 1; 
                            current_state <= s_tlv_other;
                        end if;                       
                    else 
                        current_state <= s_error;
                    end if;
                    packets_rxd := packets_rxd + 1;
                    if packets_rxd = to_integer(unsigned(packet_size)) then
                        current_state <= s_magic_word;
                    end if;
                else
                    current_state  <=  s_tlv_hdr;
                end if;
                
    ----------------------------------------TLV POINTS----------------------------  
            when s_tlv_points =>
                --r_data_rdy <= '0'; -- move into ena_shift_reg check?
                --if r_data_rdy = '1' then
                --    r_data_rdy <= '0';
                --end if;
                if ena_shift_reg = ena_rising then
                    if r_data_rdy = '1' then
                        r_data_rdy <= '0';
                    end if;
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
                        if dp_cnt = 4 then 
                            rxd_points := rxd_points + 1;
                            r_data_rdy <= '1';
                            if rxd_points < to_integer(unsigned(num_points)) then
                                x_cnt := 0;
                                y_cnt := 0;
                                z_cnt := 0;
                                dp_cnt := 0;
                            elsif rxd_points = to_integer(unsigned(num_points)) then
                                rxd_points := 0;
                                all_points_sent <= '1';
                                current_state <= s_magic_word;
                            elsif rxd_points > to_integer(unsigned(num_points)) then
                                current_state <= s_error;
                            end if;
                        end if;
                    else
                        current_state <= s_error;
                    end if;
                    packets_rxd := packets_rxd + 1;
                    if packets_rxd = to_integer(unsigned(packet_size)) then
                        current_state <= s_magic_word;
                    end if;
                else
                    current_state  <=  s_tlv_points;
                end if;
 
    ----------------------------------------TLV OTHER-----------------------------                
            when s_tlv_other =>
                if ena_shift_reg = ena_rising then
                    if skip_cnt < skip_length then
                        skip_cnt := skip_cnt  + 1;
                    elsif skip_cnt >= skip_length then
                        tlv_hdr(7 downto 0) <= i_RX_Byte;
                        tlv_hdr_cnt := 1;
                        current_state <= s_tlv_hdr;
                    else 
                        current_state <= s_error;
                    end if;
                    packets_rxd := packets_rxd + 1;
                    if packets_rxd = to_integer(unsigned(packet_size)) then
                        current_state <= s_magic_word;
                    end if;
                else
                    current_state  <=  s_tlv_other;
                end if;
                
            when s_error =>    
                r_error <= '1'; -- testing, re-enable
                current_state <= s_magic_word;
                
            when others =>
                null;
        end case;
    
        
    
        if (i_Rst = '1') then
            current_state   <=  s_rst;          -- Reset state
            magic_word_buff <= (others => '0');
            packet_size <= (others => '0');
            num_points <= (others => '0');
            tlv_hdr <= (others => '0');
            skip_length <= 0;--(others => '0');
            x_arr <= (others => '0');
            y_arr <= (others => '0');
            z_arr <= (others => '0');
            dp_arr <= (others => '0');
            r_data_rdy <= '0';
            found_magic_s <= '0';
            r_error <= '0';
        else 
            null;
        end if;
    
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
            
            
----------------------------------------------------------------------------------            
-- Output port assignments
----------------------------------------------------------------------------------            
o_Debug(0) <= '1' when current_state = s_rst else '0';  
o_Debug(1) <= '1' when current_state = s_magic_word else '0';  
o_Debug(2) <= '1' when current_state = s_frame_hdr else '0';  
o_Debug(3) <= '1' when current_state = s_tlv_hdr else '0';  
o_Debug(4) <= '1' when current_state = s_tlv_points  else '0'; 
o_Debug(5) <= '1' when current_state = s_tlv_other   else '0'; 
o_Debug(6) <= '1' when current_state = s_error else '0'; 
o_Debug(7) <= '1';
--o_Debug <= num_points(31 downto 24);

--o_Found_Magic <= found_magic_s;

o_set_done <= all_points_sent;

o_data_out <= x_arr & y_arr & z_arr & dp_arr;
o_Data_rdy <= r_data_rdy;

--o_Error <= r_error;
----------------------------------------------------------------------------------            

end Behavioral;
