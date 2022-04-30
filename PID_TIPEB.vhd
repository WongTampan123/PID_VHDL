--------------------------------------------------------------------------
--                     VHDL Implementation of PI-D                      --
--------------------------------------------------------------------------
--                            M. Azhar Wahid                            --
--                      Bandung State Polytechnic                       --
--                                                                      --
-- 				  Github:https://github.com/WongTampan123                --
--------------------------------------------------------------------------         

--------------------------------------------------------------------------
--                        ALGORITMA SI ABANG                            --
--------------------------------------------------------------------------

library ieee;
use IEEE.std_logic_1164.ALL;
--use IEEE.std_logic_signed.ALL;
use IEEE.numeric_std.ALL;
--use IEEE.fixed_pkg.all;

library ieee_proposed;
use ieee_proposed.fixed_pkg.all;
--use ieee_proposed.float_pkg.all;
use ieee_proposed.fixed_float_types.all;

entity PID_TIPEB is
	port(
		CLK		: in std_logic;
		SV_IN		: in ufixed(7 downto 0);
		PV_IN		: in ufixed(7 downto 0);
		MV_OUT	: out std_logic_vector(7 downto 0);
		TX_DV		: out std_logic
		);
end entity;

architecture behavioral of PID_TIPEB is
	--FSM untuk PID
	type State is (s_Ambil_PV_SV, s_Hitung_Error, s_Hitung_P, s_Hitung_I,
						s_Hitung_D, s_Hitung_MV, s_Delay);
	signal r_State	: State	:= s_Ambil_PV_SV; 
	
	--Konstanta KP, KI, KD, Skala
	constant c_KP		: sfixed(8 downto -8) := to_sfixed(19.5,8,-8); 
	constant c_KI		: sfixed(8 downto -8) := to_sfixed(0.97,8,-8);
	--constant c_KD		: sfixed(8 downto -8) := to_sfixed(0.748,8,-8);
	constant C_KD		: sfixed(11 downto 0) := to_sfixed(994,11,0);
	constant c_Scale	: ufixed(0 downto -9) := to_ufixed(0.00976,0,-9);
	
	--Signal Konversi PV dan SV
	signal r_SV_Conv	: ufixed(8 downto -9);
	signal r_PV_Conv	: ufixed(8 downto -9);
	signal r_SV			: sfixed(9 downto -9);
	signal r_PV			: sfixed(9 downto -9);
	
	--Signal pada FSM PID
	signal r_SV_PID			: sfixed(9 downto -9) := (others => '0');
	signal r_PV_PID			: sfixed(9 downto -9) := (others => '0');
	signal r_PV_PID_Past		: sfixed(9 downto -9) := (others => '0');
	signal r_Error				: sfixed(10 downto -9) := (others => '0');
	signal r_Error_Past		: sfixed(10 downto -9) := (others => '0');
	signal r_Error_Int		: sfixed(12 downto -22) := (others => '0');
	signal r_Error_Int_Past	: sfixed(12 downto -22) := (others => '0');
	signal r_Error_Int_Total: sfixed(13 downto -22) := (others => '0');
	signal r_P					: sfixed(19 downto -17) := (others => '0');
	signal r_I					: sfixed(22 downto -30) := (others => '0');
	signal r_D					: sfixed(28 downto -17) := (others => '0');
	signal r_MV					: sfixed(30 downto -30) := (others => '0');
	
begin
	TX_DV		 <= '1';
	
	--Konversi PV dan SV ke Nilai Sebenarnya
	r_SV_Conv <= SV_IN*c_Scale;
	r_SV		 <= to_sfixed(r_SV_Conv);
	
	r_PV_Conv <= PV_IN*c_Scale;
	r_PV		 <= to_sfixed(r_PV_Conv);
	
	--FSM PID
	process(CLK)
	variable r_cnt : integer := 0;	
	begin
		if(rising_edge(CLK))then
		
		case r_State is
		
			when s_Ambil_PV_SV => 
				r_SV_PID	<= r_SV;
				r_PV_PID <= r_PV;				
				r_State  <= s_Hitung_Error;
				
			when s_Hitung_Error =>
				r_Error 		<= r_SV_PID - r_PV_PID;				
				r_State		<= s_Hitung_P;
				
			when s_Hitung_P =>
				r_P	  <= c_KP*r_Error;
				r_State <= s_Hitung_I;
			
			when s_Hitung_I =>
				if(r_cnt = 0) then
					r_Error_Int	<= (r_Error + r_Error_Past)*to_sfixed(0.005,0,-13);
					r_State		<= s_Hitung_I;
					r_cnt			:= r_cnt+1;
				elsif(r_cnt = 1) then
					r_Error_Int_Total	<= resize(r_Error_Int + r_Error_Int_Past,
														 r_Error_Int_Total'high,r_Error_Int_Total'low);
					r_state				<= s_Hitung_I;
					r_cnt					:= r_cnt+1;
				elsif(r_cnt = 2) then
					r_I					<= resize(c_KI * r_Error_Int_Total,r_I'high,r_I'low);
					r_Error_Int_Past	<= resize(r_Error_Int_Total,r_Error_Int_Past'high,
														 r_Error_Int_Past'low);
					r_state				<= s_Hitung_D;
					r_cnt					:= 0;
				end if;
			
			when s_Hitung_D =>
				if(r_cnt = 0) then
					r_D	 <= resize((c_KD*(r_PV_PID - r_PV_PID_Past)*to_sfixed(100,8,0)),r_D'high,r_D'low);
					r_state<= s_Hitung_D;
					r_cnt	 := r_cnt+1;
				elsif(r_cnt = 1) then
					r_PV_PID_Past <= r_PV_PID;
					r_state		  <= s_Hitung_MV;
					r_cnt			  := 0;
				end if;
				
			when s_Hitung_MV =>
				r_MV 	 <= resize(r_P+r_I-r_D,r_MV'high,r_MV'low);
				r_state<= s_Delay;
				
			when s_Delay =>
				r_Error_Past <= r_Error;
				r_state		 <= s_Ambil_PV_SV;
				
			when others =>
			NULL;
			
		end case;
		
		end if;
	end process;	

	MV_OUT <= "11111111" when r_MV > to_sfixed(255,r_MV) else
				 "00000001" when r_MV < to_sfixed(0,r_MV) else
				 to_slv(r_MV(7 downto 0));
				 
end architecture;