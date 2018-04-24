`timescale 1ns / 1ps
`include "common_defines.v"

`define PASS 				1'b0
`define FAIL				1'b1
`define TEST_LENGTH 		16'hFFFF

module test_motor_mixer;
	
	parameter 	BIT_WIDTH 				= 16;
	parameter	MOTOR_RATE_BIT_WIDTH 	= 8;
	parameter	CLK_CONVERSION_HIGH		= 19;
	parameter	CLK_CONVERSION_LOW 		= 37; 
	
	parameter	YAW_INIT_VAL			= 16'h0000;
	parameter	ROLL_INIT_VAL			= 16'h0000;
	parameter	PITCH_INIT_VAL			= 16'h0000;
	parameter	THROTTLE_INIT_VAL		= 16'h0000;
	
	localparam	scale_diviser = 1'b1;
	
	reg   rst_n;
	wire  sys_clk;
	reg	  [BIT_WIDTH-1:0] yaw_rate = 0;
	reg	  [BIT_WIDTH-1:0] roll_rate = 0;
	reg	  [BIT_WIDTH-1:0] pitch_rate = 0;
	reg	  [BIT_WIDTH-1:0] throttle_rate = 0;
	wire  [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate;
	wire  [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate;
	wire  [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate;
	wire  [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate;
	
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m1_expected_output;	
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m1_expected_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor1_rate_last;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m2_expected_output;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m2_expected_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor2_rate_last;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m3_expected_output;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m3_expected_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor3_rate_last;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m4_expected_output;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m4_expected_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor4_rate_last;
	
	reg check_return1;
	reg check_return2;
	reg check_return3;
	reg check_return4;
	reg	exit_flag;

	reg [BIT_WIDTH-1:0] fail_counter;
	reg [BIT_WIDTH-1:0] pass_counter;
	reg [BIT_WIDTH-1:0] test_counter;
	
	clock clk1(.clk(sys_clk));
	/*
	us_clk	#(CLK_CONVERSION_HIGH,
			  CLK_CONVERSION_LOW)
	clk2	 (.us_clk(sys_clk),
			  .sys_clk(base_clk),
			  .resetn(rst_n));	
	*/
	motor_mixer	#(BIT_WIDTH,
				  MOTOR_RATE_BIT_WIDTH)
	mixer_DUT	(//	Inputs
				 .rst_n(rst_n),
				 .sys_clk(sys_clk),
				 .yaw_rate(yaw_rate),
				 .roll_rate(roll_rate),
				 .pitch_rate(pitch_rate),
				 .throttle_rate(throttle_rate),
				 //	Outputs
				 .motor_1_rate(motor_1_rate),
				 .motor_2_rate(motor_2_rate),
				 .motor_3_rate(motor_3_rate),
				 .motor_4_rate(motor_4_rate));
	
	initial begin
		$display("Test In Progress.");
		
		exit_flag 	 = `FALSE;
		rst_n 		 = `TRUE;
		
		yaw_rate		= YAW_INIT_VAL;
		roll_rate		= ROLL_INIT_VAL;
		pitch_rate		= PITCH_INIT_VAL;
		throttle_rate	= THROTTLE_INIT_VAL;
						  
		m1_expected_mapped = 16'h0;	
		m1_expected_output = 16'h0;
		
		test_counter = 0;
		fail_counter = 0;
		pass_counter = 0;
	end
	
	always @ (posedge sys_clk) begin
		#5 rst_n = `TRUE;
		while (exit_flag == `FALSE) begin
/*******************************************************************************************************************
*	 		Increment Test counter, check value length for rate values expected to enter
*******************************************************************************************************************/
			test_counter = test_counter + 1;
			
			if (test_counter == `TEST_LENGTH) begin 
				yaw_rate		= YAW_INIT_VAL;
				roll_rate		= ROLL_INIT_VAL;
				pitch_rate		= PITCH_INIT_VAL;
				throttle_rate	= THROTTLE_INIT_VAL;
				end
			else begin
				yaw_rate		= yaw_rate + 1;
				roll_rate		= roll_rate + 1;
				pitch_rate		= pitch_rate + 1;
				throttle_rate	= throttle_rate + 1;
				end
			#5
/*******************************************************************************************************************
*	 		Calculate expected values for motor rate outputs
*******************************************************************************************************************/			
			if (!rst_n || (!yaw_rate && !roll_rate && !pitch_rate)) begin
				m1_expected_mapped	=	16'h0000;
				m2_expected_mapped	=	16'h0000;
				m3_expected_mapped	=	16'h0000;
				m4_expected_mapped	=	16'h0000;
				end
			else begin
				//	Motor 1 calculation of expected rate value
				m1_expected_output	=	(`MOTOR_RATE_BIAS +
										((~yaw_rate + 1)	>> `MOTOR_RATE_YAW_SCALER) +
										((roll_rate)		>> `MOTOR_RATE_ROLL_SCALER) +
										((~pitch_rate + 1)	>> `MOTOR_RATE_PITCH_SCALER) +
										(throttle_rate));
				m1_expected_mapped	=	((m1_expected_output + `MOTOR_RATE_MAP_ROUND_VAL) >> `MAPPING_SHIFT_BIT);
				
				//	Motor 2 calculation of expected rate value
				m2_expected_output	=	(`MOTOR_RATE_BIAS +
										((yaw_rate)			>> `MOTOR_RATE_YAW_SCALER) +
										((~roll_rate + 1)	>> `MOTOR_RATE_ROLL_SCALER) +
										((~pitch_rate + 1)	>> `MOTOR_RATE_PITCH_SCALER) +
										(throttle_rate));
				m2_expected_mapped	=	((m2_expected_output + `MOTOR_RATE_MAP_ROUND_VAL) >> `MAPPING_SHIFT_BIT);
				
				//	Motor 3 calculation of expected rate value
				m3_expected_output	=	(`MOTOR_RATE_BIAS +
										((yaw_rate)			>> `MOTOR_RATE_YAW_SCALER) +
										((roll_rate)		>> `MOTOR_RATE_ROLL_SCALER) +
										((pitch_rate)		>> `MOTOR_RATE_PITCH_SCALER) +
										(throttle_rate));
				m3_expected_mapped	=	((m3_expected_output + `MOTOR_RATE_MAP_ROUND_VAL) >> `MAPPING_SHIFT_BIT);
				
				//	Motor 4 calculation of expected rate value
				m4_expected_output	=	(`MOTOR_RATE_BIAS +
										((~yaw_rate + 1)	>> `MOTOR_RATE_YAW_SCALER) +
										((~roll_rate + 1)	>> `MOTOR_RATE_ROLL_SCALER) +
										((pitch_rate)		>> `MOTOR_RATE_PITCH_SCALER) +
										(throttle_rate));
				m4_expected_mapped	=	((m4_expected_output + `MOTOR_RATE_MAP_ROUND_VAL) >> `MAPPING_SHIFT_BIT);
				end
/*******************************************************************************************************************
*	 		Check bounded limits of motor rate output
*******************************************************************************************************************/
			if (m1_expected_mapped < `MOTOR_VAL_MIN || m1_expected_mapped > `MOTOR_VAL_MAX)
				m1_expected_mapped = motor1_rate_last;
			else
				motor1_rate_last = m1_expected_mapped;
			if (m2_expected_mapped < `MOTOR_VAL_MIN || m2_expected_mapped > `MOTOR_VAL_MAX)
				m2_expected_mapped = motor2_rate_last;
			else
				motor2_rate_last = m2_expected_mapped;	
			if (m3_expected_mapped < `MOTOR_VAL_MIN || m3_expected_mapped > `MOTOR_VAL_MAX)
				m3_expected_mapped = motor3_rate_last;
			else
				motor3_rate_last = m3_expected_mapped;
			if (m4_expected_mapped < `MOTOR_VAL_MIN || m4_expected_mapped > `MOTOR_VAL_MAX)
				m4_expected_mapped = motor4_rate_last;
			else
				motor4_rate_last = m4_expected_mapped;
/*******************************************************************************************************************
*	 		Test Module Motor_Rate_Outputs vs. Calculated_Expected_Rates
*******************************************************************************************************************/				
			#5
			assign check_return1 = check_output(fail_counter,
											   test_counter,
						 					   yaw_rate,
						 					   roll_rate,
						 					   pitch_rate,
						 					   throttle_rate,
						 					   m1_expected_mapped, 
						 					   motor_1_rate);
			assign check_return2 = check_output(fail_counter,
											   test_counter,
						 					   yaw_rate,
						 					   roll_rate,
						 					   pitch_rate,
						 					   throttle_rate,
						 					   m1_expected_mapped, 
						 					   motor_1_rate);
			assign check_return3 = check_output(fail_counter,
											   test_counter,
						 					   yaw_rate,
						 					   roll_rate,
						 					   pitch_rate,
						 					   throttle_rate,
						 					   m1_expected_mapped, 
						 					   motor_1_rate);
			assign check_return4 = check_output(fail_counter,
											   test_counter,
						 					   yaw_rate,
						 					   roll_rate,
						 					   pitch_rate,
						 					   throttle_rate,
						 					   m1_expected_mapped, 
						 					   motor_1_rate);
			if ((check_return1 | check_return2 | check_return3 | check_return4) == `FAIL)
				fail_counter	= fail_counter + 1;
			else
				pass_counter	= pass_counter  + 1;
			 /*
			$display("Running Test %d",test_counter);
			$display("roll          %d",roll_rate);
			$display("pitch         %d",pitch_rate);
			$display("throttle      %d",throttle_rate);
			$display("Motor Rate Actual  Output  %d",motor_1_rate);
			$display("Motor Rate Expected Output %d",m1_expected_mapped);
			*/	
/*******************************************************************************************************************
*	 		Is the test done, according to the test coutner??
*******************************************************************************************************************/
			if (test_counter == `TEST_LENGTH) begin
				$display("****************Testing Completed****************");
				$display("Tests Run     -	%d", test_counter);
				$display("Tests Passed  -	%d", pass_counter);
				$display("Tests Failed  -	%d", fail_counter);
				exit_flag = `TRUE;
				end
			end
		$display("Test Ended.");
		$stop;
	end
	
	function integer check_output(input [31:0] fail_counter,
								  input [BIT_WIDTH-1:0] test_counter,	
								  input [BIT_WIDTH-1:0] yaw_rate,
								  input [BIT_WIDTH-1:0] roll_rate,
								  input [BIT_WIDTH-1:0] pitch_rate,
								  input [BIT_WIDTH-1:0] throttle_rate,
								  input [BIT_WIDTH-1:0] m1_expected_output, 
								  input [BIT_WIDTH-1:0] m1_actual_output);						   
		begin					  
		if (m1_expected_output != m1_actual_output) begin
			if (fail_counter == 0) begin
				$display("------------------Error Log-------------------");
			end
			$display("----------------------------------------------");
			$display("-   Number of Errors -%d             -" 			, fail_counter);
			$display("-   Test Case  	          -%d             -"		, test_counter);
			$display("- INPUTS                    rate             -");
			$display("-   yaw_rate \t\t           -%d -           -"	, yaw_rate);
			$display("-   roll_rate \t\t          -%d -           -"	, roll_rate);
			$display("-   pitch_rate \t\t         -%d -           -"	, pitch_rate);
			$display("-   throttle_rate         -%d -           -"		, throttle_rate);
			$display("- OUTPUTS                                    -");				  							
			$display("-   Actual Output \t\t      -%d -           -"	, m1_actual_output);	
			$display("-   Expected Output \t\t    -%d -           -"	, m1_expected_mapped);
			$display("----------------------------------------------");
			check_output = `FAIL;
		end
		else
			//$display("Passed Test Number %d", test_num);
			check_output = `PASS;
		end
	endfunction
	
endmodule