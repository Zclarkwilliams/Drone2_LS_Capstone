`timescale 1ns / 1ps

`include "pid_mixer_defs.v"

`define PASS 		1'b0
`define FAIL		1'b1
`define TEST_LENGTH 16'h0005

module tb_pid_mixer;
	
	parameter 	BIT_WIDTH = 16;
	parameter	MOTOR_RATE_BIT_WIDTH = 8;
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
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m2_expected_output;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m3_expected_output;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] m4_expected_output;
	
	reg check_return;
	reg	exit_flag;

	reg [BIT_WIDTH-1:0] fail_counter;
	reg [BIT_WIDTH-1:0] pass_counter;
	reg [BIT_WIDTH-1:0] test_counter;

	clock clk(.clk(sys_clk));	

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
				 .motor_1_rate(motor_1_rate)/*,
				 .motor_2_rate(motor_2_rate),
				 .motor_3_rate(motor_3_rate),
				 .motor_4_rate(motor_4_rate)*/);
	
	initial begin
		$display("Test In Progress.");
		
		exit_flag 	 = `FALSE;
		rst_n 		 = 0;
		
		yaw_rate		= 16'h0000;
		roll_rate		= 16'h0000;
		pitch_rate		= 16'h0000;
		throttle_rate	= 16'h0000;
		
		test_counter = 0;
		fail_counter = 0;
		pass_counter = 0;
	end
	
	always @ (posedge sys_clk) begin
		
		#5 rst_n = 1;
		#10
		while (exit_flag == `FALSE) begin
			test_counter = test_counter + 1;
			
			if (test_counter == `TEST_LENGTH) begin 
				yaw_rate		= 16'h0000;
				roll_rate		= 16'h0000;
				pitch_rate		= 16'h0000;
				throttle_rate	= 16'h0000;
				end
			else begin
				yaw_rate		= yaw_rate + 1;
				roll_rate		= roll_rate + 1;
				pitch_rate		= pitch_rate + 1;
				throttle_rate	= throttle_rate + 1;
				end
			#100
			if (!rst_n)
				m1_expected_output	=	0;
			else
				m1_expected_output	=	(`BIAS +
										((~yaw_rate + 1)	>> `M1_SCALER_YAW) +
										(roll_rate 			>> `M1_SCALER_ROLL) +
										((~pitch_rate + 1)	>> `M1_SCALER_PITCH) +
										(throttle_rate 		>> `M1_SCALER_THROTTLE));
		
			#100
			assign check_return = check_output(fail_counter,
											   test_counter,
						 					   yaw_rate,
						 					   roll_rate,
						 					   pitch_rate,
						 					   throttle_rate,
						 					   m1_expected_output, 
						 					   motor_1_rate);
			#100
			if (check_return == `FAIL)
				fail_counter	= fail_counter + 1;
			else
				pass_counter	= pass_counter  + 1;
			
			$display("Running Test %d",test_counter);
			$display("yaw      %d",yaw_rate);
			$display("roll     %d",roll_rate);
			$display("pitch    %d",pitch_rate);
			$display("throttle %d",throttle_rate);
			$display("Motor Rate Actual  Output  %d",motor_1_rate);
			$display("Motor Rate Expected Output %d",m1_expected_output);
			
			if (test_counter == `TEST_LENGTH) begin
				$display("****************Testing Completed****************");
				$display("Tests Run     -	     %d", test_counter);
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
			$display("-   Test Case - 	          %d             -"		, test_counter);
			$display("- INPUTS            rate                     -");
			$display("-   yaw_rate \t\t   -%d -             	     -"	, yaw_rate);
			$display("-   roll_rate \t\t  -%d -           	       -"	, roll_rate);
			$display("-   pitch_rate \t\t -%d -                   -"	, pitch_rate);
			$display("-   throttle_rate -%d -                   -"		, throttle_rate);
			$display("- OUTPUTS                                    -");				  							
			$display("-   Actual Output \t\t   -   %d             -"	, m1_actual_output);	
			$display("-   Expected Output \t\t -   %d             -"	, m1_expected_output);
			$display("----------------------------------------------");
			check_output = `FAIL;
		end
		else
			//$display("Passed Test Number %d", test_num);
			check_output = `PASS;
		end
	endfunction
	
endmodule