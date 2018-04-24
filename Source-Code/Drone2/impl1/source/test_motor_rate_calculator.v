`timescale 1ns / 1ps

`define PASS 	1'b0
`define FAIL	1'b1
`define TRUE 	1'b1
`define FALSE	1'b0


module tb_motor_rate_calculator;
	parameter 	BIT_WIDTH = 16;
	parameter 	MOTOR_RATE_BIT_WIDTH = 16;
	localparam	scale_diviser = 1'b1;
	
	wire 	sys_clk;
	
	wire	[MOTOR_RATE_BIT_WIDTH-1:0] motor_rate_output;
	
	reg		[BIT_WIDTH-1:0] yaw_rate = 0;
	reg		[BIT_WIDTH-1:0] roll_rate = 0;
	reg 	[BIT_WIDTH-1:0] pitch_rate = 0;
	reg		[BIT_WIDTH-1:0] throttle_rate = 0;
	
	reg		rst_n;
	
	reg		bias;
	reg		[BIT_WIDTH-1:0] yaw_scaler;
	reg		[BIT_WIDTH-1:0] roll_scaler;
	reg		[BIT_WIDTH-1:0] pitch_scaler;
	reg		[BIT_WIDTH-1:0] throttle_scaler;
	
	reg		exit_flag  = `FALSE;	 
	reg		yaw_val;
	reg		roll_val;
	reg		pitch_val;
	reg		throttle_val;
	reg		check_return;
	reg		[2:0]  counter = 0;		 
	reg		[15:0] test_counter = 0;
	reg		[31:0] pass_counter = 0;
	reg 	[31:0] fail_counter = 0;
	reg		[MOTOR_RATE_BIT_WIDTH-1:0] expected_output;
	
//clock clock1(.clk(sys_clk));

motor_rate_calculator	#(	BIT_WIDTH)
		rate_calc_DUT	(	// Inputs
							.bias(bias),
							.rst_n(rst_n),
							// Sending YAW Values
							.yaw_rate(yaw_rate),
							// Sending ROLL Values
							.roll_rate(roll_rate),
							// Sending PITCH Values
							.pitch_rate(pitch_rate),
							// Sending THROTTLE Values
							.throttle_rate(throttle_rate),
							// Output -> Motor_Rate Value Calculated
							.motor_rate(motor_rate_output));
	
	initial begin
		exit_flag 				<= `FALSE;
		rst_n 					<= 	1;
		bias					<=	0;
	//	end	 
	
	//always @(sys_clk)begin
		$display("Test Begining.");
		
		while (exit_flag == `FALSE) begin
			//$display("Testing......");
			if (counter == 0) begin 
				bias 			<=	0;
				end
			else if (counter == 1) begin 
				bias 			<=	0;
				end
			else if (counter == 2) begin
				rst_n			<=	0;
				bias 			<=	1;
				end
				
			#1
			if (!rst_n)
				expected_output =	0;
			else
				expected_output	=	(bias +
									 throttle_rate +
									 (yaw_rate >> 1) +
									 (roll_rate >> 1) +
									 (pitch_rate >> 1));
			
			#5
			
			$display("FFFFFFF    %d", expected_output);
			
			test_counter 	  = test_counter + 1;
			
			assign check_return = check_output	(fail_counter,
												 test_counter,
						 						 yaw_rate,
												 roll_rate,
												 pitch_rate,
												 throttle_rate,
												 expected_output, 
												 motor_rate_output);
			//$display("',',',',' check_return val - %d ',',',','", check_return);
			if (check_return == `FAIL)
				fail_counter	= fail_counter + 1;
			else
				pass_counter	= pass_counter  + 1;
			
			if (throttle_rate == 16'hFF) begin 
				counter 		<= counter + 1;
				yaw_rate		<= 16'h00;
				roll_rate		<= 16'h00;
				pitch_rate		<= 16'h00;
				throttle_rate	<= 16'h00;
				end
			else begin
				yaw_rate		<= yaw_rate + 1;
				roll_rate		<= roll_rate + 1;
				pitch_rate		<= pitch_rate + 1;
				throttle_rate	<= throttle_rate + 1;
				end
			
			if (counter == 2 && throttle_rate == 16'hFF) begin
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
							 input [15:0] test_counter,	
							 input [15:0] yaw_rate,
							 input [15:0] roll_rate,
							 input [15:0] pitch_rate,
							 input [15:0] throttle_rate,
							 input [15:0] expected_output, 
					  		 input [15:0] actual_output);						   
		begin					  
		if (expected_output != actual_output) begin
			$display("------------------Error Log-------------------");
			$display("----------------------------------------------");
			$display("-   Number of Errors -%d             -" 			, fail_counter);
			$display("-   Test Case - 	          %d             -"		, test_counter);
			$display("- INPUTS            rate                     -");
			$display("-   yaw_rate \t\t   -%d -             	     -"	, yaw_rate);
			$display("-   roll_rate \t\t  -%d -           	       -"	, roll_rate);
			$display("-   pitch_rate \t\t -%d -                   -"	, pitch_rate);
			$display("-   throttle_rate -%d -                   -"		, throttle_rate);
			$display("- OUTPUTS                                    -");				  							
			$display("-   Actual Output \t\t   -   %d             -"	, actual_output);	
			$display("-   Expected Output \t\t -   %d             -"	, expected_output);
			$display("----------------------------------------------");
			check_output = `FAIL;
		end
		else
			//$display("Passed Test Number %d", test_counter);
			check_output = `PASS;
		end
	endfunction
endmodule