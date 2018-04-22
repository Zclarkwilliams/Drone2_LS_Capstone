/**
 * motor_rate_calculator - 	takes the motor specific PID outputs and axis scalers and 
 *							calculates the motor rate per motor. Each input will be entered
 * 							either as a positive or negiative 16 bit 2's compliment depending
 *							on the motor position being calculated.
 * 
 * Equation - motor_rate = throttle_scaler * throttle_rate
 *						   + yaw_scaler * yaw_rate
 *						   + roll_scaler * roll_rate
 *						   + pitch_scaler * pitch_rate
 *
 * Parameters
 * 	@BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 *
 * Outputs
 * 	@motor_rate: rate to run instantiating motor at (units?)
 *
 * Inputs
 * 	Scalers - 
 *	@scaler_throttle:	scaler specific to throttle contribution to total motor rate
 *	@scaler_yaw:		scaler specific to yaw contribution to total motor rate
 *	@scaler_roll:		scaler specific to roll contribution to total motor rate
 *	@scaler_pitch:		scaler specific to pitch contribution to total motor rate
 *	Rate Values - 		
 * 	@throttle_rate:	(+)   throttle rate (rad/s) in fixed point 2's complement
 * 	@yaw_rate: 		(+/-) yaw rate (rad/s) in fixed point 2's complement
 * 	@roll_rate: 	(+/-) roll rate (rad/s) in fixed point 2's complement
 * 	@pitch_rate: 	(+/-) pitch rate (rad/s) in fixed point 2's complement
 * 	@sys_clk: 		system clock
 *
 */
`timescale 1ns / 1ns
 `include "pid_mixer_defs.v"
 
module motor_rate_calculator	#(	parameter 		BIT_WIDTH = 16)
								(	//Inputs
									input	wire	rst_n,
									input 	wire	bias,
									input 	wire	[BIT_WIDTH-1:0] yaw_rate,
									input	wire	[BIT_WIDTH-1:0] roll_rate,
									input	wire	[BIT_WIDTH-1:0] pitch_rate,
									input	wire	[BIT_WIDTH-1:0] throttle_rate,
									//	Outputs
									output	reg 	[BIT_WIDTH-1:0] motor_rate);

	wire [BIT_WIDTH-1:0]	yaw;  
	wire [BIT_WIDTH-1:0]	roll; 
	wire [BIT_WIDTH-1:0]	pitch;
	wire [BIT_WIDTH-1:0]	throttle; 

/**********************************************************************************************************************\
|	Calculation
|		Motor_Rate 	= 	Bias|						+ throttle
|						+ yaw_rate / 2
|						+ roll_rate / 2 
|						+ pitch_rate / 2
|
\**********************************************************************************************************************/
	
	//	Multiplication stage - get the product for each axis rate value scaled by axis scale

	//	Summation stage - sum the scaled rate values and the bias
	//	Output the found value - as long as rst_n is not asserted, and final_sum is not zero				
	assign yaw   	= (yaw_rate   >> `DIVIDE_SCALER); 
	assign roll  	= (roll_rate  >> `DIVIDE_SCALER); 
	assign pitch    = (pitch_rate >> `DIVIDE_SCALER);
	assign throttle = throttle_rate;
		
	always @ (negedge rst_n or yaw or roll or pitch or throttle) begin		
		if (!rst_n)
			motor_rate = `ZERO;
		else begin
			motor_rate = bias + yaw + roll + pitch + throttle;
			$display("T %d", throttle);
			$display("T %d", yaw);
			$display("T %d", roll);
			$display("T %d", pitch);
			$display("----------");
			$display("A %d", motor_rate);
			$display("    ");
			end
	end
endmodule