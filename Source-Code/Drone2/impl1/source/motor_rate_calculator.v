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
 *	Rate Values -
 * 	@throttle_rate:	(+)   throttle rate (rad/s) in fixed point 2's complement
 * 	@yaw_rate: 		(+/-) yaw rate (rad/s) in fixed point 2's complement
 * 	@roll_rate: 	(+/-) roll rate (rad/s) in fixed point 2's complement
 * 	@pitch_rate: 	(+/-) pitch rate (rad/s) in fixed point 2's complement
 * 	@sys_clk: 		system clock
 *
 */
`timescale 1ns / 1ns
`include "common_defines.v"

module motor_rate_calculator	#(	parameter 		BIT_WIDTH = 16)
								 (	//Inputs
									input	wire	rst_n,
									input 	wire	bias,
									input wire signed [BIT_WIDTH-1:0] yaw_rate,
									input wire signed [BIT_WIDTH-1:0] roll_rate,
									input wire signed [BIT_WIDTH-1:0] pitch_rate,
									input wire signed [BIT_WIDTH-1:0] throttle_rate,
									//	Outputs
									output	reg 	[BIT_WIDTH-1:0] motor_rate);

	reg [BIT_WIDTH-1:0]	yaw;
	reg [BIT_WIDTH-1:0]	roll;
	reg [BIT_WIDTH-1:0]	pitch;
	reg [BIT_WIDTH-1:0]	throttle;

/**********************************************************************************************************************\
|	Calculation
|		Motor_Rate 	= 	Bias
|						+ throttle
|						+ yaw_rate / 2
|						+ roll_rate / 2
|						+ pitch_rate / 2
\**********************************************************************************************************************/

//	Multiplication stage 	- get the product for each axis rate value scaled by axis scale
	always @ (*) begin
		if (!rst_n) begin
			yaw   		= `ALL_ZERO_2BYTE;
			roll  		= `ALL_ZERO_2BYTE;
			pitch    	= `ALL_ZERO_2BYTE;
			throttle	= `ALL_ZERO_2BYTE;
			end
		else begin
			yaw   		= (yaw_rate   >>> `MOTOR_RATE_YAW_SCALER);
			roll  		= (roll_rate  >>> `MOTOR_RATE_ROLL_SCALER);
			pitch    	= (pitch_rate >>> `MOTOR_RATE_PITCH_SCALER);
			throttle	= throttle_rate;
			end
		end

	//	Summation stage 	 	- sum the scaled rate values and the bias
	//	Output the found value  - as long as rst_n is not asserted, and final_sum is not zero
	always @ (bias or yaw or roll or pitch or throttle) begin
			motor_rate	= bias + yaw + roll + pitch + throttle;
		end
endmodule
