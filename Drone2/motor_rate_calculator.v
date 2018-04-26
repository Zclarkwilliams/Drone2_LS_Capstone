/**
 * motor_rate_calculator - 	takes the motor specific PID outputs and axis scalers and 
 *							calculates the motor rate per motor
 *
 * Parameters
 * 	@RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 * 	@MOTOR_RATE_BIT_WIDTH: Number of bits for motor outputs
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
 * 	@throttle_rate:	throttle rate (rad/s) in fixed point 2's complement
 * 	@yaw_rate: 		yaw rate (rad/s) in fixed point 2's complement
 * 	@roll_rate: 	roll rate (rad/s) in fixed point 2's complement
 * 	@pitch_rate: 	pitch rate (rad/s) in fixed point 2's complement
 * 	@sys_clk: 		system clock
 *
 */
 
 `include "mixer_defs.v"
 
module motor_rate_calculator	#(	parameter 		RATE_BIT_WIDTH = 16,
									parameter 		MOTOR_RATE_BIT_WIDTH = 8)
								(	output	wire	[MOTOR_RATE_BIT_WIDTH-1:0] motor_rate,
									input 	wire	bias_throttle,
									input	wire	throttle_scaler,
									input 	wire	yaw_scaler,
									input 	wire	roll_scaler,
									input 	wire	pitch_scaler,
									input	wire	[RATE_BIT_WIDTH-1:0] throttle_rate,
									input 	wire	[RATE_BIT_WIDTH-1:0] yaw_rate,
									input	wire	[RATE_BIT_WIDTH-1:0] roll_rate,
									input	wire	[RATE_BIT_WIDTH-1:0] pitch_rate,
									input	wire 	sys_clk);
	
	wire					empty;
	wire					go_flag;
	reg	[`OVERFLOW_BITS-1:0]overflow;
	reg	[`STATE_BITS-1:0]	calc_state, n_calc_state;
	reg	[`STATE_BITS-1:0]	call_state, n_call_state;
	reg	[`SUM_BITS-1:0]		sum_1, sum_2, sum_3, final_sum;
	reg	[`PROD_BITS-1:0]	throttle, yaw, roll, pitch;

	always @(posedge sys_clk) begin
		calc_state	<=	n_calc_state;
		call_state	<=	n_call_state;
	end

	always @(*) begin

	end

	generate
		if (call_state == `MULTIPLY) begin
			multiplier mult1(.DataA(throttle_rate), .DataB(throttle_scaler), .Result(throttle));
			n_calc_step 	=	`STEP_2;
			end
		else:	begin
			//	Do Nothing!
			end
	endgenerate
		
	
	/*	Calculating the motor rate from given values and scalers 	*/
	assign motor_rate = {all_done? final_sum : motor_rate};
	
endmodule