/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * module motor_mixer - takes finalized rates and converts them to motor rates
 *
 * Parameters:
 * @RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 * @MOTOR_RATE_BIT_WIDTH: Number of bits for motor outputs
 *
 * Outputs:
 * @motor_1_rate: rate to run motor 1
 * @motor_2_rate: rate to run motor 2
 * @motor_3_rate: rate to run motor 3
 * @motor_4_rate: rate to run motor 4
 *
 * Inputs:
 * @rst_n:			system reset
 * @sys_clk: 		system clock
 * @yaw_rate: 		yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate: 		roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate: 	pitch rate (rad/s) in fixed point 2's complement
 * @throttle_rate:	throttle rate (rad/s) in fixed point 2's complement
 *		^^^ NOTE: Inputs rates expected to be formated as follows
 *					[15:0] rate_input = [15:4] IntegerPart . [3:0] DecimalPart
 *
 *	Top level (Drone2.v) instantiation of module
 *
 *		motor_mixer  #(BIT_WIDTH,
 *					   MOTOR_RATE_BIT_WIDTH)
 *		motor_mixer	(.rst_n(resetn),
 *					 .sys_clk(sys_clk),
 *					 .yaw_rate(yaw_rate),
 *					 .roll_rate(roll_rate),
 *					 .pitch_rate(pitch_rate),
 *					 .throttle_rate(throttle_rate),
 *					 .motor_1_rate(motor_1_rate),
 *					 .motor_2_rate(motor_2_rate),
 *					 .motor_3_rate(motor_3_rate),
 *					 .motor_4_rate(motor_4_rate));
 *
 *		 		   | <-----			 -----> |
 *		 		   | Motor_1		Motor_2 |
 *		 		   V 	\			  /     V
 *		 				 \			 /
 *		 				  \			/
 *		 				   /-------\
 *		 				   | DRONE |
 *		 				   | 	   |
 *		 				   \-------/
 *						  /		    \	
 *						 /			 \
 *						/			  \
 *		 		    -----> | 	   | <-----
 *		 		   Motor_4 |	   | Motor_3
 *		 		           V       V
 *
 *		Motors_1 and Motor_4 will spin clockwise (CW)
 *		Motors_2 and Motor_3 will spin counter clockwise (CCW)
 *
 *		TODO Check that the motor spin rotation is correct
 *
 *		Referenceing the image above the following equations are generated
 *			Motor_1 = bias + throttle + (-)yaw/2 + (+)roll/2 + (+)pitch/2
 *			Motor_2 = bias + throttle + (+)yaw/2 + (-)roll/2 + (+)pitch/2
 *			Motor_3 = bias + throttle + (-)yaw/2 + (-)roll/2 + (-)pitch/2
 *			Motor_4 = bias + throttle + (+)yaw/2 + (+)roll/2 + (-)pitch/2
 *
 */

`timescale 1ns / 1ns
`include "common_defines.v"

module motor_mixer	#(parameter BIT_WIDTH = 16,
					  parameter MOTOR_RATE_BIT_WIDTH = 8)
					(input  wire rst_n,
					 input  wire sys_clk,
					 input wire signed [BIT_WIDTH-1:0] yaw_rate,
					 input wire signed [BIT_WIDTH-1:0] roll_rate,
					 input wire signed [BIT_WIDTH-1:0] pitch_rate,
					 input wire signed [BIT_WIDTH-1:0] throttle_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate,
					 output reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate);

	/*	Params for states and state size	*/
	localparam STATE_BIT_WIDTH = 3;
	localparam [STATE_BIT_WIDTH-1:0]
			STATE_CONVERT_RATES 	= 0,
			STATE_SCALE_RATES 		= 1,
			STATE_MOTOR_RATE_CALC 	= 2,
			STATE_BOUNDARY_CHECK	= 3,
			STATE_SEND_OUTPUT		= 4;

	reg	 [STATE_BIT_WIDTH-1:0] motor_mixer_state;

	/*	Motor specific variables per axis	*/
	reg  [BIT_WIDTH-1:0] yaw_scale;
	reg  [BIT_WIDTH-1:0] roll_scale;
	reg  [BIT_WIDTH-1:0] pitch_scale;
	reg	 [BIT_WIDTH-1:0] n_throttle_rate;

	reg  [BIT_WIDTH-1:0] m1_yaw_rate;
	reg  [BIT_WIDTH-1:0] m2_yaw_rate;
	reg  [BIT_WIDTH-1:0] m3_yaw_rate;
	reg  [BIT_WIDTH-1:0] m4_yaw_rate;

	reg	 [BIT_WIDTH-1:0] m1_roll_rate;
	reg	 [BIT_WIDTH-1:0] m2_roll_rate;
	reg	 [BIT_WIDTH-1:0] m3_roll_rate;
	reg	 [BIT_WIDTH-1:0] m4_roll_rate;

	reg	 [BIT_WIDTH-1:0] m1_pitch_rate;
	reg	 [BIT_WIDTH-1:0] m2_pitch_rate;
	reg	 [BIT_WIDTH-1:0] m3_pitch_rate;
	reg	 [BIT_WIDTH-1:0] m4_pitch_rate;

	reg [BIT_WIDTH-1:0] motor_1_output;
	reg [BIT_WIDTH-1:0] motor_2_output;
	reg [BIT_WIDTH-1:0] motor_3_output;
	reg [BIT_WIDTH-1:0] motor_4_output;

	reg [BIT_WIDTH-1:0] motor_1_temp;
	reg [BIT_WIDTH-1:0] motor_2_temp;
	reg [BIT_WIDTH-1:0] motor_3_temp;
	reg [BIT_WIDTH-1:0] motor_4_temp;

	always @(posedge sys_clk or negedge rst_n) begin
		if (!rst_n) begin //On reset input LOW set all variables to zero
			yaw_scale					<= `ALL_ZERO_2BYTE;
			roll_scale					<= `ALL_ZERO_2BYTE;
			pitch_scale					<= `ALL_ZERO_2BYTE;
			n_throttle_rate				<= `ALL_ZERO_2BYTE;

			m1_yaw_rate					<= `ALL_ZERO_2BYTE;
			m2_yaw_rate					<= `ALL_ZERO_2BYTE;
			m3_yaw_rate					<= `ALL_ZERO_2BYTE;
			m4_yaw_rate					<= `ALL_ZERO_2BYTE;

			m1_roll_rate				<= `ALL_ZERO_2BYTE;
			m2_roll_rate				<= `ALL_ZERO_2BYTE;
			m3_roll_rate				<= `ALL_ZERO_2BYTE;
			m4_roll_rate				<= `ALL_ZERO_2BYTE;

			m1_pitch_rate				<= `ALL_ZERO_2BYTE;
			m2_pitch_rate				<= `ALL_ZERO_2BYTE;
			m3_pitch_rate				<= `ALL_ZERO_2BYTE;
			m4_pitch_rate				<= `ALL_ZERO_2BYTE;

			motor_1_output				<= `ALL_ZERO_2BYTE;
			motor_2_output				<= `ALL_ZERO_2BYTE;
			motor_3_output				<= `ALL_ZERO_2BYTE;
			motor_4_output				<= `ALL_ZERO_2BYTE;

			motor_1_temp				<= `ALL_ZERO_2BYTE;
			motor_2_temp				<= `ALL_ZERO_2BYTE;
			motor_3_temp				<= `ALL_ZERO_2BYTE;
			motor_4_temp				<= `ALL_ZERO_2BYTE;

			motor_1_rate				<= `ALL_ZERO_2BYTE;
			motor_2_rate				<= `ALL_ZERO_2BYTE;
			motor_3_rate				<= `ALL_ZERO_2BYTE;
			motor_4_rate				<= `ALL_ZERO_2BYTE;

			motor_mixer_state			<= STATE_GET_RATES;
		end
		else begin
			case(motor_mixer_state)
				STATE_SCALE_RATES: 	begin	// Get the value rates input and scale them in half for later arithmetic
					yaw_scale			<= (yaw_rate   >>> `MOTOR_RATE_YAW_SCALER);
					roll_scale			<= (roll_rate  >>> `MOTOR_RATE_ROLL_SCALER);
					pitch_scale			<= (pitch_rate >>> `MOTOR_RATE_PITCH_SCALER);
					//	Throttle does not get scaled because it is equal across all motors
					motor_mixer_state	<= STATE_SET_RATES;
				end
				STATE_CONVERT_RATES: 	begin
				// If we don't have throttle input, we don't want to fire off any motors
					if (throttle_rate < `MOTOR_VAL_MIN) begin
						m1_yaw_rate		<= `ALL_ZERO_2BYTE;
						m2_yaw_rate		<= `ALL_ZERO_2BYTE;
						m3_yaw_rate		<= `ALL_ZERO_2BYTE;
						m4_yaw_rate		<= `ALL_ZERO_2BYTE;
						m1_roll_rate	<= `ALL_ZERO_2BYTE;
						m3_roll_rate	<= `ALL_ZERO_2BYTE;
						m2_roll_rate	<= `ALL_ZERO_2BYTE;
						m4_roll_rate	<= `ALL_ZERO_2BYTE;
						m1_pitch_rate	<= `ALL_ZERO_2BYTE;
						m2_pitch_rate	<= `ALL_ZERO_2BYTE;
						m3_pitch_rate	<= `ALL_ZERO_2BYTE;
						m4_pitch_rate	<= `ALL_ZERO_2BYTE;
						n_throttle_rate	<= `ALL_ZERO_2BYTE;
					end
					else begin
						//	Set up motor_1 rates for only addition
						m1_yaw_rate		<= (~yaw_scale + 1'b1);
						m1_roll_rate	<= roll_scale;
						m1_pitch_rate	<= pitch_scale;
						//	Set up motor_2 rates for only addition
						m2_yaw_rate		<= yaw_scale;
						m2_roll_rate	<= (~roll_scale + 1'b1);
						m2_pitch_rate	<= pitch_scale;
						//	Set up motor_3 rates for only addition
						m3_yaw_rate		<= (~yaw_scale + 1'b1);
						m3_roll_rate	<= (~roll_scale + 1'b1);
						m3_pitch_rate	<= (~pitch_scale + 1'b1);
						//	Set up motor_4 rates for only addition
						m4_yaw_rate		<= yaw_scale;
						m4_roll_rate	<= roll_scale;
						m4_pitch_rate	<= (~pitch_scale + 1'b1);
						//	Throttle will be identicle to all motors
						n_throttle_rate	<= throttle_rate;
					end
					motor_mixer_state	<= STATE_MOTOR_RATE_CALC;
				end
				STATE_MOTOR_RATE_CALC:  begin
					//	Motor 1 equation motor_rate = bias + throttle - yaw/2 + roll/2 + pitch/2
					motor_1_output		<= `MOTOR_1_RATE_BIAS + n_throttle_rate + m1_yaw_rate + m1_roll_rate + m1_pitch_rate;
					//	Motor 2 equation motor_rate = bias + throttle + yaw/2 - roll/2 + pitch/2
					motor_2_output		<= `MOTOR_2_RATE_BIAS + n_throttle_rate + m2_yaw_rate + m2_roll_rate + m2_pitch_rate;
					//	Motor 3 equation motor_rate = bias + throttle - yaw/2 - roll/2 - pitch/2
					motor_3_output		<= `MOTOR_3_RATE_BIAS + n_throttle_rate + m3_yaw_rate + m3_roll_rate + m3_pitch_rate;
					//	Motor 4 equation motor_rate = bias + throttle + yaw/2 + roll/2 - pitch/2
					motor_4_output		<= `MOTOR_4_RATE_BIAS + n_throttle_rate + m4_yaw_rate + m4_roll_rate + m4_pitch_rate;
					motor_mixer_state 	<= STATE_BOUNDARY_CHECK;
				end
				STATE_BOUNDARY_CHECK: 	begin // Test to see if motor_#_output is wwithin reasonable range for flight
					if ($signed(motor_1_output) < $signed(`MOTOR_VAL_MIN))
						motor_1_temp	<= `MOTOR_VAL_MIN;
					else if ($signed(motor_1_output) > $signed(`MOTOR_VAL_MAX))
						motor_1_temp	<= `MOTOR_VAL_MAX;
					else
						motor_1_temp	<= motor_1_output;

					if ($signed(motor_2_output) < $signed(`MOTOR_VAL_MIN))
						motor_2_temp	<= `MOTOR_VAL_MIN;
					else if ($signed(motor_2_output) > $signed(`MOTOR_VAL_MAX))
						motor_2_temp	<= `MOTOR_VAL_MAX;
					else
						motor_2_temp	<= motor_2_output;

					if ($signed(motor_3_output) < $signed(`MOTOR_VAL_MIN))
						motor_3_temp	<= `MOTOR_VAL_MIN;
					else if ($signed(motor_3_output) > $signed(`MOTOR_VAL_MAX))
						motor_3_temp	<= `MOTOR_VAL_MAX;
					else
						motor_3_temp	<= motor_3_output;

					if ($signed(motor_4_output) < $signed(`MOTOR_VAL_MIN))
						motor_4_temp	<= `MOTOR_VAL_MIN;
					else if ($signed(motor_4_output) > $signed(`MOTOR_VAL_MAX))
						motor_4_temp	<= `MOTOR_VAL_MAX;
					else
						motor_4_temp	<= motor_4_output;

					motor_mixer_state	<= STATE_SEND_OUTPUT;

				end
				STATE_SEND_OUTPUT: 	begin	//	Reduce the motor_rates to 8 bit for pwm_generator use
					motor_1_rate		<= motor_1_temp[9:2];
					motor_2_rate		<= motor_2_temp[9:2];
					motor_3_rate		<= motor_3_temp[9:2];
					motor_4_rate		<= motor_4_temp[9:2];
					motor_mixer_state 	<= STATE_GET_RATES;
				end
				default begin
					// This state should never be reached! If reached, act as a rst_n signal.
					motor_1_rate		<= `ALL_ZERO_2BYTE;
					motor_2_rate		<= `ALL_ZERO_2BYTE;
					motor_3_rate		<= `ALL_ZERO_2BYTE;
					motor_4_rate		<= `ALL_ZERO_2BYTE;
					motor_mixer_state	<= STATE_GET_RATES;
				end
			endcase
		end
	end
endmodule
