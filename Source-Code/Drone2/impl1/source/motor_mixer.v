/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * module motor_mixer - takes finalized rates and converts them to motor rates
 *
 * Outputs:
 * @motor_1_rate: rate to run motor 1
 * @motor_2_rate: rate to run motor 2
 * @motor_3_rate: rate to run motor 3
 * @motor_4_rate: rate to run motor 4
 *
 * Inputs:
 * @resetn:			system reset
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
 *		motor_mixer	(.resetn(resetn),
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

module motor_mixer (
	input  wire resetn,
	input  wire sys_clk,
	input wire signed [`RATE_BIT_WIDTH-1:0] yaw_rate,
	input wire signed [`RATE_BIT_WIDTH-1:0] roll_rate,
	input wire signed [`RATE_BIT_WIDTH-1:0] pitch_rate,
	input wire signed [`RATE_BIT_WIDTH-1:0] throttle_rate,
	output reg  [`MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate,
	output reg  [`MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate,
	output reg  [`MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate,
	output reg  [`MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate);

	/*	Params for states and state size	*/
	localparam STATE_BIT_WIDTH = 2;
	localparam [STATE_BIT_WIDTH-1:0]
			STATE_CONVERT_RATES 	= 0,
			STATE_MOTOR_RATE_CALC 	= 1,
			STATE_BOUNDARY_CHECK	= 2,
			STATE_SEND_OUTPUT		= 3;

	reg	 [STATE_BIT_WIDTH-1:0] motor_mixer_state;

	/*	Motor specific variables per axis	*/
	reg signed [`RATE_BIT_WIDTH-1:0] yaw_scale;
	reg signed [`RATE_BIT_WIDTH-1:0] roll_scale;
	reg signed [`RATE_BIT_WIDTH-1:0] pitch_scale;
	reg	signed [`RATE_BIT_WIDTH-1:0] n_throttle_rate;

	reg signed [`RATE_BIT_WIDTH-1:0] motor_1_output;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_2_output;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_3_output;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_4_output;

	reg signed [`RATE_BIT_WIDTH-1:0] motor_1_temp;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_2_temp;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_3_temp;
	reg signed [`RATE_BIT_WIDTH-1:0] motor_4_temp;

	always @(posedge sys_clk or negedge resetn) begin
		if (!resetn) begin //On reset input LOW set all variables to zero
			yaw_scale					<= `ALL_ZERO_2BYTE;
			roll_scale					<= `ALL_ZERO_2BYTE;
			pitch_scale					<= `ALL_ZERO_2BYTE;
			n_throttle_rate				<= `ALL_ZERO_2BYTE;

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

			motor_mixer_state			<= STATE_SCALE_RATES;
		end
		else begin
			case(motor_mixer_state)
				STATE_SCALE_RATES: 	begin	// Get the value rates input and scale them in half for later arithmetic
					yaw_scale			<= (yaw_rate   >>> `MOTOR_RATE_YAW_SCALER);
					roll_scale			<= (roll_rate  >>> `MOTOR_RATE_ROLL_SCALER);
					pitch_scale			<= (pitch_rate >>> `MOTOR_RATE_PITCH_SCALER);
					//	Throttle does not get scaled because it is equal across all motors
					motor_mixer_state	<= STATE_MOTOR_RATE_CALC;
				end
				STATE_MOTOR_RATE_CALC:  begin
					motor_1_output		<= `MOTOR_1_RATE_BIAS + n_throttle_rate - yaw_scale + roll_scale + pitch_scale;
					motor_2_output		<= `MOTOR_2_RATE_BIAS + n_throttle_rate + yaw_scale - roll_scale + pitch_scale;
					motor_3_output		<= `MOTOR_3_RATE_BIAS + n_throttle_rate - yaw_scale - roll_scale - pitch_scale;
					motor_4_output		<= `MOTOR_4_RATE_BIAS + n_throttle_rate + yaw_scale + roll_scale - pitch_scale;
					motor_mixer_state 	<= STATE_BOUNDARY_CHECK;
				end
				STATE_BOUNDARY_CHECK: 	begin // Test to see if motor_#_output is wwithin reasonable range for flight
					if (n_throttle_rate <= `MOTOR_VAL_MIN) begin
						// If we don't have throttle input, we don't want to fire off any motors so set all to ZERO
						motor_1_temp	<= `ALL_ZERO_2BYTE;
						motor_2_temp	<= `ALL_ZERO_2BYTE;
						motor_3_temp	<= `ALL_ZERO_2BYTE;
						motor_4_temp	<= `ALL_ZERO_2BYTE;
					end
					else begin
						if (motor_1_output < `MOTOR_VAL_MIN)
							motor_1_temp <= `MOTOR_VAL_MIN;
						else if (motor_1_output > `MOTOR_VAL_MAX)
							motor_1_temp <= `MOTOR_VAL_MAX;
						else
							motor_1_temp <= motor_1_output;

						if (motor_2_output < `MOTOR_VAL_MIN)
							motor_2_temp <= `MOTOR_VAL_MIN;
						else if (motor_2_output > `MOTOR_VAL_MAX)
							motor_2_temp <= `MOTOR_VAL_MAX;
						else
							motor_2_temp <= motor_2_output;

						if (motor_3_output < `MOTOR_VAL_MIN)
							motor_3_temp <= `MOTOR_VAL_MIN;
						else if (motor_3_output > `MOTOR_VAL_MAX)
							motor_3_temp <= `MOTOR_VAL_MAX;
						else
							motor_3_temp <= motor_3_output;

						if (motor_4_output < `MOTOR_VAL_MIN)
							motor_4_temp <= `MOTOR_VAL_MIN;
						else if (motor_4_output > `MOTOR_VAL_MAX)
							motor_4_temp <= `MOTOR_VAL_MAX;
						else
							motor_4_temp <= motor_4_output;
					end

					motor_mixer_state <= STATE_SEND_OUTPUT;

				end
				STATE_SEND_OUTPUT: 	begin	//	Reduce the motor_rates to 8 bit for pwm_generator use
					motor_1_rate		<= motor_1_temp[11:4];
					motor_2_rate		<= motor_2_temp[11:4];
					motor_3_rate		<= motor_3_temp[11:4];
					motor_4_rate		<= motor_4_temp[11:4];
					motor_mixer_state 	<= STATE_SCALE_RATES;
				end
				default begin
					// This state should never be reached! If reached, act as a resetn signal.
					motor_1_rate		<= `ALL_ZERO_2BYTE;
					motor_2_rate		<= `ALL_ZERO_2BYTE;
					motor_3_rate		<= `ALL_ZERO_2BYTE;
					motor_4_rate		<= `ALL_ZERO_2BYTE;
					motor_mixer_state	<= STATE_SCALE_RATES;
				end
			endcase
		end
	end
endmodule
