/**
 * module pid_mixer - takes finalized rates and converts them to motor rates
 *
 * Parameters
 * @RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 * @MOTOR_RATE_BIT_WIDTH: Number of bits for motor outputs
 *
 * Outputs
 * @motor_1_rate: rate to run motor 1 at (units?)
 * @motor_2_rate: rate to run motor 2 at (units?)
 * @motor_3_rate: rate to run motor 3 at (units?)
 * @motor_4_rate: rate to run motor 4 at (units?)
 *
 * Inputs
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
 *		pid_mixer  #(
 *					 PID_RATE_BIT_WIDTH,
 *					`MOTOR_RATE_BIT_WIDTH)
 *		pid_mixer	(
 *					.sys_clk(sys_clk),
 *					.yaw_rate(yaw_rate),
 *					.roll_rate(roll_rate),
 *					.pitch_rate(pitch_rate),
 *					.throttle_rate(throttle_target_rate),
 *					.motor_1_rate(motor_1_rate),
 *					.motor_2_rate(motor_2_rate),
 *					.motor_3_rate(motor_3_rate),
 *					.motor_4_rate(motor_4_rate));
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
 *		 		   Motor_3 |	   | Motor_4
 *		 		           V       V
 *
 *		Motors_1 and Motor_4 will spin counter clockwise (CCW)
 *		Motors_2 and Motor_3 will spin clockwise (CW)
 *
 *		TODO Check that the motor spin rotation is correct
 *
 *		Referenceing the image above the following equations are generated
 *			Motor_1 = bias + throttle + (-)yaw/2 + (+)roll/2 + (-)pitch/2
 *			Motor_2 = bias + throttle + (+)yaw/2 + (-)roll/2 + (-)pitch/2
 *			Motor_3 = bias + throttle + (+)yaw/2 + (+)roll/2 + (+)pitch/2
 *			Motor_4 = bias + throttle + (-)yaw/2 + (-)roll/2 + (+)pitch/2
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

	localparam MOTOR_MIXER_STATE_BIT_WIDTH = 2;
	localparam [MOTOR_MIXER_STATE_BIT_WIDTH-1:0] STATE_MAP_16_TO_8	= 0,
												 STATE_SEND_OUTPUT	= 1;

	reg	 [MOTOR_MIXER_STATE_BIT_WIDTH-1:0] output_state;

	reg  [BIT_WIDTH-1:0] n_m1_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m2_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m3_yaw_rate;
	reg  [BIT_WIDTH-1:0] n_m4_yaw_rate;

	reg	 [BIT_WIDTH-1:0] n_m1_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m2_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m3_roll_rate;
	reg	 [BIT_WIDTH-1:0] n_m4_roll_rate;

	reg	 [BIT_WIDTH-1:0] n_m1_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m2_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m3_pitch_rate;
	reg	 [BIT_WIDTH-1:0] n_m4_pitch_rate;

	reg	 [BIT_WIDTH-1:0] n_throttle_rate;

	wire [BIT_WIDTH-1:0] motor_1_output;
	wire [BIT_WIDTH-1:0] motor_2_output;
	wire [BIT_WIDTH-1:0] motor_3_output;
	wire [BIT_WIDTH-1:0] motor_4_output;

	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_mapped;
	reg  [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_mapped;

	reg	 [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate_last;
	reg	 [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate_last;
	reg	 [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate_last;
	reg	 [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate_last;

	motor_rate_calculator	#(BIT_WIDTH)
		motor_1_rate_calc	(.rst_n(rst_n),
							 .bias(`MOTOR_RATE_BIAS),
							 .yaw_rate(n_m1_yaw_rate),
							 .roll_rate(n_m1_roll_rate),
							 .pitch_rate(n_m1_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_1_output));
	motor_rate_calculator	#(BIT_WIDTH)
		motor_2_rate_calc	(.rst_n(rst_n),
							 .bias(`MOTOR_RATE_BIAS),
							 .yaw_rate(n_m2_yaw_rate),
							 .roll_rate(n_m2_roll_rate),
							 .pitch_rate(n_m2_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_2_output));
	motor_rate_calculator	#(BIT_WIDTH)
		motor_3_rate_calc	(.rst_n(rst_n),
							 .bias(`MOTOR_RATE_BIAS),
							 .yaw_rate(n_m3_yaw_rate),
							 .roll_rate(n_m3_roll_rate),
							 .pitch_rate(n_m3_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_3_output));
	motor_rate_calculator	#(BIT_WIDTH)
		motor_4_rate_calc	(.rst_n(rst_n),
							 .bias(`MOTOR_RATE_BIAS),
							 .yaw_rate(n_m4_yaw_rate),
							 .roll_rate(n_m4_roll_rate),
							 .pitch_rate(n_m4_pitch_rate),
							 .throttle_rate(n_throttle_rate),
							 .motor_rate(motor_4_output));

	always @(posedge sys_clk or negedge rst_n)begin
		if (!rst_n) begin
			n_m1_yaw_rate			<= `ALL_ZERO_2BYTE;
			n_m4_yaw_rate			<= `ALL_ZERO_2BYTE;
			n_m2_yaw_rate			<= `ALL_ZERO_2BYTE;
			n_m3_yaw_rate			<= `ALL_ZERO_2BYTE;
			n_m1_roll_rate			<= `ALL_ZERO_2BYTE;
			n_m3_roll_rate			<= `ALL_ZERO_2BYTE;
			n_m2_roll_rate			<= `ALL_ZERO_2BYTE;
			n_m4_roll_rate			<= `ALL_ZERO_2BYTE;
			n_m1_pitch_rate			<= `ALL_ZERO_2BYTE;
			n_m2_pitch_rate			<= `ALL_ZERO_2BYTE;
			n_m3_pitch_rate			<= `ALL_ZERO_2BYTE;
			n_m4_pitch_rate			<= `ALL_ZERO_2BYTE;
			n_throttle_rate			<= `ALL_ZERO_2BYTE;
			end
		else begin
			n_m1_yaw_rate			<= (~yaw_rate + 1'b1);
			n_m4_yaw_rate			<= (~yaw_rate + 1'b1);
			n_m2_yaw_rate			<= yaw_rate;
			n_m3_yaw_rate			<= yaw_rate;

			n_m1_roll_rate			<= roll_rate;
			n_m3_roll_rate			<= roll_rate;
			n_m2_roll_rate			<= (~roll_rate + 1'b1);
			n_m4_roll_rate			<= (~roll_rate + 1'b1);

			n_m1_pitch_rate			<= (~pitch_rate + 1'b1);
			n_m2_pitch_rate			<= (~pitch_rate + 1'b1);
			n_m3_pitch_rate			<= pitch_rate;
			n_m4_pitch_rate			<= pitch_rate;

			n_throttle_rate			<= throttle_rate;
			end
		end

	always @(posedge sys_clk) begin
		case(output_state)
			STATE_MAP_16_TO_8: 	begin
				if (motor_1_output != `ALL_ZERO_2BYTE) begin
					
					motor_1_mapped		<= ((motor_1_output + `MOTOR_RATE_ROUND_UP_VAL) >> `MAPPING_SHIFT_8BIT); // Map 16 bit to 8
					motor_2_mapped		<= ((motor_2_output + `MOTOR_RATE_ROUND_UP_VAL) >> `MAPPING_SHIFT_8BIT); // Map 16 bit to 8
					motor_3_mapped		<= ((motor_3_output + `MOTOR_RATE_ROUND_UP_VAL) >> `MAPPING_SHIFT_8BIT); // Map 16 bit to 8
					motor_4_mapped		<= ((motor_4_output + `MOTOR_RATE_ROUND_UP_VAL) >> `MAPPING_SHIFT_8BIT); // Map 16 bit to 8
					
					output_state		<= STATE_SEND_OUTPUT;
					end
				else
					output_state		<= STATE_MAP_16_TO_8;
				end
			STATE_SEND_OUTPUT: 	begin
				if (motor_1_mapped < `MOTOR_VAL_MIN || motor_1_mapped > `MOTOR_VAL_MAX) begin
					motor_1_rate 		<= motor_1_rate_last;
					end
				else begin
					motor_1_rate 		<= motor_1_mapped;
					motor_1_rate_last 	<= motor_1_mapped;
					end
				if (motor_2_mapped < `MOTOR_VAL_MIN || motor_2_mapped > `MOTOR_VAL_MAX) begin
					motor_2_rate 		<= motor_2_rate_last;
					end
				else begin
					motor_2_rate 		<= motor_2_mapped;
					motor_2_rate_last 	<= motor_2_mapped;
					end
				if (motor_3_mapped < `MOTOR_VAL_MIN || motor_3_mapped > `MOTOR_VAL_MAX) begin
					motor_3_rate 		<= motor_3_rate_last;
					end
				else begin
					motor_3_rate 		<= motor_3_mapped;
					motor_3_rate_last 	<= motor_3_mapped;
					end
				if (motor_4_mapped < `MOTOR_VAL_MIN || motor_4_mapped > `MOTOR_VAL_MAX) begin
					motor_4_rate 		<= motor_4_rate_last;
					end
				else begin
					motor_4_rate 		<= motor_4_mapped;
					motor_4_rate_last 	<= motor_4_mapped;
					end
				output_state 			<= STATE_MAP_16_TO_8;
				end
			default begin
				// This state should never be reached! If reached, act as a rst_n signal.
				motor_1_rate			<= `ALL_ZERO_2BYTE;
				motor_2_rate			<= `ALL_ZERO_2BYTE;
				motor_3_rate			<= `ALL_ZERO_2BYTE;
				motor_4_rate			<= `ALL_ZERO_2BYTE;
				motor_1_mapped			<= `ALL_ZERO_2BYTE;
				motor_2_mapped			<= `ALL_ZERO_2BYTE;
				motor_3_mapped			<= `ALL_ZERO_2BYTE;
				motor_4_mapped			<= `ALL_ZERO_2BYTE;
				motor_1_rate_last		<= `ALL_ZERO_2BYTE;
				motor_2_rate_last		<= `ALL_ZERO_2BYTE;
				motor_3_rate_last		<= `ALL_ZERO_2BYTE;
				motor_4_rate_last		<= `ALL_ZERO_2BYTE;
				output_state			<= STATE_MAP_16_TO_8;
				end
			endcase
		end
endmodule
