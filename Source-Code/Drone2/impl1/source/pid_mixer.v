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
 * @throttle_rate: throttle rate (rad/s) in fixed point 2's complement
 * @yaw_rate: yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate: roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate: pitch rate (rad/s) in fixed point 2's complement
 * @sys_clk: system clock
 */
module pid_mixer #(parameter RATE_BIT_WIDTH = 36,
				   parameter MOTOR_RATE_BIT_WIDTH = 36)
				  (output reg [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate,
				   output reg [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate,
				   output reg [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate,
				   output reg [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate,
				   input [RATE_BIT_WIDTH-1:0] throttle_rate,
				   input [RATE_BIT_WIDTH-1:0] yaw_rate,
				   input [RATE_BIT_WIDTH-1:0] roll_rate,
				   input [RATE_BIT_WIDTH-1:0] pitch_rate,
				   input sys_clk);

	always @(posedge sys_clk) begin
		if (throttle_rate || yaw_rate || roll_rate || pitch_rate) begin
			motor_1_rate <= ~motor_1_rate;
			motor_2_rate <= ~motor_1_rate;
			motor_3_rate <= ~motor_1_rate;
			motor_4_rate <= ~motor_1_rate;
		end
	end

endmodule
