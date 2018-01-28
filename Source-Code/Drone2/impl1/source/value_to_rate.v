/**
 * module value_to_rate - map the input values (-9000 to 9000) into corresponding
 *						  rates in rad/s.  The rates are represented as fixed point
 *						  2's compelement.
 *
 * Parameters
 * @VAL_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs
 * @RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch rate outputs
 *
 * Outputs
 * @throttle_rate: throttle rate (rad/s) in fixed point 2's complement
 * @yaw_rate: yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate: roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate: pitch rate (rad/s) in fixed point 2's complement
 *
 * Inputs
 * @throttle_val: throttle value from -9000 to 9000
 * @yaw_val: yaw value from -9000 to 9000
 * @roll_val: roll value from -9000 to 9000
 * @pitch_val: pitch value from -9000 to 9000
 * @sys_clk: system clock
 */
module value_to_rate #(parameter RATE_BIT_WIDTH = 36,
					   parameter VAL_BIT_WIDTH = 14)
					  (output reg [RATE_BIT_WIDTH-1:0] throttle_rate,
					   output reg [RATE_BIT_WIDTH-1:0] yaw_rate,
					   output reg [RATE_BIT_WIDTH-1:0] roll_rate,
					   output reg [RATE_BIT_WIDTH-1:0] pitch_rate,
					   input [VAL_BIT_WIDTH-1:0] throttle_val,
					   input [VAL_BIT_WIDTH-1:0] yaw_val,
					   input [VAL_BIT_WIDTH-1:0] roll_val,
					   input [VAL_BIT_WIDTH-1:0] pitch_val,
					   input sys_clk);

	always @(posedge sys_clk) begin
		if (throttle_val || yaw_val || roll_val || pitch_val) begin
			throttle_rate <= ~throttle_rate;
			yaw_rate <= ~yaw_rate;
			roll_rate <= ~roll_rate;
			pitch_rate <= ~pitch_rate;
		end
	end
endmodule
