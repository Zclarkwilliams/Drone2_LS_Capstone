/**
 * module value_to_rate - map the input values (-9000 to 9000) into corresponding
 *						  rates in rad/s.  The rates are represented as fixed point
 *						  2's compelement.
 *
 * Parameters
 * @N_VAL: Number of bits for throttle/yaw/roll/pitch val outputs
 * @N_AUX: Number of bits for aux1/aux2 val outputs
 * @N_MODE: Number of bits for the mode output
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
 module value_to_rate #(parameter N_RATE = 36,
					    parameter N_VAL = 14)
					   (output reg [N_RATE-1:0] throttle_rate,
					    output reg [N_RATE-1:0] yaw_rate,
					    output reg [N_RATE-1:0] roll_rate,
					    output reg [N_RATE-1:0] pitch_rate,
					    input [N_VAL-1:0] throttle_val,
					    input [N_VAL-1:0] yaw_val,
					    input [N_VAL-1:0] roll_val,
					    input [N_VAL-1:0] pitch_val,
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
