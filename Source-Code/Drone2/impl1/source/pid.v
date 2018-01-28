/**
 * module pid - top level pid module that takes in target rates (rad/s) represented
 *				and velocity/rotation vectors represented in 2's compelement fixed
 *				point.
 *
 * Parameters
 * @RATE_BIT_WIDTH: Number of bits for throttle/yaw/roll/pitch val inputs/outputs
 * @VELOCITY_BIT_WIDTH: Number of bits for velocity inputs
 * @ROTATION_BIT_WIDTH: Number of bits for rotation inputs
 *
 * Outputs
 * @yaw_rate_out: yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate_out: roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate_out: pitch rate (rad/s) in fixed point 2's complement
 *
 * Inputs
 * @yaw_rate_in: yaw rate (rad/s) in fixed point 2's complement
 * @roll_rate_in: roll rate (rad/s) in fixed point 2's complement
 * @pitch_rate_in: pitch rate (rad/s) in fixed point 2's complement
 * @x_velocity: velocity vector in the x-direction
 * @y_velocity: velocity vector in the y-direction
 * @z_velocity: velocity vector in the z-direction
 * @x_rotation: rotation vector in the x-direction
 * @y_rotation: rotation vector in the y-direction
 * @z_rotation: rotation vector in the z-direction
 * @sys_clk: system clock
 */
 module pid #(parameter RATE_BIT_WIDTH = 36,
 			  parameter VELOCITY_BIT_WIDTH = 36,
 			  parameter ROTATION_BIT_WIDTH = 36)
 			 (output reg [RATE_BIT_WIDTH-1:0] yaw_rate_out,
 			  output reg [RATE_BIT_WIDTH-1:0] roll_rate_out,
 			  output reg [RATE_BIT_WIDTH-1:0] pitch_rate_out,
 			  input [RATE_BIT_WIDTH-1:0] yaw_rate_in,
 			  input [RATE_BIT_WIDTH-1:0] roll_rate_in,
 			  input [RATE_BIT_WIDTH-1:0] pitch_rate_in,
 			  input [VELOCITY_BIT_WIDTH-1:0] x_velocity,
 			  input [VELOCITY_BIT_WIDTH-1:0] y_velocity,
 			  input [VELOCITY_BIT_WIDTH-1:0] z_velocity,
 			  input [ROTATION_BIT_WIDTH-1:0] x_rotation,
 			  input [ROTATION_BIT_WIDTH-1:0] y_rotation,
 			  input [ROTATION_BIT_WIDTH-1:0] z_rotation,
 			  input sys_clk);

	always @(posedge sys_clk) begin
		if (yaw_rate_in || roll_rate_in || pitch_rate_in || x_velocity || y_velocity || z_velocity || x_rotation || y_rotation || z_rotation) begin
			yaw_rate_out <= ~yaw_rate_out;
			roll_rate_out <= ~roll_rate_out;
			pitch_rate_out <= ~pitch_rate_out;
		end
	end

endmodule
