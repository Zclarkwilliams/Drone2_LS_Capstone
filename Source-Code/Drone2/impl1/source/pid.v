/**
 * module pid - top level pid module that takes in target rates (rad/s) represented
 *				and velocity/rotation vectors represented in 2's compelement fixed
 *				point.
 *
 * Parameters
 * @N_VAL: Number of bits for throttle/yaw/roll/pitch val outputs
 * @N_AUX: Number of bits for aux1/aux2 val outputs
 * @N_MODE: Number of bits for the mode output
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
 module pid #(parameter N_RATE = 36,
			  parameter N_VELOCITY = 36,
			  parameter N_ROTATION = 36)
			 (output reg [N_RATE-1:0] yaw_rate_out,
			  output reg [N_RATE-1:0] roll_rate_out,
			  output reg [N_RATE-1:0] pitch_rate_out,
			  input [N_RATE-1:0] yaw_rate_in,
			  input [N_RATE-1:0] roll_rate_in,
			  input [N_RATE-1:0] pitch_rate_in,
			  input [N_VELOCITY-1:0] x_velocity,
	 		  input [N_VELOCITY-1:0] y_velocity,
	 		  input [N_VELOCITY-1:0] z_velocity,
	 		  input [N_ROTATION-1:0] x_rotation,
	 		  input [N_ROTATION-1:0] y_rotation,
	 		  input [N_ROTATION-1:0] z_rotation,
			  input sys_clk);

	always @(posedge sys_clk) begin
		if (yaw_rate_in || roll_rate_in || pitch_rate_in || x_velocity || y_velocity || z_velocity || x_rotation || y_rotation || z_rotation) begin
			yaw_rate_out <= ~yaw_rate_out;
			roll_rate_out <= ~roll_rate_out;
			pitch_rate_out <= ~pitch_rate_out;
		end
	end

endmodule
