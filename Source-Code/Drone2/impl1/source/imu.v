/**
 * module imu - receive input from the hardware imu over i2C and output 
 				the values that were read.
 *
 * Parameters
 * @N_VELOCITY: Number of bits for velocity 2's complement fixed point outputs
 * @N_ROTATION: Number of bits for rotation 2's complement fixed point outputs
 * @N_ACCEL: Number of bits for acceleration 2's complement fixed point outputs
 *
 * Outputs
 * @x_velocity: x-axis velocity vector in 2's complement fixed point
 * @y_velocity: y-axis velocity vector in 2's complement fixed point
 * @z_velocity: z-axis velocity vector in 2's complement fixed point
 * @x_rotation: x-axis rotation vector in 2's complement fixed point
 * @y_rotation: y-axis rotation vector in 2's complement fixed point
 * @z_rotation: z-axis rotation vector in 2's complement fixed point
 * @x_accel: x-axis acceleration vector in 2's complement fixed point
 * @y_accel: y-axis acceleration vector in 2's complement fixed point
 * @z_accel: z-axis acceleration vector in 2's complement fixed point
 * 
 * Inputs
 * @sda: i2c serial data
 * @scl: i2c serial clock
 * @sys_clk: system clock
 */
module imu #(parameter N_VELOCITY = 36,
 			 parameter N_ROTATION = 36,
			 parameter N_ACCEL = 36)
			(output reg [N_VELOCITY-1:0] x_velocity,
			 output reg [N_VELOCITY-1:0] y_velocity,
			 output reg [N_VELOCITY-1:0] z_velocity,
			 output reg [N_ROTATION-1:0] x_rotation,
			 output reg [N_ROTATION-1:0] y_rotation,
			 output reg [N_ROTATION-1:0] z_rotation,
			 output reg [N_ACCEL-1:0] x_accel,
			 output reg [N_ACCEL-1:0] y_accel,
			 output reg [N_ACCEL-1:0] z_accel,
			 input sda,
			 input scl,
			 input sys_clk);

 	always @(posedge sys_clk) begin
 		if (sda || scl) begin
 			x_velocity <= ~x_velocity;
 			y_velocity <= ~y_velocity;
 			z_velocity <= ~z_velocity;
 			x_rotation <= ~x_rotation;
 			y_rotation <= ~y_rotation;
 			z_rotation <= ~z_rotation;
 			x_accel <= ~x_accel;
 			y_accel <= ~y_accel;
 			z_accel <= ~z_accel;
 		end
 	end

endmodule
