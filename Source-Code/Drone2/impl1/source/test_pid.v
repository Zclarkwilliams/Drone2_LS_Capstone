module test_pid;
	
	localparam N_RATE = 36;
	localparam N_VELOCITY = 36;
	localparam N_ROTATION = 36;

	// rates represented in 2's complement fixed point
	wire [N_RATE-1:0] yaw_rate_out;
	wire [N_RATE-1:0] roll_rate_out;
	wire [N_RATE-1:0] pitch_rate_out;

	// rates represented in 2's complement fixed point
	wire [N_RATE-1:0] yaw_rate_in;
	wire [N_RATE-1:0] roll_rate_in;
	wire [N_RATE-1:0] pitch_rate_in;

	// velocities represented in 2's complement fixed point
	wire [N_VELOCITY-1:0] x_velocity;
	wire [N_VELOCITY-1:0] y_velocity;
	wire [N_VELOCITY-1:0] z_velocity;

	// rotations represented in 2's complement fixed point
	wire [N_ROTATION-1:0] x_rotation;
	wire [N_ROTATION-1:0] y_rotation;
	wire [N_ROTATION-1:0] z_rotation;

	wire clk;

	// line up the parameters here to the ones internal to the receiver module
	pid #(N_RATE, N_VELOCITY, N_ROTATION) DUT (.yaw_rate_out(yaw_rate_out),
											   .roll_rate_out(roll_rate_out),
											   .pitch_rate_out(pitch_rate_out),
											   .yaw_rate_in(yaw_rate_in),
											   .roll_rate_in(roll_rate_in),
											   .pitch_rate_in(pitch_rate_in),
											   .x_velocity(x_velocity),
											   .y_velocity(y_velocity),
											   .z_velocity(z_velocity),
											   .x_rotation(x_rotation),
											   .y_rotation(y_rotation),
											   .z_rotation(z_rotation),
											   .sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule
