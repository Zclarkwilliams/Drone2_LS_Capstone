module test_imu;

	localparam N_VELOCITY = 36;
	localparam N_ROTATION = 36;
	localparam N_ACCEL = 36;

	// output vectors
	wire [N_VELOCITY-1:0] x_velocity = 0;
	wire [N_VELOCITY-1:0] y_velocity = 0;
	wire [N_VELOCITY-1:0] z_velocity = 0;
	wire [N_ROTATION-1:0] x_rotation = 0;
	wire [N_ROTATION-1:0] y_rotation = 0;
	wire [N_ROTATION-1:0] z_rotation = 0;
	wire [N_ACCEL-1:0] x_accel = 0;
	wire [N_ACCEL-1:0] y_accel = 0;
	wire [N_ACCEL-1:0] z_accel = 0;

	// inputs
	wire sda = 0;
	wire scl = 0;
	wire sys_clk = 0;

	imu #(N_VELOCITY, N_ROTATION, N_ACCEL) DUT (.x_velocity(x_velocity),
												.y_velocity(y_velocity),
												.z_velocity(z_velocity),
												.x_rotation(x_rotation),
												.y_rotation(y_rotation),
												.z_rotation(z_rotation),
												.x_accel(x_accel),
												.y_accel(y_accel),
												.z_accel(z_accel),
												.sda(sda),
												.scl(scl),
												.sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule
