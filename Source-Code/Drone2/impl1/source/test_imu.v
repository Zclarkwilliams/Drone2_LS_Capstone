module test_imu;

	localparam VELOCITY_BIT_WIDTH = 36;
	localparam ROTATION_BIT_WIDTH = 36;
	localparam ACCEL_BIT_WIDTH = 36;

	// output vectors
	wire [VELOCITY_BIT_WIDTH-1:0] x_velocity = 0;
	wire [VELOCITY_BIT_WIDTH-1:0] y_velocity = 0;
	wire [VELOCITY_BIT_WIDTH-1:0] z_velocity = 0;
	wire [ROTATION_BIT_WIDTH-1:0] x_rotation = 0;
	wire [ROTATION_BIT_WIDTH-1:0] y_rotation = 0;
	wire [ROTATION_BIT_WIDTH-1:0] z_rotation = 0;
	wire [ACCEL_BIT_WIDTH-1:0] x_accel = 0;
	wire [ACCEL_BIT_WIDTH-1:0] y_accel = 0;
	wire [ACCEL_BIT_WIDTH-1:0] z_accel = 0;

	// inputs
	wire sda = 0;
	wire scl = 0;
	wire sys_clk = 0;

	imu #(VELOCITY_BIT_WIDTH, ROTATION_BIT_WIDTH, ACCEL_BIT_WIDTH) DUT (.x_velocity(x_velocity),
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
