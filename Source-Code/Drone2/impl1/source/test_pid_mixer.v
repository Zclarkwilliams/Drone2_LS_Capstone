module test_pid_mixer;

	localparam RATE_BIT_WIDTH = 36;
	localparam MOTOR_RATE_BIT_WIDTH = 36;

	// motor rates represented as 2's complement fixed point
	wire [MOTOR_RATE_BIT_WIDTH-1:0] motor_1_rate;
	wire [MOTOR_RATE_BIT_WIDTH-1:0] motor_2_rate;
	wire [MOTOR_RATE_BIT_WIDTH-1:0] motor_3_rate;
	wire [MOTOR_RATE_BIT_WIDTH-1:0] motor_4_rate;

	// rates represented as 2's complement fixed point
	wire [RATE_BIT_WIDTH-1:0] throttle_rate;
	wire [RATE_BIT_WIDTH-1:0] yaw_rate;
	wire [RATE_BIT_WIDTH-1:0] roll_rate;
	wire [RATE_BIT_WIDTH-1:0] pitch_rate;

	wire sys_clk;

	pid_mixer #(RATE_BIT_WIDTH, MOTOR_RATE_BIT_WIDTH) DUT (.motor_1_rate(motor_1_rate),
														   .motor_2_rate(motor_2_rate),
														   .motor_3_rate(motor_3_rate),
														   .motor_4_rate(motor_4_rate),
														   .throttle_rate(throttle_rate),
														   .yaw_rate(yaw_rate),
														   .roll_rate(roll_rate),
														   .pitch_rate(pitch_rate),
														   .sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule
