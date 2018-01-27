module test_pid_mixer;

	localparam N_RATE = 36;
	localparam N_MOTOR_RATE = 36;
	
	// motor rates represented as 2's complement fixed point
	wire [N_MOTOR_RATE-1:0] motor_1_rate;
	wire [N_MOTOR_RATE-1:0] motor_2_rate;
	wire [N_MOTOR_RATE-1:0] motor_3_rate;
	wire [N_MOTOR_RATE-1:0] motor_4_rate;

	// rates represented as 2's complement fixed point
	wire [N_RATE-1:0] throttle_rate;
	wire [N_RATE-1:0] yaw_rate;
	wire [N_RATE-1:0] roll_rate;
	wire [N_RATE-1:0] pitch_rate;

	wire sys_clk;

	pid_mixer #(N_RATE, N_MOTOR_RATE) DUT (.motor_1_rate(motor_1_rate),
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