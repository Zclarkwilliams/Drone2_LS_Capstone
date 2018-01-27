module test_value_to_rate;
	
	localparam N_RATE = 36;
	localparam N_VAL = 14;

	// target rates (rad/s) in 2's complement fixed point
	wire [N_RATE-1:0] throttle_rate = 0;
	wire [N_RATE-1:0] yaw_rate = 0;
	wire [N_RATE-1:0] roll_rate = 0;
	wire [N_RATE-1:0] pitch_rate = 0;

	// values ranging from -9000 to 9000
	wire [N_VAL-1:0] throttle_val = 0;
	wire [N_VAL-1:0] yaw_val = 0;
	wire [N_VAL-1:0] roll_val = 0;
	wire [N_VAL-1:0] pitch_val = 0;

	wire sys_clk = 0;

	// line up the parameters here to the ones internal to the receiver module
	value_to_rate #(N_RATE, N_VAL) DUT (.throttle_rate(throttle_rate),
										.yaw_rate(yaw_rate),
										.roll_rate(roll_rate),
										.pitch_rate(pitch_rate),
										.throttle_val(throttle_val),
										.yaw_val(yaw_val),
										.roll_val(roll_val),
										.pitch_val(pitch_val),
										.sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule
