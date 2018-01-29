module test_pwm_generator;

	localparam RATE_BIT_WIDTH = 36;

	// motor pwm signals
	wire motor_1_pwm = 0;
	wire motor_2_pwm = 0;
	wire motor_3_pwm = 0;
	wire motor_4_pwm = 0;

	// motor rates
	wire [RATE_BIT_WIDTH-1:0] motor_1_rate = 0;
	wire [RATE_BIT_WIDTH-1:0] motor_2_rate = 0;
	wire [RATE_BIT_WIDTH-1:0] motor_3_rate = 0;
	wire [RATE_BIT_WIDTH-1:0] motor_4_rate = 0;

	wire sys_clk;

	pwm_generator #(RATE_BIT_WIDTH) DUT (.motor_1_pwm(motor_1_pwm),
										 .motor_2_pwm(motor_2_pwm),
										 .motor_3_pwm(motor_3_pwm),
										 .motor_4_pwm(motor_4_pwm),
										 .motor_1_rate(motor_1_rate),
										 .motor_2_rate(motor_2_rate),
										 .motor_3_rate(motor_3_rate),
										 .motor_4_rate(motor_4_rate),
										 .sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule