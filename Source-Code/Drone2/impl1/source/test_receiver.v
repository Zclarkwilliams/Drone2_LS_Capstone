module test_receiver;

	localparam N_VAL = 14;  // Number of bits for throttle/yaw/roll/pitch val outputs
	localparam N_AUX = 4;   // Number of bits for aux1/aux2 val outputs
	localparam N_MODE = 3;  // Number of bits for the mode output

	// scaled output values corresponding to their pwm input counterpart
	wire [N_AUX-1:0] aux1_val = 0;
	wire [N_AUX-1:0] aux2_val = 0;
	wire [N_MODE-1:0] mode_val = 0;
	wire [N_VAL-1:0] throttle_val = 0;
	wire [N_VAL-1:0] yaw_val = 0;
	wire [N_VAL-1:0] roll_val = 0;
	wire [N_VAL-1:0] pitch_val = 0;

	// pwm inputs modeling the hardware receiver
	wire aux1_pwm = 0;
	wire aux2_pwm = 0;
	wire mode_pwm = 0;
	wire throttle_pwm = 0;
	wire yaw_pwm = 0;
	wire roll_pwm = 0;
	wire pitch_pwm = 0;

	wire sys_clk = 0;

	// line up the parameters here to the ones internal to the receiver module
	receiver #(N_VAL, N_AUX, N_MODE) DUT (.aux1_val(aux1_val),
										  .aux2_val(aux2_val),
										  .mode_val(mode_val),
										  .throttle_val(throttle_val),
										  .yaw_val(yaw_val),
										  .roll_val(roll_val),
										  .pitch_val(pitch_val),
										  .aux1_pwm(aux1_pwm),
										  .aux2_pwm(aux2_pwm),
										  .mode_pwm(mode_pwm),
										  .throttle_pwm(throttle_pwm),
										  .yaw_pwm(yaw_pwm),
										  .roll_pwm(roll_pwm),
										  .pitch_pwm(pitch_pwm),
										  .sys_clk(sys_clk));

	initial begin
		$display("%m successful");
	end

endmodule
