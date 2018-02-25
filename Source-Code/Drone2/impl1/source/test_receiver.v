`timescale 1ns / 1ns

`include "common_defines.v"

module test_receiver;

	localparam SUCCESS = 1'b1;
	localparam FAIL = 1'b0;

	reg result = SUCCESS;
	// scaled output values corresponding to their pwm input counterpart
	wire [`PWM_VALUE_BIT_WIDTH - 1:0] throttle_val;
	wire [`PWM_VALUE_BIT_WIDTH - 1:0] yaw_val;
	wire [`PWM_VALUE_BIT_WIDTH - 1:0] roll_val;
	wire [`PWM_VALUE_BIT_WIDTH - 1:0] pitch_val;
	reg [`PWM_VALUE_BIT_WIDTH - 1:0] expected_val;
	// pwm inputs modeling the hardware receiver
	reg throttle_pwm = 0;
	reg yaw_pwm = 0;
	reg roll_pwm = 0;
	reg pitch_pwm = 0;
	reg resetn = 1;

	wire sys_clk = 0;
	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (
		.STDBY(1'b0),
       	.OSC(sys_clk),
       	.SEDSTDBY());

	wire us_clk;
	us_clk us_clk_divider (.us_clk(us_clk),
						   .sys_clk(sys_clk),
						   .resetn(1));  // TODO: Change this to the top level reset signal (.resetn(resetn))

	// line up the parameters here to the ones internal to the receiver module
	receiver DUT (
		.throttle_val(throttle_val),
		.yaw_val(yaw_val),
		.roll_val(roll_val),
		.pitch_val(pitch_val),
		.throttle_pwm(throttle_pwm),
		.yaw_pwm(yaw_pwm),
		.roll_pwm(roll_pwm),
		.pitch_pwm(pitch_pwm),
		.us_clk(us_clk),
		.resetn(resetn));

	initial begin
		// Test 1: reset all submodules and make sure they output their default values
		resetn = 0;
		#5000 resetn = 1;
		#5000;

		expected_val = map_pwm_to_value(`THROTTLE_DEFAULT_PULSE_TIME_HIGH_US);
		if (throttle_val != expected_val) begin
			display_fail("throttle", throttle_val, expected_val);
			result = FAIL;
		end

		expected_val = map_pwm_to_value(`YAW_DEFAULT_PULSE_TIME_HIGH_US);
		if (yaw_val != expected_val) begin
			display_fail("yaw", yaw_val, expected_val);
			result = FAIL;
		end

		expected_val = map_pwm_to_value(`ROLL_DEFAULT_PULSE_TIME_HIGH_US);
		if (roll_val != expected_val) begin
			display_fail("roll", roll_val, expected_val);
			result = FAIL;
		end

		expected_val = map_pwm_to_value(`PITCH_DEFAULT_PULSE_TIME_HIGH_US);
		if (pitch_val != expected_val) begin
			display_fail("pitch", pitch_val, expected_val);
			result = FAIL;
		end

		if (result == FAIL)
				#10 $finish;

		display_success();
		#10 $finish;
	end

	function [`PWM_VALUE_BIT_WIDTH - 1:0] map_pwm_to_value(input [`PWM_TIME_BIT_WIDTH - 1:0] pwm_time_high_us);
		begin
			pwm_time_high_us = pwm_time_high_us - `MIN_PWM_TIME_HIGH_US;
			map_pwm_to_value = pwm_time_high_us[9:2];
		end
	endfunction

	task display_fail (input [63:0] val_name,
					   input [`PWM_VALUE_BIT_WIDTH - 1:0] actual_val,
  					   input [`PWM_VALUE_BIT_WIDTH - 1:0] expected_val);
		begin
			#1 $display("########################################################################");
			#1 $display("\tTEST FAILED(%s):", val_name);
			#1 $display("\t\tactual=%d expected=%d", actual_val, expected_val);
			#1 $display("########################################################################");
		end
	endtask

	task display_success ();
		begin
			#1 $display("########################################################################");
			#1 $display("########################################################################");
			#1 $display("########################### ALL TESTS PASSED ###########################");
			#1 $display("########################################################################");
			#1 $display("########################################################################");
		end
	endtask

endmodule
