`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * test_pwm_value.v - Test bench for pwm_to_value.v
 */

`include "common_defines.v"

module test_pwm_reader;

	localparam SUCCESS = 1'b0;
	localparam FAIL = 1'b1;
	localparam [10:0] INITIAL_PULSE_HIGH_US = 1500;

	wire sys_clk;
	wire us_clk;
	wire [10:0] pwm_pulse_length_us;
	reg pwm = 0;
	reg resetn = 1;
	reg result = SUCCESS;

	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (.us_clk(us_clk),
						   .sys_clk(sys_clk),
						   .resetn(1));  // TODO: Change this to the top level reset signal (.resetn(resetn))

	pwm_reader DUT(.pwm_pulse_length_us(pwm_pulse_length_us),
				   .pwm(pwm),
				   .us_clk(us_clk),
				   .resetn(resetn));

	initial begin
		// Test 1: Test initial reset
		resetn = 0;
		#5000;
		if (pwm_pulse_length_us != INITIAL_PULSE_HIGH_US) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		// Test 2: Make sure that if we don't see any signal (constant 0 pwm) we output the default value
		#10 resetn = 1;
		#5000;
		if (pwm_pulse_length_us != INITIAL_PULSE_HIGH_US) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		// Test 3: Staying high for only 900us (1ms is min) should output the minimum pwm_pulse_length_us value
		#1000 pwm = 1;
		#900000 pwm = 0;
		#5000; // wait ~5 clock cycles to check the result
		if (pwm_pulse_length_us != `MIN_PWM_TIME_HIGH_US) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		// Test 4: Staying high for over 2000us should output 2000us
		#200000 pwm = 1; // Stay at 0 for a while before raising to increase readability in the waveform
		#2100000 pwm = 0;
		#5000;
		if (pwm_pulse_length_us != 2000) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		// Test 5: Staying high for around 1500us should output a time with +/- 5us of 1500us
		#200000 pwm = 1; // Stay at 0 for a while before raising to increase readability in the waveform
		#1500000 pwm = 0;
		#5000;
		if (pwm_pulse_length_us <= 1450 || pwm_pulse_length_us >= 1550) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		// Test 6: Staying high for over 800us should output the initial pwm_pulse_length_us value
		#200000 pwm = 1; // Stay at 0 for a while before raising to increase readability in the waveform
		#800000 pwm = 0;
		#5000;
		if (pwm_pulse_length_us != 1000) begin
			result = FAIL;
			display_fail(resetn, pwm_pulse_length_us);
			#10 $finish;
		end

		if (result == SUCCESS)
			display_success();
		#10 $finish;
	end

	task display_fail (input result_in,
					   input resetn,
					   input [10:0] pwm_pulse_length_us);
		begin
			#1 $display("########################################################################");
			#1 $display("########################################################################");
			#1 $display("########## TEST FAILED: resetn=%d pwm_pulse_length_us=%d ##########", resetn, pwm_pulse_length_us);
			#1 $display("########################################################################");
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

