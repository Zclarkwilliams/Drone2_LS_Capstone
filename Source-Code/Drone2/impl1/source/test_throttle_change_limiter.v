/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ps / 1ps
`default_nettype none
`include "common_defines.v"

module test_throttle_change_limiter;

	// PWM from receiver module
	reg signed [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_in = 0;

	wire signed [`RATE_BIT_WIDTH-1:0] throttle_pwm_value_out;

	reg  resetn;
	wire sys_clk;
	wire us_clk;
	wire complete_signal;
	wire active_signal;
	reg  start_signal;


	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	// line up the parameters here to the ones internal to the receiver module
	throttle_rate_limiter DUT (
		.throttle_pwm_value_in(throttle_pwm_value_in),
		.throttle_pwm_value_out(throttle_pwm_value_out),
		.complete_signal(complete_signal),
		.active_signal(active_signal),
		.resetn(resetn),
		.start_signal(start_signal),
		.us_clk(us_clk));

			task run_test;
				input reg [15:0] task_throttle_pwm_value_in;
				begin
					throttle_pwm_value_in = task_throttle_pwm_value_in;
					start_signal    = 1;
					repeat (2) @(posedge us_clk); start_signal    = 0; repeat (8) @(posedge us_clk);
					$display("%t: %m Throttle rate in =%d",$time, task_throttle_pwm_value_in);
					$display("%t: %m Throttle rate out =%d",$time, task_throttle_pwm_value_out);
				end
			endtask

	initial begin
		$display("%t: %m Reset throttle rate limiter", $time);
		resetn = 1;
		#10 resetn = 0;
		#10 resetn = 1;

		$display("%t: %m Set initial values",$time);
		run_test(.task_throttle_pwm_value_in(0));

		$display("\n%t: %m Throttle to max",$time);
		run_test(
		.task_throttle_target(250),
		.task_yaw_target(0),
		.task_roll_target(0),
		.task_pitch_target(0),
		.task_yaw_actual(0),
		.task_pitch_actual(0),
		.task_roll_actual(0)
		);


		$display("%t: %m Test complete", $time);
		$stop;
	end

endmodule

