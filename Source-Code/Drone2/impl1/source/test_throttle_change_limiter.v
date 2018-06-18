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
	integer i;


	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	// line up the parameters here to the ones internal to the receiver module
	throttle_change_limiter DUT (
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
			//$display("%t: %m Throttle value in =%d",$time, DUT.latched_throttle);
			//$display("%t: %m Throttle value sum =%d",$time, DUT.summed_throttle);
			//$display("%t: %m Throttle value average =%d",$time, DUT.average_throttle);
			//$display("%t: %m Throttle value out =%d",$time, DUT.throttle_pwm_value_out);
		end
	endtask

	initial begin
		//$display("%t: %m Reset throttle rate limiter", $time);
		resetn = 1;
		#10 resetn = 0;
		#10 resetn = 1;

		//$display("%t: %m Set initial values",$time);
		run_test(.task_throttle_pwm_value_in(0));

		//$display("\n\n\n%t: %m Throttle to 250, MAX",$time);
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(250));
		end
		

		//$display("\n\n\n%t: %m Throttle to 10, just above idle",$time);
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(10));
		end
		
		//$display("\n%t: %m Throttle to 250, MAX",$time);
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(250));
		end
		//$display("%t: %m Throttle to 0, idle",$time);
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(0));
		end
		
		
		//$display("\n\n\n%t: %m Noisy throttle input +/- 10 of 125",$time);
		
		//$display("%t: %m Throttle to 125, MID",$time);
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(125));
		end
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(135));
			//$display("%t: %m",$time);
			run_test(.task_throttle_pwm_value_in(115));
		end
		
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(250*0.40));
		end
		
		for(i = 0; i < 64; i=i+1) begin
			//$display("%t: %m iteration %d",$time, i);
			run_test(.task_throttle_pwm_value_in(250*0.60));
		end

		$display("%t: %m Test complete", $time);
		$stop;
	end

endmodule

