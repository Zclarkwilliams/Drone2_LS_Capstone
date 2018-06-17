/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ps / 1ps
`default_nettype none
`include "common_defines.v"

module test_angle_controller;

	// PWM from receiver module
	reg [`REC_VAL_BIT_WIDTH-1:0] throttle_target = 0;
	reg [`REC_VAL_BIT_WIDTH-1:0] yaw_target      = 0;
	reg [`REC_VAL_BIT_WIDTH-1:0] roll_target     = 0;
	reg [`REC_VAL_BIT_WIDTH-1:0] pitch_target    = 0;

	// IMU euler angles
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] yaw_actual   = 0;
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] pitch_actual = 0;
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] roll_actual  = 0;

	// IMU linear acceleration vectors
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] x_linear_accel = 0;
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] y_linear_accel = 0;
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] z_linear_accel = 0;

	wire signed [`RATE_BIT_WIDTH-1:0] throttle_rate_out;
	wire signed [`RATE_BIT_WIDTH-1:0] yaw_rate_out;
	wire signed [`RATE_BIT_WIDTH-1:0] pitch_rate_out;
	wire signed [`RATE_BIT_WIDTH-1:0] roll_rate_out;

	wire signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error;
	wire signed [`RATE_BIT_WIDTH-1:0] pitch_angle_error;
	wire signed [`RATE_BIT_WIDTH-1:0] roll_angle_error;

	reg  resetn;
	wire sys_clk;
	wire us_clk;
	wire complete_signal;
	wire active_signal;
	reg  start_signal;
	int i,j;

	// angle value aliases
	// 5760/4 = 1440 = 90˚ and 4320 = 270˚
	localparam
		ANGLE_360_DEG = 5760, 
		ANGLE_270_DEG = 4320, 
		ANGLE_180_DEG = 2880, 
		ANGLE_90_DEG  = 1440,
		ANGLE_0_DEG   = 0;
	

	defparam OSCH_inst.NOM_FREQ = "38.00";
	OSCH OSCH_inst (.STDBY(1'b0),
       			    .OSC(sys_clk),
       			    .SEDSTDBY());

	us_clk us_clk_divider (
		.us_clk(us_clk),
		.sys_clk(sys_clk),
		.resetn(resetn));

	// line up the parameters here to the ones internal to the receiver module
	angle_controller DUT (
		.throttle_rate_out(throttle_rate_out),
		.yaw_rate_out(yaw_rate_out),
		.pitch_rate_out(pitch_rate_out),
		.roll_rate_out(roll_rate_out),
		.yaw_angle_error(yaw_angle_error),
		.pitch_angle_error(pitch_angle_error),
		.roll_angle_error(roll_angle_error),
		.complete_signal(complete_signal),
		.active_signal(active_signal),
		.throttle_target(throttle_target),
		.yaw_target(yaw_target),
		.pitch_target(pitch_target),
		.roll_target(roll_target),
		.yaw_actual(yaw_actual),
		.pitch_actual(pitch_actual),
		.roll_actual(roll_actual),
		.resetn(resetn),
		.start_signal(start_signal),
		.us_clk(us_clk));

			task run_test;
				input reg [15:0] task_throttle_target;
				input reg [15:0] task_yaw_target    ;
				input reg [15:0] task_roll_target   ;
				input reg [15:0] task_pitch_target  ;
				input reg [15:0] task_yaw_actual    ;
				input reg [15:0] task_pitch_actual  ;
				input reg [15:0] task_roll_actual   ;
				begin
					throttle_target = task_throttle_target;
					yaw_target      = task_yaw_target    ;
					roll_target     = task_roll_target   ;
					pitch_target    = task_pitch_target  ;
					yaw_actual      = task_yaw_actual    ;
					pitch_actual    = task_pitch_actual  ;
					roll_actual     = task_roll_actual   ;
					start_signal    = 1;
					repeat (2) @(posedge us_clk); start_signal    = 0; repeat (8) @(posedge us_clk);
					//$display("%t: %m Throttle rate out =%d",$time, throttle_rate_out);
					$display("%t: %m Yaw PWM Value Latch =\t%d",$time, DUT.latched_yaw);
					$display("%t: %m Yaw rate out =\t%d",$time, yaw_rate_out);
					$display("%t: %m Yaw angle error =\t%d",$time, yaw_angle_error);
					$display("%t: %m Yaw tracking angle =\t%d",$time, DUT.yaw_target_angle_tracking);
					$display("%t: %m Yaw mapped value =\t%d",$time, DUT.mapped_yaw);
					$display("%t: %m Yaw actual value =\t%d",$time, DUT.yaw_actual);
					$display("%t: %m Yaw rate out       (BIN) =\t%b",$time, yaw_rate_out);
					$display("%t: %m Yaw angle error    (BIN) =\t%b",$time, yaw_angle_error);
					$display("%t: %m Yaw tracking angle (BIN) =\t%b",$time, DUT.yaw_target_angle_tracking);
					$display("%t: %m Yaw mapped value   (BIN) =\t%b",$time, DUT.mapped_yaw);
					//$display("%t: %m Pitch rate out =%d",$time, pitch_rate_out);
					//$display("%t: %m Pitch angle error =%d",$time, pitch_angle_error);
					//$display("%t: %m Roll rate out =%d",$time, roll_rate_out);
					//$display("%t: %m Roll angle error =%d",$time, roll_angle_error);
				end
			endtask

	initial begin
		$display("%t: %m Reset angle controller", $time);
		resetn = 1;
		#10 resetn = 0;
		#10 resetn = 1;

		$display("%t: %m Set initial values",$time);
		run_test(
		.task_throttle_target(0),
		.task_yaw_target(0),
		.task_roll_target(0),
		.task_pitch_target(0),
		.task_yaw_actual(250),
		.task_pitch_actual(0),
		.task_roll_actual(0)
		);

		
		$display("\n%t: %m Test neutral stick positions, throttle to 50 percent with angles of value 0 from IMU",$time);
		run_test(
		.task_throttle_target(125),
		.task_yaw_target(250),
		.task_roll_target(0),
		.task_pitch_target(0),
		.task_yaw_actual(ANGLE_360_DEG*4),
		.task_pitch_actual(0),
		.task_roll_actual(0)
		);

		j = 0;
		for(i = 0; i < 250; i=i+1) begin
			$display("\n%t: %m Test yaw at max PWM input (250), throttle to 50 percent, 0 linear acceleration, with angles of value 0 from IMU",$time);
			if(((125*j)+125) > (ANGLE_360_DEG*4))
				j = 0;
			$display("%t: %m i=%d, j=%d, yaw_actual input =%d",$time, i, j, ((125*j)+125));
			run_test(
			.task_throttle_target(125),
			.task_yaw_target(250),
			.task_roll_target(0),
			.task_pitch_target(0),
			.task_yaw_actual(((125*j)+125)),
			.task_pitch_actual(0),
			.task_roll_actual(0)
			);
			j = j+1;
		end

		$display("%t: %m Test complete", $time);
		$stop;
	end

endmodule

