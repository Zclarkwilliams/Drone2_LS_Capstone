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
	reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_input = 0;
	reg [`REC_VAL_BIT_WIDTH-1:0] yaw_pwm_value_input      = 0;

	// IMU Euler angles
	reg signed [`IMU_VAL_BIT_WIDTH-1:0] yaw_angle_imu   = 0;

	wire signed [`RATE_BIT_WIDTH-1:0] body_yaw_angle;

	wire signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error;

	reg  resetn;
	wire sys_clk;
	wire us_clk;
	wire complete_signal;
	wire active_signal;
	reg  start_signal;
	integer i;
	integer j;

	// angle value aliases
	// 5760/4 = 1440 = 90˚ and 4320 = 270˚
	localparam signed
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
	yaw_angle_accumulator DUT (
		.body_yaw_angle(body_yaw_angle),
		.yaw_angle_error(yaw_angle_error),
		.complete_signal(complete_signal),
		.active_signal(active_signal),
		.throttle_pwm_value_input(throttle_pwm_value_input),
		.yaw_pwm_value_input(yaw_pwm_value_input),
		.yaw_angle_imu(yaw_angle_imu),
		.resetn(resetn),
		.start_signal(start_signal),
		.us_clk(us_clk));

			task run_test;
				input reg [15:0] task_throttle_pwm_value_input;
				input reg [15:0] task_yaw_pwm_value_input     ;
				input reg [15:0] task_yaw_angle_imu           ;
				begin
					throttle_pwm_value_input = task_throttle_pwm_value_input;
					yaw_pwm_value_input      = task_yaw_pwm_value_input     ;
					yaw_angle_imu            = task_yaw_angle_imu           ;
					start_signal    = 1;
					repeat (2) @(posedge us_clk); start_signal    = 0; repeat (8) @(posedge us_clk);
					$display("%t: %m Yaw PWM Value Latch\t\t = %d",$time, DUT.latched_yaw_pwm_value_input);
					$display("%t: %m Yaw IMU value\t\t\t = %d",$time, DUT.latched_yaw_angle_imu);
					$display("%t: %m Yaw angle\t\t\t = %d",$time, body_yaw_angle);
					$display("%t: %m Yaw angle error\t\t = %d",$time, yaw_angle_error);
					//$display("%t: %m Yaw tracking angle\t = %d",$time, (DUT.body_yaw_angle_tracking));
					//$display("%t: %m Yaw body angle       (BIN) = %b",$time, body_yaw_angle);
					//$display("%t: %m Yaw body angle error (BIN) = %b",$time, yaw_angle_error);
					//$display("%t: %m Yaw tracking angle   (BIN) = %b",$time, (DUT.body_yaw_angle_tracking));
				end
			endtask

	initial begin
		$display("%t: %m Reset angle controller", $time);
		resetn = 1;
		#10 resetn = 0;
		#10 resetn = 1;

		$display("%t: %m Set initial values",$time);
		run_test(
		.task_throttle_pwm_value_input(0),
		.task_yaw_pwm_value_input(0),
		.task_yaw_angle_imu(249)
		);

		
		$display("\n%t: %m Test neutral stick positions, throttle to 50 percent with angles of value 0 from IMU",$time);
		run_test(
		.task_throttle_pwm_value_input(125),
		.task_yaw_pwm_value_input(249),
		.task_yaw_angle_imu(249)
		);

		j = 0;
		for(i = 0; i < 250; i=i+1) begin
			if(((31*j)+124) > ANGLE_360_DEG)
				j = 0;
			$display("\n%t: %m Test yaw at max PWM input, throttle to 50 percent",$time);
			$display("%t: %m i=%d, j=%d, yaw_angle_imu input =%d",$time, i, j, ((31*j)+124));
			run_test(
			.task_throttle_pwm_value_input(125),
			.task_yaw_pwm_value_input(249),
			.task_yaw_angle_imu(((31*j)+124))
			);
			j = j+1;
		end
		$display("%t: %m Set initial values",$time);
		run_test(
		.task_throttle_pwm_value_input(0),
		.task_yaw_pwm_value_input(0),
		.task_yaw_angle_imu(5760)
		);

		
		$display("\n%t: %m Test neutral stick positions, throttle to 50 percent with angles of value 0 from IMU",$time);
		run_test(
		.task_throttle_pwm_value_input(125),
		.task_yaw_pwm_value_input(1),
		.task_yaw_angle_imu(5760)
		);

		$display("\n\n%t: %m Now reversing directions\n\n",$time);
		j = 0;
		for(i = 0; i < 250; i=i+1) begin
			if((ANGLE_360_DEG-(31*j)) < ANGLE_0_DEG)
				j = 0;
			$display("\n%t: %m Test yaw at max PWM input, throttle to 50 percent",$time);
			$display("%t: %m i=%d, j=%d, yaw_angle_imu input =%d",$time, i, j, (ANGLE_360_DEG-(31*j)));
			run_test(
			.task_throttle_pwm_value_input(125),
			.task_yaw_pwm_value_input(1),
			.task_yaw_angle_imu((ANGLE_360_DEG-(31*j)))
			);
			j = j+1;
		end
		$display("%t: %m Test complete", $time);
		$stop;
	end

endmodule

