/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

`timescale 1ps / 1ps
`default_nettype none
`include "common_defines.v"
`include "pid_parameters.v"

`define INPUT_SEL_BIT_WIDTH	4
`define YAW_INPUT		4'd0
`define ROLL_INPUT		4'd1
`define PITCH_INPUT		4'd2
`define THROTTLE_INPUT		4'd3
`define DONT_CARE		0

module test_angle_controller;

	// PWM from receiver module
	reg signed [`REC_VAL_BIT_WIDTH-1:0] throttle_target = 0;
	reg signed [`REC_VAL_BIT_WIDTH-1:0] yaw_target      = 0;
	reg signed [`REC_VAL_BIT_WIDTH-1:0] roll_target     = 0;
	reg signed [`REC_VAL_BIT_WIDTH-1:0] pitch_target    = 0;

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

	wire signed [`RATE_BIT_WIDTH-1:0] pitch_angle_error;
	wire signed [`RATE_BIT_WIDTH-1:0] roll_angle_error;

	reg  resetn;
	wire sys_clk;
	wire us_clk;
	wire complete_signal;
	wire active_signal;
	reg  start_signal;
	
	reg signed [`RATE_BIT_WIDTH-1:0] expected_rate;

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

	function CHECK_EQUALS_RBW;
		input reg signed [`RATE_BIT_WIDTH-1:0] expected;
		input reg signed [`RATE_BIT_WIDTH-1:0] actual;

		begin
			CHECK_EQUALS_RBW = (expected === actual);
		end
	endfunction

	function automatic signed [`RATE_BIT_WIDTH-1:0] check_for_overflow;
		input reg signed [31:0] val;

		localparam signed [31:0]
			OVERFLOW_PROTECTION_MIN = 32'shFFFF8000,
			OVERFLOW_PROTECTION_MAX = 32'sh00007FFF;

		begin
			if (val <= OVERFLOW_PROTECTION_MIN)
				check_for_overflow = OVERFLOW_PROTECTION_MIN[`RATE_BIT_WIDTH-1:0];
			else if (val >= OVERFLOW_PROTECTION_MAX)
				check_for_overflow = OVERFLOW_PROTECTION_MAX[`RATE_BIT_WIDTH-1:0];
			else
				check_for_overflow = val[15:0];
		end
	endfunction

	function [`RATE_BIT_WIDTH-1:0] calculate_actual_rate;
		input reg [`INPUT_SEL_BIT_WIDTH-1:0] input_sel;
		input reg signed [`REC_VAL_BIT_WIDTH-1:0] rec_input_val;
		input reg signed [`RATE_BIT_WIDTH-1:0] imu_input_val;

		reg signed [`RATE_BIT_WIDTH-1:0] actual;
		reg signed [31:0] actual32;

		localparam signed [`OPS_BIT_WIDTH-1:0]
			THROTTLE_MAX 	=  250 << `FIXED_POINT_SHIFT,
			YAW_MAX 	=  100 << `FIXED_POINT_SHIFT,
			YAW_MIN 	= -100 << `FIXED_POINT_SHIFT,
			PITCH_MAX 	=  100 << `FIXED_POINT_SHIFT,
			PITCH_MIN 	= -100 << `FIXED_POINT_SHIFT,
			ROLL_MAX	=  100 << `FIXED_POINT_SHIFT,
			ROLL_MIN	= -100 << `FIXED_POINT_SHIFT;

		localparam
			SHIFT_BACK	= 7'd16;

		localparam signed
			ZERO_PAD	= 16'd0,
			FRONT_PAD	= 6'b0,
			END_PAD		= 2'b0,
			THROTTLE_F_PAD	= 4'b0,
			THROTTLE_R_PAD	= 4'b0;

		localparam signed [`OPS_BIT_WIDTH-1:0]
			MAPPING_SUBS	= 16'd500;

		localparam signed [31:0]
			OVERFLOW_PROTECTION_MIN = 32'shFFFF8000,
			OVERFLOW_PROTECTION_MAX = 32'sh00007FFF;
		begin	
			case (input_sel)
				`YAW_INPUT: begin
					actual = $signed({FRONT_PAD, rec_input_val, END_PAD}) - MAPPING_SUBS;
					actual32 = $signed({actual, ZERO_PAD}) >>> SHIFT_BACK;
					actual = check_for_overflow((actual32 * `YAW_SCALE_MULT) / (1 << `YAW_SCALE_SHIFT));	
					$display("actual = %d\n", $signed(actual[15:4]));
					if(actual > YAW_MAX)
						calculate_actual_rate = YAW_MAX;
					else if(actual < YAW_MIN)
						calculate_actual_rate = YAW_MIN;
					else
						calculate_actual_rate = actual;
				end
				`ROLL_INPUT: begin
					actual = $signed({FRONT_PAD, rec_input_val, END_PAD}) - MAPPING_SUBS + imu_input_val;
					actual32 = $signed({actual, ZERO_PAD}) >>> SHIFT_BACK;
					actual = check_for_overflow((actual32 * `ROLL_SCALE_MULT) / (1 << `ROLL_SCALE_SHIFT));
					if(actual > ROLL_MAX)
						calculate_actual_rate = ROLL_MAX;
					else if(actual < ROLL_MIN)
						calculate_actual_rate = ROLL_MIN;
					else
						calculate_actual_rate = actual;
				end
				`PITCH_INPUT: begin
					actual = $signed({FRONT_PAD, rec_input_val, END_PAD}) - MAPPING_SUBS - imu_input_val;
					actual32 = $signed({actual, ZERO_PAD}) >>> SHIFT_BACK;
					actual = check_for_overflow((actual32 * `PITCH_SCALE_MULT) / (1 << `PITCH_SCALE_SHIFT));
					if(actual > PITCH_MAX)
						calculate_actual_rate = PITCH_MAX;
					else if(actual < PITCH_MIN)
						calculate_actual_rate = PITCH_MIN;
					else
						calculate_actual_rate = actual;
				end
				`THROTTLE_INPUT: begin
					actual = $signed({THROTTLE_F_PAD, rec_input_val, THROTTLE_R_PAD});
					actual32 = $signed({actual, ZERO_PAD}) >>> SHIFT_BACK;
					actual = check_for_overflow((actual32 * `THROTTLE_SCALE_MULT) >>> `THROTTLE_SCALE_SHIFT);
					if(actual > THROTTLE_MAX)
						calculate_actual_rate = THROTTLE_MAX;
					else if(actual < `MOTOR_VAL_MIN)
						calculate_actual_rate = `MOTOR_VAL_MIN;
					else
						calculate_actual_rate = actual;
				end
				default:
					$display("Invalid input_sel %d\n", input_sel);
			endcase	
		end
	endfunction

	task run_test;
		input reg signed [15:0] task_throttle_target;
		input reg signed [15:0] task_yaw_target;
		input reg signed [15:0] task_roll_target;
		input reg signed [15:0] task_pitch_target;
		input reg signed [15:0] task_yaw_actual;
		input reg signed [15:0] task_pitch_actual;
		input reg signed [15:0] task_roll_actual;
		begin
			throttle_target = task_throttle_target;
			yaw_target      = task_yaw_target;
			roll_target     = task_roll_target;
			pitch_target    = task_pitch_target;
			yaw_actual      = task_yaw_actual;
			pitch_actual    = task_pitch_actual;
			roll_actual     = task_roll_actual;
			start_signal    = 1;
			repeat (2) @(posedge us_clk);
			start_signal = 0;
			repeat (8) @(posedge us_clk);
			// TODO: Deal with error expect/actual rates
			$display("%t: %m Pitch angle error =%d",$time, pitch_angle_error);
			$display("%t: %m Roll angle error =%d",$time, roll_angle_error);

			#10 expected_rate = calculate_actual_rate(`THROTTLE_INPUT, task_throttle_target, `DONT_CARE);
			if (!CHECK_EQUALS_RBW(expected_rate, throttle_rate_out)) begin
				// TODO: Create function that prints integer and decimal value
				$display("Test Failed, expected_throttle_rate_out = %d actual_throttle_rate_out = %d\n",
					 expected_rate, throttle_rate_out);
				 $stop;
			end
			
			#10 expected_rate = calculate_actual_rate(`YAW_INPUT, task_yaw_target, `DONT_CARE);
			if (!CHECK_EQUALS_RBW(expected_rate, yaw_rate_out)) begin
				// TODO: Create function that prints integer and decimal value
				$display("Test Failed, expected_yaw_rate_out = %d actual_yaw_rate_out = %d\n",
					 expected_rate, yaw_rate_out);
				 $stop;
			end
			#10 expected_rate = calculate_actual_rate(`PITCH_INPUT, task_pitch_target, task_pitch_actual);
			if (!CHECK_EQUALS_RBW(expected_rate, pitch_rate_out)) begin
				// TODO: Create function that prints integer and decimal value
				$display("Test Failed, expected_pitch_rate_out = %d actual_pitch_rate_out = %d\n",
					 expected_rate, pitch_rate_out);
				 $stop;
			end
			#10 expected_rate = calculate_actual_rate(`ROLL_INPUT, task_roll_target, task_roll_actual);
			if (!CHECK_EQUALS_RBW(expected_rate, roll_rate_out)) begin
				// TODO: Create function that prints integer and decimal value
				$display("Test Failed, expected_pitch_rate_out = %d actual_pitch_rate_out = %d\n",
					 expected_rate, pitch_rate_out);
				 $stop;
			end

		end
	endtask

	// TODO: Add more test cases
	initial begin
		$display("%t: %m Reset angle controller", $time);
		resetn = 1;
		#10 resetn = 0;
		#10 resetn = 1;

		$display("%t: %m Set initial values",$time);
		run_test(
			.task_throttle_target(8'd0),
			.task_yaw_target(8'd125),
			.task_roll_target(8'd125),
			.task_pitch_target(8'd125),
			.task_yaw_actual(`DONT_CARE),
			.task_pitch_actual(0),
			.task_roll_actual(0)
		);

		$display("\n%t: %m Yaw all the way left, throttle to 50 percent with angles of value 0 from IMU",$time);
		run_test(
			.task_throttle_target(125),
			.task_yaw_target(8'd0),
			.task_roll_target(8'd125),
			.task_pitch_target(8'd125),
			.task_yaw_actual(`DONT_CARE),
			.task_pitch_actual(0),
			.task_roll_actual(0)
		);


		$display("\n%t: %m Pitch 140, throttle to 50 percent, 0 linear acceleration, with angles of value 0 from IMU",$time);
		run_test(
			.task_throttle_target(125),
			.task_yaw_target(8'd125),
			.task_roll_target(8'd125),
			.task_pitch_target(8'd140),
			.task_yaw_actual(`DONT_CARE),
			.task_pitch_actual(0),
			.task_roll_actual(0)
		);

		$display("%t: %m Test Success", $time);
		$stop;
	end

		

endmodule

