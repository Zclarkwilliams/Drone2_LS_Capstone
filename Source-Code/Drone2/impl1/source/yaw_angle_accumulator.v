/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell,
 * Brett Creeley,
 * Daniel Christiansen,
 * Kirk Hooper,
 * Zachary Clark-Williams
 */

/**
 *  Module takes as inputs:
 *		- Yaw and throttle PWM value from the receiver module
 * 		- 8-bit values
 * 		- The throttle represents a rate, from 0 to max 250
 * 		- The yaw represents a rate, from 0 to max 250
 * 	- Actual yaw angle from the IMU
 * 		- Represent degrees
 * 		- In 16-bit, 2's complement, 12-bits integer, 4-bits fractional
 *
 * Module provides as output (all values are 16-bit, 2's complement):


 */
`timescale 1ns / 1ns

`include "common_defines.v"

module yaw_angle_accumulator (
	output reg  signed [`RATE_BIT_WIDTH-1:0] body_yaw_angle,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error,
	output reg  active_signal,
	output reg  complete_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_input,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] yaw_pwm_value_input,
	input  wire signed [`RATE_BIT_WIDTH-1:0] yaw_angle_imu,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk);


	reg signed [31:0] 				 body_yaw_angle_tracking;
	reg [`REC_VAL_BIT_WIDTH-1:0]	 latched_throttle_pwm_value_input;
	reg signed [8:0]	 			 latched_yaw_pwm_value_input; // This is intentionally 9 bits wide
	reg signed [`RATE_BIT_WIDTH-1:0] latched_yaw_angle_imu;

	// state names
	localparam
		STATE_WAITING           = 5'b00001,
		STATE_CALC_TRACK_ANGLE  = 5'b00010,
		STATE_CALC_BODY_ANGLE   = 5'b00100,
		STATE_CALC_ANGLE_ERROR  = 5'b01000,
		STATE_COMPLETE          = 5'b10000;
		
	// angle value aliases
	// 5760/4 = 1440 = 90˚ and 4320 = 270˚
	localparam signed
		ANGLE_360_DEG = 5760, 
		ANGLE_270_DEG = 4320, 
		ANGLE_180_DEG = 2880, 
		ANGLE_90_DEG  = 1440,
		ANGLE_0_DEG   = 0;
	
	// state variables
	reg [4:0] state, next_state;

	reg start_flag = `FALSE;

	// latch start signal
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			start_flag 		<= `FALSE;
		else if(start_signal && !start_flag)
			start_flag 		<= `TRUE;
		else if(!start_signal && start_flag) begin
			if(state != STATE_WAITING)
				start_flag 	<= `FALSE;
		end
		else
			start_flag 		<= start_flag;
	end

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state 				<= STATE_WAITING;
		else
			state 				<= next_state;
	end

	// next state logic
	always @(state or start_flag) begin
		case(state)
			STATE_WAITING: begin
				if(start_flag) begin
					latched_throttle_pwm_value_input = throttle_pwm_value_input;
					latched_yaw_pwm_value_input      = {1'b0, yaw_pwm_value_input};
					latched_yaw_angle_imu            = yaw_angle_imu;
					next_state                       = STATE_CALC_TRACK_ANGLE;
				end
				else
					next_state = STATE_WAITING;
			end
			STATE_CALC_TRACK_ANGLE: begin
				next_state = STATE_CALC_BODY_ANGLE;
			end
			STATE_CALC_BODY_ANGLE: begin
				next_state = STATE_CALC_ANGLE_ERROR;
			end
			STATE_CALC_ANGLE_ERROR: begin
				next_state = STATE_COMPLETE;
			end
			STATE_COMPLETE: begin
				next_state = STATE_WAITING;
			end
			default: begin
				next_state = STATE_WAITING;
			end
		endcase
	end

	// output logic
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			// reset values
			body_yaw_angle          <= `ALL_ZERO_2BYTE;
			body_yaw_angle_tracking <= `ALL_ZERO_2BYTE;
			yaw_angle_error         <= `ALL_ZERO_2BYTE;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
				end
				STATE_CALC_TRACK_ANGLE: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) begin //Throttle is off or nearly off, use current IMU angle as the tracked angle
						body_yaw_angle_tracking <= (latched_yaw_angle_imu*4);
						$display("body_yaw_angle_tracking obtained from IMU rotation value: %d", (latched_yaw_angle_imu*4));
					end
					else if ( ( body_yaw_angle_tracking + (latched_yaw_pwm_value_input - 125)) > (ANGLE_360_DEG*4) ) begin // Which means target angle will be > 360˚ and needs to wrap around to something  > 0˚
						body_yaw_angle_tracking <= (body_yaw_angle_tracking + (latched_yaw_pwm_value_input - 125) - (ANGLE_360_DEG*4));
						$display("tracking angle wrap around in positive direction");
					end
					else if ( ( body_yaw_angle_tracking + (latched_yaw_pwm_value_input - 125)) < ANGLE_0_DEG ) begin // Which means target angle will be < 0˚ and needs to wrap around to something  < 360˚
						body_yaw_angle_tracking <= (body_yaw_angle_tracking + (latched_yaw_pwm_value_input - 125) + (ANGLE_360_DEG*4));
						$display("tracking angle wrap around in negative direction");
					end
					else begin 
						body_yaw_angle_tracking <= (body_yaw_angle_tracking + (latched_yaw_pwm_value_input - 125));
						$display("tracking angle normal update");
					end
				end
				STATE_CALC_BODY_ANGLE: begin
					$display("body_yaw_angle_tracking=%d", body_yaw_angle_tracking);
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) begin //Throttle is off or nearly off, use current IMU angle as the target angle also
						body_yaw_angle 		<= latched_yaw_angle_imu;
						$display("body_yaw_angle obtained from IMU rotation value due to idle throttle");
					end
					else begin //Divide by 4 to scale this larger number down to ANGLE_360_DEG again, to compare to IMU yaw rotation values
						body_yaw_angle 		<= body_yaw_angle_tracking>>2;
						$display("body_yaw_angle obtained from tracking angle");
					end
				end
				STATE_CALC_ANGLE_ERROR: begin
					$display("body_yaw_angle=%d", body_yaw_angle);
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) begin //Throttle is off or nearly off, use current IMU angle as the target angle also
						yaw_angle_error 	<= 0;
						$display("body_yaw_angle obtained from IMU rotation value due to idle throttle");
					end
					else if((body_yaw_angle > ANGLE_270_DEG) && (latched_yaw_angle_imu < ANGLE_90_DEG)) begin
						yaw_angle_error 	<= (body_yaw_angle - ANGLE_360_DEG - latched_yaw_angle_imu);
						$display("yaw_angle_error wrap around in the positive direction");
					end
					else if( (latched_yaw_angle_imu > ANGLE_270_DEG) && (body_yaw_angle < ANGLE_90_DEG)) begin
						yaw_angle_error 	<= (ANGLE_360_DEG - latched_yaw_angle_imu + body_yaw_angle);
						$display("yaw_angle_error wrap around in the negative direction");
					end
					else begin
						yaw_angle_error 	<= (body_yaw_angle - latched_yaw_angle_imu);
						$display("yaw_angle_error normal update");
					end
				end
				STATE_COMPLETE: begin
					$display("yaw_angle_error=%d", yaw_angle_error);
					complete_signal 		<= `TRUE;
					active_signal 			<= `FALSE;

				end
				default: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
					body_yaw_angle 			<= `ALL_ZERO_2BYTE;
					yaw_angle_error			<= `ALL_ZERO_2BYTE;
				end
			endcase
		end
	end

endmodule
