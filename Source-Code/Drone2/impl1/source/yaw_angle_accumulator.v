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
	output reg  signed [`RATE_BIT_WIDTH-1:0] body_yaw_angle_target,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error_out,
	output reg  active_signal,
	output reg  complete_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_input,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] yaw_pwm_value_input,
	input  wire signed [`RATE_BIT_WIDTH-1:0] yaw_angle_imu,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk);


	reg signed [31:0]	body_yaw_angle_target_tracking;
	reg [`REC_VAL_BIT_WIDTH-1:0]		latched_throttle_pwm_value_input;
	reg signed [`REC_VAL_BIT_WIDTH:0]	latched_yaw_pwm_value_input; //One more bit than receiver value width to fix issues with unsigned input into signed bit
	reg signed [`REC_VAL_BIT_WIDTH:0]	yaw_stick_neutral_value; // 9 bits for the same reason
	reg signed [`RATE_BIT_WIDTH-1:0]    latched_yaw_angle_imu;
	reg  signed [`RATE_BIT_WIDTH-1:0]	yaw_angle_error;
	reg  signed [`RATE_BIT_WIDTH-1:0] trash_2_bytes;
	reg  yaw_stick_out_of_neutral_window = `FALSE;

	// state names
	localparam [5:0]
		STATE_WAITING           = 6'b000001,
		STATE_CALC_TRACK_ANGLE  = 6'b000010,
		STATE_CALC_BODY_ANGLE   = 6'b000100,
		STATE_CALC_ANGLE_ERROR  = 6'b001000,
		STATE_LIMIT_ERROR       = 6'b010000,
		STATE_COMPLETE          = 6'b100000;
		
	// angle value aliases
	// 5760/4 = 1440 = 90˚ and 4320 = 270˚
	localparam signed [15:0]
		ANGLE_360_DEG = 5760, 
		ANGLE_270_DEG = 4320, 
		ANGLE_180_DEG = 2880, 
		ANGLE_90_DEG  = 1440,
		ANGLE_0_DEG   = 0;
	
	// state variables
	reg [5:0] state, next_state;

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
			state	<= STATE_WAITING;
		else
			state	<= next_state;
	end

	// next state logic
	always @(state or start_flag) begin
		case(state)
			STATE_WAITING: begin
				if(start_flag)
					next_state = STATE_CALC_TRACK_ANGLE;
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
				next_state = STATE_LIMIT_ERROR;
			end
			STATE_LIMIT_ERROR: begin
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
			body_yaw_angle_target			<= `ALL_ZERO_2BYTE;
			body_yaw_angle_target_tracking	<= `ALL_ZERO_2BYTE;
			yaw_angle_error					<= `ALL_ZERO_2BYTE;
			yaw_stick_neutral_value			<= {1'b0, `BYTE_ALL_ZERO};
			latched_yaw_pwm_value_input		<= {1'b0, `BYTE_ALL_ZERO};
			yaw_stick_out_of_neutral_window	<= `FALSE;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 					<= `FALSE;
					active_signal 						<= `FALSE;
					latched_throttle_pwm_value_input	<= throttle_pwm_value_input;
					latched_yaw_pwm_value_input     	<= {1'b0, yaw_pwm_value_input};
					latched_yaw_angle_imu           	<= yaw_angle_imu;
				end
				STATE_CALC_TRACK_ANGLE: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) begin //Throttle is off, use current IMU angle as the tracked angle
						body_yaw_angle_target_tracking	<= (latched_yaw_angle_imu*4);
						yaw_stick_out_of_neutral_window	<= `FALSE;
						yaw_stick_neutral_value 		<= latched_yaw_pwm_value_input;
					end
					else if (latched_yaw_pwm_value_input < (yaw_stick_neutral_value+10) && latched_yaw_pwm_value_input > (yaw_stick_neutral_value-10)) begin //Yaw is neutral stick +/-10 PWM units
						if(yaw_stick_out_of_neutral_window) begin // If the yaw stick was previously outside of the neutral window, make target window match current IMU angle
							body_yaw_angle_target_tracking	<= (latched_yaw_angle_imu*4);
							yaw_stick_out_of_neutral_window	<= `FALSE;
						end
						else begin //Otherwise, leave the target angle alone (This prevents drift)
							body_yaw_angle_target_tracking	<= body_yaw_angle_target_tracking;
							yaw_stick_out_of_neutral_window	<= `FALSE;
						end
					end
					else if ( ( body_yaw_angle_target_tracking + (latched_yaw_pwm_value_input - yaw_stick_neutral_value)) > (ANGLE_360_DEG*4) ) begin // Which means target angle will be > 360˚ and needs to wrap around to something  > 0˚
						body_yaw_angle_target_tracking	<= (body_yaw_angle_target_tracking + (latched_yaw_pwm_value_input - yaw_stick_neutral_value) - (ANGLE_360_DEG*4));
						yaw_stick_out_of_neutral_window	<= `TRUE;
					end
					else if ( ( body_yaw_angle_target_tracking + (latched_yaw_pwm_value_input - yaw_stick_neutral_value)) < ANGLE_0_DEG ) begin // Which means target angle will be < 0˚ and needs to wrap around to something  < 360˚
						body_yaw_angle_target_tracking	<= (body_yaw_angle_target_tracking + (latched_yaw_pwm_value_input - yaw_stick_neutral_value) + (ANGLE_360_DEG*4));
						yaw_stick_out_of_neutral_window	<= `TRUE;
					end
					else begin
						body_yaw_angle_target_tracking	<= (body_yaw_angle_target_tracking + (latched_yaw_pwm_value_input - yaw_stick_neutral_value));
						yaw_stick_out_of_neutral_window	<= `TRUE;
					end
				end
				STATE_CALC_BODY_ANGLE: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) //Throttle is off, use current IMU angle as the target angle
						body_yaw_angle_target 		<= latched_yaw_angle_imu;
					else //Divide by 4 to scale this larger number down to ANGLE_360_DEG again, to compare to IMU yaw rotation values
						{trash_2_bytes, body_yaw_angle_target} <= body_yaw_angle_target_tracking>>>2;
				end
				STATE_CALC_ANGLE_ERROR: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					if (latched_throttle_pwm_value_input < 10) //Throttle is off, there can't be angle error
						yaw_angle_error 	<= 0;
					else if((body_yaw_angle_target > ANGLE_270_DEG) && (latched_yaw_angle_imu < ANGLE_90_DEG))
						yaw_angle_error 	<= (body_yaw_angle_target - ANGLE_360_DEG - latched_yaw_angle_imu);
					else if( (latched_yaw_angle_imu > ANGLE_270_DEG) && (body_yaw_angle_target < ANGLE_90_DEG))
						yaw_angle_error 	<= (ANGLE_360_DEG - latched_yaw_angle_imu + body_yaw_angle_target);
					else
						yaw_angle_error 	<= (body_yaw_angle_target - latched_yaw_angle_imu);
				end
				STATE_LIMIT_ERROR: begin // limit error output to +/-180˚ from target angle to prevent wraparound
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					if (yaw_angle_error > ANGLE_90_DEG)
						yaw_angle_error_out	<= (ANGLE_90_DEG>>>2);
					else if (yaw_angle_error < (-ANGLE_90_DEG) )
						yaw_angle_error_out	<= ( (-ANGLE_90_DEG)>>>2);
					else
						yaw_angle_error_out	<= yaw_angle_error;
				end
				STATE_COMPLETE: begin
					complete_signal 		<= `TRUE;
					active_signal 			<= `FALSE;

				end
				default: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
					body_yaw_angle_target 	<= `ALL_ZERO_2BYTE;
					yaw_angle_error			<= `ALL_ZERO_2BYTE;
				end
			endcase
		end
	end

endmodule
