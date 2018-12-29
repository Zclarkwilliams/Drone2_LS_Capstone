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
`include "pid_parameters.v"

module yaw_angle_accumulator (
	//output reg  signed [`RATE_BIT_WIDTH-1:0] body_yaw_angle_target,
	output reg  signed [`RATE_BIT_WIDTH-1:0]body_yaw_angle_target,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error_out,
	output wire [`RATE_BIT_WIDTH-1:0] debug_out,
	output reg  active_signal,
	output reg  complete_signal,
	output reg yaw_stick_out_of_neutral_window,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_input,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] yaw_pwm_value_input,
	input  wire signed [`RATE_BIT_WIDTH-1:0] yaw_angle_imu,
	input  wire yaac_enable_n,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk
	);
	
	reg signed [`RATE_BIT_WIDTH-1:0]    body_yaw_angle_target_tracking_temp;
	reg signed [`RATE_BIT_WIDTH-1:0]    body_yaw_angle_target_tracking;
	reg signed [`RATE_BIT_WIDTH-1:0]    latched_throttle_pwm_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    latched_yaw_pwm_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    yaw_stick_neutral_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    old_yaw_stick_normalized;
	reg signed [`RATE_BIT_WIDTH-1:0]    latched_yaw_stick_normalized;
	reg signed [`RATE_BIT_WIDTH-1:0]    latched_yaw_angle_imu;
	reg signed [`RATE_BIT_WIDTH-1:0]    old_yaw_angle_imu;
	reg signed [`RATE_BIT_WIDTH-1:0]    yaw_angle_error_temp;
	reg signed [`RATE_BIT_WIDTH-1:0]    yaw_angle_error;
	reg signed [`RATE_BIT_WIDTH-1:0]    old_yaw_angle_error;
	reg old_yaw_stick_out_of_neutral_window;
	
	//Next values of above registers
	reg signed [`RATE_BIT_WIDTH-1:0]    next_body_yaw_angle_target_tracking_temp;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_body_yaw_angle_target_tracking;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_body_yaw_angle_target;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_latched_throttle_pwm_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_latched_yaw_pwm_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_yaw_stick_neutral_value;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_latched_yaw_stick_normalized;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_latched_yaw_angle_imu;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_yaw_angle_error_temp;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_yaw_angle_error;
	reg signed [`RATE_BIT_WIDTH-1:0]    next_yaw_angle_error_out;

	reg next_yaw_stick_out_of_neutral_window;
	reg next_complete_signal;
	reg next_active_signal;
	reg next_old_yaw_stick_out_of_neutral_window;
	reg trigger_body_yaw_angle_target_tracking;
	reg trigger_register_old_values;
	reg start_flag = `FALSE;

	// number of total FSM states, determines the number of required bits for states
	`define NUM_STATES 14

	// state names
	localparam [`NUM_STATES-1:0]

		STATE_INIT                    = `NUM_STATES'b1<<0,
		STATE_WAITING                 = `NUM_STATES'b1<<1,
		STATE_LATCH_VALUES            = `NUM_STATES'b1<<2,
		STATE_GET_NEUTRAL_YAW         = `NUM_STATES'b1<<3,
		STATE_NORMALIZE_STICK         = `NUM_STATES'b1<<4,
		STATE_TRACK_STICK             = `NUM_STATES'b1<<5,
		STATE_CALC_TRACK_ANGLE_TEMP   = `NUM_STATES'b1<<6,
		STATE_CALC_TRACK_ANGLE        = `NUM_STATES'b1<<7,
		STATE_SCALE_TRACK_ANGLE       = `NUM_STATES'b1<<8,
		STATE_CALC_ANGLE_ERROR_TEMP   = `NUM_STATES'b1<<9,
		STATE_CALC_ANGLE_ERROR        = `NUM_STATES'b1<<10,
		STATE_LIMIT_ERROR_CHANGE      = `NUM_STATES'b1<<11,
		STATE_LIMIT_ERROR             = `NUM_STATES'b1<<12,
		STATE_COMPLETE                = `NUM_STATES'b1<<13;
		
	localparam SCALE_BITS = 2; //Accumulated angle maxes at 2^SCALE_BITS before wrapping around
	localparam signed [`RATE_BIT_WIDTH-1:0] YAW_ERROR_MAX_CHANGE = 50; //Maximum amount that yaw angle error is allowed to change per module iteration. 100 counts = 6˚
	                                                   //This value dampens the rate that IMU *and* user inputs can change the drone orientation

	// angle value aliases
	// 16*360˚ = 5760, The IMU max yaw angle is 360˚, which is expressed as an integer with the value 5760
	// 5760/4 = 1440 = 90˚, 3*1440 = 4320 = 270˚, and 2*1440 = 2880 = 180˚
	localparam signed [`RATE_BIT_WIDTH-1:0]
		ANGLE_360_DEG = 5760, 
		ANGLE_270_DEG = 4320, 
		ANGLE_180_DEG = 2880, 
		ANGLE_90_DEG  = 1440,
		ANGLE_10_DEG  = 160,
		ANGLE_0_DEG   = 0,
		
		ANGLE_360_DEG_SCALED = (ANGLE_360_DEG <<<SCALE_BITS), 
		ANGLE_270_DEG_SCALED = (ANGLE_270_DEG <<<SCALE_BITS), 
		ANGLE_180_DEG_SCALED = (ANGLE_180_DEG <<<SCALE_BITS), 
		ANGLE_90_DEG_SCALED  = (ANGLE_90_DEG  <<<SCALE_BITS),
		ANGLE_10_DEG_SCALED  = (ANGLE_10_DEG  <<<SCALE_BITS),
		ANGLE_0_DEG_SCALED   = ANGLE_0_DEG; //Redundant, but here for clarity
	
	// state variables
	reg [`NUM_STATES-1:0] state, next_state;	
	
	//assign debug_out = body_yaw_angle_target;
	//assign debug_out = latched_yaw_stick_normalized;
	//assign debug_out = latched_yaw_pwm_value;
	assign debug_out = yaw_angle_error_temp;

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
		if(!resetn) begin
			state                               <= STATE_INIT;
			complete_signal                     <= `FALSE;
			active_signal                       <= `FALSE;
			body_yaw_angle_target               <= `ALL_ZERO_4BYTE;
			body_yaw_angle_target_tracking_temp <= `ALL_ZERO_4BYTE;
			latched_yaw_stick_normalized        <= `ALL_ZERO_4BYTE;
			yaw_angle_error_temp                <= `ALL_ZERO_4BYTE;
			yaw_angle_error                     <= `ALL_ZERO_4BYTE;
			yaw_angle_error_out                 <= `ALL_ZERO_2BYTE;
			yaw_stick_neutral_value             <= `ALL_ZERO_4BYTE;
			latched_yaw_pwm_value               <= `ALL_ZERO_4BYTE;
			latched_throttle_pwm_value          <= `ALL_ZERO_4BYTE;
			latched_yaw_angle_imu               <= `ALL_ZERO_4BYTE;
			yaw_stick_out_of_neutral_window     <= `FALSE;
		end
		else begin
			state                               <= next_state;
			complete_signal                     <= next_complete_signal;
			active_signal                       <= next_active_signal;
			body_yaw_angle_target               <= next_body_yaw_angle_target;
			body_yaw_angle_target_tracking_temp <= next_body_yaw_angle_target_tracking_temp;
			latched_yaw_stick_normalized        <= next_latched_yaw_stick_normalized;
			yaw_angle_error_temp                <= next_yaw_angle_error_temp;
			yaw_angle_error                     <= next_yaw_angle_error;
			yaw_angle_error_out                 <= next_yaw_angle_error_out;
			yaw_stick_neutral_value             <= next_yaw_stick_neutral_value;
			latched_yaw_pwm_value               <= next_latched_yaw_pwm_value;
			latched_throttle_pwm_value          <= next_latched_throttle_pwm_value;
			latched_yaw_angle_imu               <= next_latched_yaw_angle_imu;
			yaw_stick_out_of_neutral_window     <= next_yaw_stick_out_of_neutral_window;
		end
	end
	
	// Register body_yaw_angle_target_tracking
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			body_yaw_angle_target_tracking     <= `ALL_ZERO_4BYTE;
		end
		else begin
			if(trigger_body_yaw_angle_target_tracking == `TRUE)
				body_yaw_angle_target_tracking <= next_body_yaw_angle_target_tracking;
		end
	end
	
	// Register old values for next iteration
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			old_yaw_angle_imu                   <= `ALL_ZERO_4BYTE;
			old_yaw_angle_error                 <= `ALL_ZERO_4BYTE;
			old_yaw_stick_normalized            <= `ALL_ZERO_4BYTE;
			old_yaw_stick_out_of_neutral_window <= `FALSE;
		end
		else begin
			if(trigger_register_old_values == `TRUE)
			old_yaw_angle_imu                   <= latched_yaw_angle_imu;
			old_yaw_angle_error                 <= yaw_angle_error;
			old_yaw_stick_normalized            <= latched_yaw_stick_normalized;
			old_yaw_stick_out_of_neutral_window <= yaw_stick_out_of_neutral_window;
		end
	end

	// next state logic
	always @(state or start_flag) begin
		case(state)
			STATE_INIT: begin
				next_state = STATE_WAITING;
			end
			STATE_WAITING: begin
				if(start_flag)
					next_state = STATE_LATCH_VALUES;
				else
					next_state = STATE_WAITING;
			end
			STATE_LATCH_VALUES: begin
				next_state = STATE_GET_NEUTRAL_YAW;
			end
			STATE_GET_NEUTRAL_YAW: begin
				next_state = STATE_NORMALIZE_STICK;
			end
			STATE_NORMALIZE_STICK: begin
				next_state = STATE_TRACK_STICK;
			end
			STATE_TRACK_STICK: begin
				next_state = STATE_CALC_TRACK_ANGLE_TEMP;
			end
			STATE_CALC_TRACK_ANGLE_TEMP: begin
				next_state = STATE_CALC_TRACK_ANGLE;
			end
			STATE_CALC_TRACK_ANGLE: begin
				next_state = STATE_SCALE_TRACK_ANGLE;
			end
			STATE_SCALE_TRACK_ANGLE: begin
				next_state = STATE_CALC_ANGLE_ERROR_TEMP;
			end
			STATE_CALC_ANGLE_ERROR_TEMP: begin
				next_state = STATE_CALC_ANGLE_ERROR;
			end
			STATE_CALC_ANGLE_ERROR: begin
				next_state = STATE_LIMIT_ERROR_CHANGE;
			end
			STATE_LIMIT_ERROR_CHANGE: begin
				next_state = STATE_LIMIT_ERROR;
			end
			STATE_LIMIT_ERROR: begin
				next_state = STATE_COMPLETE;
			end
			STATE_COMPLETE: begin
				next_state = STATE_WAITING;
			end
			default: begin
				next_state = STATE_INIT;
			end
		endcase
	end

	// output and FSM logic
	always @(*) begin
		if(!resetn) begin
			// reset values
			next_complete_signal                     = `FALSE;
			next_active_signal                       = `FALSE;
			next_body_yaw_angle_target               = `ALL_ZERO_4BYTE;
			next_body_yaw_angle_target_tracking_temp = `ALL_ZERO_4BYTE;
			next_body_yaw_angle_target_tracking      = `ALL_ZERO_4BYTE;
			next_latched_yaw_stick_normalized        = `ALL_ZERO_4BYTE;
			next_yaw_angle_error                     = `ALL_ZERO_4BYTE;
			next_yaw_angle_error_out                 = `ALL_ZERO_2BYTE;
			next_yaw_stick_neutral_value             = `ALL_ZERO_4BYTE;
			next_latched_yaw_pwm_value               = `ALL_ZERO_4BYTE;
			next_latched_throttle_pwm_value          = `ALL_ZERO_4BYTE;
			next_latched_yaw_angle_imu               = `ALL_ZERO_4BYTE;
			next_yaw_stick_out_of_neutral_window     = `FALSE;
			trigger_body_yaw_angle_target_tracking   = `FALSE;
			trigger_register_old_values              = `FALSE;
		end
		else begin
			// Retain previous value unless otherwise modified in FSM case statement.
			next_body_yaw_angle_target               = body_yaw_angle_target;
			next_body_yaw_angle_target_tracking      = body_yaw_angle_target_tracking;
			next_body_yaw_angle_target_tracking_temp = body_yaw_angle_target_tracking_temp;
			next_latched_yaw_stick_normalized        = latched_yaw_stick_normalized;
			next_yaw_angle_error_temp                = yaw_angle_error_temp;
			next_yaw_angle_error                     = yaw_angle_error;
			next_yaw_angle_error_out                 = yaw_angle_error_out;
			next_yaw_stick_neutral_value             = yaw_stick_neutral_value;
			next_latched_yaw_pwm_value               = latched_yaw_pwm_value;
			next_latched_throttle_pwm_value          = latched_throttle_pwm_value;
			next_latched_yaw_angle_imu               = latched_yaw_angle_imu;
			next_yaw_stick_out_of_neutral_window     = yaw_stick_out_of_neutral_window;
			trigger_body_yaw_angle_target_tracking   = `FALSE;
			trigger_register_old_values              = `FALSE;
			case(state)
                STATE_INIT: begin
					// Good startup values
					next_complete_signal                     = `FALSE;
					next_active_signal                       = `FALSE;
					next_body_yaw_angle_target               = `ALL_ZERO_2BYTE;
					next_body_yaw_angle_target_tracking_temp = `ALL_ZERO_2BYTE;
					next_body_yaw_angle_target_tracking      = `ALL_ZERO_2BYTE;
					next_latched_yaw_stick_normalized        = `ALL_ZERO_2BYTE;
					next_yaw_angle_error_temp                = `ALL_ZERO_2BYTE;
					next_yaw_angle_error                     = `ALL_ZERO_2BYTE;
					next_yaw_angle_error_out                 = `ALL_ZERO_2BYTE;
					next_yaw_stick_neutral_value             = `ALL_ZERO_2BYTE;
					next_latched_yaw_pwm_value               = `ALL_ZERO_2BYTE;
					next_latched_throttle_pwm_value          = `ALL_ZERO_2BYTE;
					next_latched_yaw_angle_imu               = `ALL_ZERO_2BYTE;
					next_yaw_stick_out_of_neutral_window     = `FALSE;
                end
				STATE_WAITING: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `FALSE;
				end
				STATE_LATCH_VALUES: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					next_latched_throttle_pwm_value = {8'd0, throttle_pwm_value_input};
					//latched_yaw_pwm_value     	= $signed({8'b0, yaw_pwm_value_input}*`YAW_ACCUMULATOR_INPUT_MULTIPLIER);
					next_latched_yaw_pwm_value      = {8'd0, yaw_pwm_value_input};
					next_latched_yaw_angle_imu      = yaw_angle_imu;
				end
				STATE_GET_NEUTRAL_YAW: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					
					if (latched_throttle_pwm_value < 10)
						next_yaw_stick_neutral_value = latched_yaw_pwm_value;
					//Otherwise retain the neutral value
					else
						next_yaw_stick_neutral_value = yaw_stick_neutral_value;
				end
				STATE_NORMALIZE_STICK: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					
					next_latched_yaw_stick_normalized = (latched_yaw_pwm_value - yaw_stick_neutral_value);
				end
				STATE_TRACK_STICK: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					
					// Throttle is OFF, use current IMU angle as the tracked angle
					if (latched_throttle_pwm_value < 10)
						next_yaw_stick_out_of_neutral_window = `FALSE;
					// Throttle is ON
					/*
					// Yaw stick crossed over neutral from higher to lower, handle left to right stick bounce
					else if((old_yaw_stick_normalized > 10) && (latched_yaw_stick_normalized < -10 ))
						next_yaw_stick_out_of_neutral_window = `FALSE;
					// Yaw stick crossed over neutral from lower to higher, handle right to left stick bounce
					else if((latched_yaw_stick_normalized > 10) && (old_yaw_stick_normalized < -10 ))
						next_yaw_stick_out_of_neutral_window = `FALSE;
					*/
					// Yaw is neutral stick +/-10 PWM units
					else if ((latched_yaw_stick_normalized  < 4 ) && (latched_yaw_stick_normalized > -4 ))
						next_yaw_stick_out_of_neutral_window = `FALSE;
					// Throttle on and yaw stick not neutral or crossing over
					else
						next_yaw_stick_out_of_neutral_window = `TRUE;
				end
				STATE_CALC_TRACK_ANGLE_TEMP: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					next_body_yaw_angle_target_tracking_temp = (body_yaw_angle_target_tracking + latched_yaw_stick_normalized);
				end
				STATE_CALC_TRACK_ANGLE: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					trigger_body_yaw_angle_target_tracking   = `TRUE;
					
					// Throttle is OFF, use current IMU angle scaled by 2 bits as the tracked angle
					if (latched_throttle_pwm_value < 10)
						next_body_yaw_angle_target_tracking	=(latched_yaw_angle_imu<<<SCALE_BITS);//Multiply IMU angle by 2^SCALE_BITS
					//Yaw stick currently NOT outside of neutral window
					else if (yaw_stick_out_of_neutral_window == `FALSE) begin
						// If the yaw stick was previously outside of the neutral window and is not now,
						// then set target window to current IMU angle scaled by 2 bits
						/*if(old_yaw_stick_out_of_neutral_window == `TRUE)
							next_body_yaw_angle_target_tracking	= (latched_yaw_angle_imu<<<SCALE_BITS);//Multiply IMU angle by 2^SCALE_BITS
						*/
						//Otherwise, leave the target angle alone (This prevents drift)
						//else
							next_body_yaw_angle_target_tracking = body_yaw_angle_target_tracking;
					end
					// Not neutral yaw stick, change target angle angle
					else begin//yaw_stick_out_of_neutral_window == `TRUE and throttle not idle
						// Target angle will be > 360˚ and needs to wrap around to something > 0˚ and < 360˚
						if (body_yaw_angle_target_tracking_temp > ANGLE_360_DEG_SCALED )
							next_body_yaw_angle_target_tracking	= (body_yaw_angle_target_tracking_temp - ANGLE_360_DEG_SCALED);
						// Target angle will be < 0˚ and needs to wrap around to something < 360˚ and > 0˚
						else if (body_yaw_angle_target_tracking_temp < ANGLE_0_DEG_SCALED )
							next_body_yaw_angle_target_tracking	= (body_yaw_angle_target_tracking_temp + ANGLE_360_DEG_SCALED);
						else
							next_body_yaw_angle_target_tracking	= body_yaw_angle_target_tracking_temp;
					end
				end
				STATE_SCALE_TRACK_ANGLE: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					
					//Throttle is off, use current IMU angle as the target angle
					if (latched_throttle_pwm_value < 10)
						next_body_yaw_angle_target = latched_yaw_angle_imu;
					else
						next_body_yaw_angle_target = (body_yaw_angle_target_tracking>>>SCALE_BITS); //Divide track angle by 2^SCALE_BITS
				end
				STATE_CALC_ANGLE_ERROR_TEMP: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					next_yaw_angle_error_temp = (body_yaw_angle_target - latched_yaw_angle_imu);
				end
				STATE_CALC_ANGLE_ERROR: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					//Throttle is off, there can't be angle error
					if (latched_throttle_pwm_value < 10) begin
						next_yaw_angle_error = 0;
					end
					else if((body_yaw_angle_target > ANGLE_270_DEG) && (latched_yaw_angle_imu <  ANGLE_90_DEG) && (yaw_angle_error_temp >= ANGLE_180_DEG)) begin
						next_yaw_angle_error = -(ANGLE_360_DEG - body_yaw_angle_target + latched_yaw_angle_imu);
					end
					else if((body_yaw_angle_target > ANGLE_270_DEG) && (latched_yaw_angle_imu <  ANGLE_90_DEG)) begin
						next_yaw_angle_error = -(body_yaw_angle_target - ANGLE_360_DEG + latched_yaw_angle_imu);
					end					
					else if((latched_yaw_angle_imu > ANGLE_270_DEG) && (body_yaw_angle_target <  ANGLE_90_DEG) && (yaw_angle_error_temp <= -ANGLE_180_DEG)) begin
						next_yaw_angle_error = (ANGLE_360_DEG - latched_yaw_angle_imu + body_yaw_angle_target);
					end
					else if((latched_yaw_angle_imu > ANGLE_270_DEG) && (body_yaw_angle_target <  ANGLE_90_DEG)) begin
						next_yaw_angle_error = (latched_yaw_angle_imu - ANGLE_360_DEG + body_yaw_angle_target);
					end
					///*
					//Just here for safety, numbers should never exceed +/- 360˚
					// Yaw angle error would be < -360˚, wrap to > 0˚ and < 360˚
					else if(yaw_angle_error_temp < -ANGLE_360_DEG) begin
						next_yaw_angle_error = (yaw_angle_error_temp + ANGLE_360_DEG);
					end
					// yaw angle error would be > 360˚, wrap to > 0˚ and < 360˚
					else if(yaw_angle_error_temp > ANGLE_360_DEG) begin
						next_yaw_angle_error = (yaw_angle_error_temp - ANGLE_360_DEG);
					end
					//*/
					//Default case, hit most of the time
					else begin
						next_yaw_angle_error = yaw_angle_error_temp;
					end
				end
				STATE_LIMIT_ERROR_CHANGE: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					///*
					//Prevent wrap from +90˚ to -90˚, retain at +90˚
					if ((old_yaw_angle_error >= ANGLE_90_DEG) && (yaw_angle_error <= -ANGLE_90_DEG))
						next_yaw_angle_error = ANGLE_90_DEG;
					//Prevent wrap from -90˚ to +90˚, retain at -90˚
					else if ((old_yaw_angle_error <= -ANGLE_90_DEG) && (yaw_angle_error >= ANGLE_90_DEG))
						next_yaw_angle_error = -ANGLE_90_DEG;
					//Out of range yaw angle error change, exceeds negative change limit per iteration
					else if (yaw_angle_error < (old_yaw_angle_error - YAW_ERROR_MAX_CHANGE))
						next_yaw_angle_error = old_yaw_angle_error - YAW_ERROR_MAX_CHANGE;
					//Out of range yaw angle error change, exceeds positive change limit per iteration
					else if (yaw_angle_error > (old_yaw_angle_error + YAW_ERROR_MAX_CHANGE))
						next_yaw_angle_error = old_yaw_angle_error + YAW_ERROR_MAX_CHANGE;
					// In range yaw angle error change this iteration
					else
						next_yaw_angle_error = yaw_angle_error;
					//*/
				end
				// Limit actual angle error to +/- 90˚ from target angle to prevent excessive output
				STATE_LIMIT_ERROR: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `TRUE;
					//If YAAc functionality not enabled, set outputs to bypass this module
					if(yaac_enable_n) begin
						next_body_yaw_angle_target = {{(`RATE_BIT_WIDTH-`REC_VAL_BIT_WIDTH){1'b0}}, latched_yaw_pwm_value};
						next_yaw_angle_error_out = 0;
					end
					else if (yaw_angle_error >  ANGLE_90_DEG)
						next_yaw_angle_error_out =  ANGLE_90_DEG;
					else if (yaw_angle_error < -ANGLE_90_DEG)
						next_yaw_angle_error_out = -ANGLE_90_DEG;
					else
						next_yaw_angle_error_out = yaw_angle_error;
				end
				STATE_COMPLETE: begin
					next_complete_signal = `TRUE;
					next_active_signal   = `FALSE;	
					trigger_register_old_values = `TRUE;

				end
				default: begin
					next_complete_signal = `FALSE;
					next_active_signal   = `FALSE;
					next_body_yaw_angle_target               = `ALL_ZERO_2BYTE;
					next_body_yaw_angle_target_tracking_temp = `ALL_ZERO_2BYTE;
					next_body_yaw_angle_target_tracking      = `ALL_ZERO_2BYTE;
					next_latched_yaw_stick_normalized        = `ALL_ZERO_2BYTE;
					next_yaw_angle_error                     = `ALL_ZERO_2BYTE;
					next_yaw_angle_error_out                 = `ALL_ZERO_2BYTE;
					next_yaw_stick_neutral_value             = `ALL_ZERO_2BYTE;
					next_latched_yaw_pwm_value               = `ALL_ZERO_1BYTE;
					next_latched_throttle_pwm_value          = `ALL_ZERO_1BYTE;
					next_latched_yaw_angle_imu               = `ALL_ZERO_4BYTE;
					next_yaw_stick_out_of_neutral_window     = `FALSE;
				end
			endcase
		end
	end


endmodule