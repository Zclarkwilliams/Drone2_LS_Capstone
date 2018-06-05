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
 *		- Target rate & angles from the receiver module
 * 		- 8-bit values
 * 		- The throttle represents a rate, from 0 to max (???)
 * 		- The yaw is a rate, from  a negative min to a positive max
 * 		- The pitch and roll represent target angles (degrees)
 * 	- Actual pitch and roll angles from the IMU
 * 		- Represent degrees
 * 		- In 16-bit, 2's complement, 12-bits integer, 4-bits fractional
 *
 * Module provides as output (all values are 16-bit, 2's complement):
 *		- Limited throttle rate (>= 0)
 *		- Limited yaw, pitch, and roll rates
 * 	- Represent degrees/second
 *
 * TODO:
 *		Rate limits???
 *		Update this header description to look like other files
 */
`timescale 1ns / 1ns

`include "common_defines.v"

module angle_controller (
	output reg  signed [`RATE_BIT_WIDTH-1:0] throttle_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] pitch_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] roll_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] pitch_angle_error,
	output reg  signed [`RATE_BIT_WIDTH-1:0] roll_angle_error,
	output reg  active_signal,
	output reg  complete_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] yaw_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] pitch_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] roll_target,
	input  wire signed [`RATE_BIT_WIDTH-1:0] yaw_actual /* synthesis syn_force_pads=1 syn_noprune=1*/ ,
	input  wire signed [`RATE_BIT_WIDTH-1:0] pitch_actual,
	input  wire signed [`RATE_BIT_WIDTH-1:0] roll_actual,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk);

	// rate limits (16-bit, 2's complement, 12-bit integer, 4-bit fractional)
	localparam signed [`OPS_BIT_WIDTH-1:0]
		THROTTLE_MAX 	=  250 << `FIXED_POINT_SHIFT,
		YAW_MAX 		=  100 << `FIXED_POINT_SHIFT,
		YAW_MIN 		= -100 << `FIXED_POINT_SHIFT,
		PITCH_MAX 		=  100 << `FIXED_POINT_SHIFT,
		PITCH_MIN 		= -100 << `FIXED_POINT_SHIFT,
		ROLL_MAX		=  100 << `FIXED_POINT_SHIFT,
		ROLL_MIN		= -100 << `FIXED_POINT_SHIFT;

	// scale factors (16-bit, 2's complement, 12-bit integer, 4-bit fractional)
	// Multiplier Scaler Values
	localparam signed [`OPS_BIT_WIDTH-1:0]
		YAW_SCALE_MULT 		= 1,
		ROLL_SCALE_MULT 	= 36,
		PITCH_SCALE_MULT 	= 32,
		THROTTLE_SCALE_MULT	= 1;
	
	//Divisor Shift Values
	localparam signed [`SHIFT_OP_BIT_WIDTH-1:0]	
		YAW_SCALE_DIV 		= 0,
		PITCH_SCALE_DIV 	= 4,
		ROLL_SCALE_DIV	 	= 4,
		THROTTLE_SCALE_DIV	= 0;
		
	// Mapping input range to other
	localparam signed [`OPS_BIT_WIDTH-1:0]
		MAPPING_SUBS 		= 500;
	
	// Pad with zeros on front to compare as 32 bit number
	localparam signed MOTOR_VAL_MIN_32BIT = {`PADDING_ZEROS, `MOTOR_VAL_MIN};
	
	// Mapping input Padding Zeros
	localparam signed
		END_PAD				=  2'b0,
		FRONT_PAD 			= 24'b0,
		THROTTLE_F_PAD		= 20'b0,
		THROTTLE_R_PAD		=  4'b0;

	// working registers
	reg signed [`OPS_BIT_WIDTH-1:0]		mapped_throttle, mapped_yaw, mapped_roll, mapped_pitch;
	reg signed [`OPS_BIT_WIDTH-1:0] 	scaled_throttle, scaled_yaw, scaled_roll, scaled_pitch;
	reg signed [`REC_VAL_BIT_WIDTH-1:0]	latched_throttle, latched_yaw, latched_pitch, latched_roll;

	// state names
	localparam
		STATE_WAITING  = 5'b00001,
		STATE_MAPPING  = 5'b00010,
		STATE_SCALING  = 5'b00100,
		STATE_LIMITING = 5'b01000,
		STATE_COMPLETE = 5'b10000;

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
		if(!resetn) begin
			state 				<= STATE_WAITING;
			latched_throttle 	<= `ALL_ZERO_2BYTE;
			latched_yaw 		<= `ALL_ZERO_2BYTE;
			latched_pitch 		<= `ALL_ZERO_2BYTE;
			latched_roll 		<= `ALL_ZERO_2BYTE;
		end
		else begin
			state 				<= next_state;
			latched_throttle 	<= throttle_target;
			latched_yaw 		<= yaw_target;
			latched_pitch 		<= pitch_target;
			latched_roll 		<= roll_target;
		end
	end

	// next state logic
	always @(*) begin
		case(state)
			STATE_WAITING: begin
				if(start_flag)
					next_state = STATE_MAPPING;
				else
					next_state = STATE_WAITING;
			end
			STATE_MAPPING: begin
				next_state = STATE_SCALING;
			end
			STATE_SCALING: begin
				next_state = STATE_LIMITING;
			end
			STATE_LIMITING: begin
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
			yaw_rate_out 	<= `ALL_ZERO_2BYTE;
			roll_rate_out 	<= `ALL_ZERO_2BYTE;
			pitch_rate_out	<= `ALL_ZERO_2BYTE;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
				end
				STATE_MAPPING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;

					mapped_throttle 		<= {THROTTLE_F_PAD, latched_throttle, THROTTLE_R_PAD};
					// input values mapped from 0 - 250 to -31.25 - 31.25
					mapped_yaw 				<= $signed({FRONT_PAD, latched_yaw,   END_PAD}) - MAPPING_SUBS;
					mapped_roll 			<= $signed({FRONT_PAD, latched_roll,  END_PAD}) - MAPPING_SUBS + roll_actual; // roll value from IMU is flipped, add instead of subtract
					mapped_pitch 			<= $signed({FRONT_PAD, latched_pitch, END_PAD}) - MAPPING_SUBS - pitch_actual;
				end
				STATE_SCALING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					// Apply scaler: (axis_val * scale_multiplier) / Scale_divisor
					scaled_yaw				<= (mapped_yaw * YAW_SCALE_MULT)			>>> YAW_SCALE_DIV;
					scaled_roll				<= (mapped_roll * ROLL_SCALE_MULT) 			>>> ROLL_SCALE_DIV;
					scaled_pitch 			<= (mapped_pitch * PITCH_SCALE_MULT) 		>>> PITCH_SCALE_DIV;
					scaled_throttle			<= (mapped_throttle * THROTTLE_SCALE_MULT) 	>>> THROTTLE_SCALE_DIV;
				end
				STATE_LIMITING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;

					// Throttle rate limits
					if(scaled_throttle > THROTTLE_MAX)
						throttle_rate_out 	<= THROTTLE_MAX;
					else if(scaled_throttle < MOTOR_VAL_MIN_32BIT)
						throttle_rate_out 	<= `MOTOR_VAL_MIN;
					else
						throttle_rate_out	<= scaled_throttle[`BITS_EXTRACT-1:0];
					
					// Yaw rate limits
					if(scaled_yaw > YAW_MAX)
						yaw_rate_out 		<= YAW_MAX[`BITS_EXTRACT-1:0];
					else if(scaled_yaw < YAW_MIN)
						yaw_rate_out 		<= YAW_MIN[`BITS_EXTRACT-1:0];
					else
						yaw_rate_out 		<= scaled_yaw[`BITS_EXTRACT-1:0];
					
					// Roll rate limits
					if(scaled_roll > ROLL_MAX)
						roll_rate_out 		<= ROLL_MAX[`BITS_EXTRACT-1:0];
					else if(scaled_roll < ROLL_MIN)
						roll_rate_out 		<= ROLL_MIN[`BITS_EXTRACT-1:0];
					else
						roll_rate_out 		<= scaled_roll[`BITS_EXTRACT-1:0];
					
					// Pitch rate limits
					if(scaled_pitch > PITCH_MAX)
						pitch_rate_out		<= PITCH_MAX[`BITS_EXTRACT-1:0];
					else if(scaled_pitch < PITCH_MIN)
						pitch_rate_out 		<= PITCH_MIN[`BITS_EXTRACT-1:0];
					else
						pitch_rate_out		<= scaled_pitch[`BITS_EXTRACT-1:0];

					pitch_angle_error		<= mapped_pitch[`BITS_EXTRACT-1:0];
					roll_angle_error		<= mapped_roll[`BITS_EXTRACT-1:0];
				end
				STATE_COMPLETE: begin
					complete_signal 		<= `TRUE;
					active_signal 			<= `FALSE;
				end
				default: begin
					pitch_rate_out 			<= `ALL_ZERO_2BYTE;
					yaw_rate_out 			<= `ALL_ZERO_2BYTE;
					roll_rate_out 			<= `ALL_ZERO_2BYTE;
					throttle_rate_out		<= `ALL_ZERO_2BYTE;
				end
			endcase
		end
	end
endmodule