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

module throttle_change_limiter (
	output reg  [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_out,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_in;
	input  wire resetn,
	input  wire us_clk);

	// working registers
	reg [`OPS_BIT_WIDTH-1:0]		mapped_throttle;
	reg [`OPS_BIT_WIDTH-1:0] 		scaled_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0]	latched_throttle;
	reg [`OPS_BIT_WIDTH-1:0] 		latched_throttle_buffer;

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
			latched_throttle 	<= `BYTE_ALL_ZERO;
		end
		else begin
			state 				<= next_state;
			latched_throttle 	<= throttle_pwm_value_in;
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
			throttle_pwm_value_out <= `BYTE_ALL_ZERO_2BYTE;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
				end
				STATE_MAPPING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;;
				end
				STATE_SCALING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
				end
				STATE_LIMITING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
				end
				STATE_COMPLETE: begin
					complete_signal 		<= `TRUE;
					active_signal 			<= `FALSE;
				end
				default: begin
					throttle_pwm_value_in <= `BYTE_ALL_ZERO;
				end
			endcase
		end
	end
endmodule
