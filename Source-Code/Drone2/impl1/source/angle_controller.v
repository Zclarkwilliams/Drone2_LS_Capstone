`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
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
 *		Change non-blocking to blocking
 *		Clean up defines/localparams
 *		Rate limits???
 *		Optimize resource usage?
 *		Update this header description to look like other files
 */
module angle_controller
	#(parameter RATE_BIT_WIDTH = 16,  // size of target output values
	parameter IMU_VAL_BIT_WIDTH = 16, // size of input from IMU
	parameter REC_VAL_BIT_WIDTH = 8)  // size of input from receiver
	(output reg [RATE_BIT_WIDTH-1:0] throttle_rate_out,
	output reg [RATE_BIT_WIDTH-1:0] yaw_rate_out,
	output reg [RATE_BIT_WIDTH-1:0] pitch_rate_out,
	output reg [RATE_BIT_WIDTH-1:0] roll_rate_out,
	output reg [RATE_BIT_WIDTH-1:0] pitch_angle_error,
	output reg [RATE_BIT_WIDTH-1:0] roll_angle_error,
	output reg complete_signal,
	output reg active_signal,
	input wire [REC_VAL_BIT_WIDTH-1:0] throttle_target,
	input wire [REC_VAL_BIT_WIDTH-1:0] yaw_target,
	input wire [REC_VAL_BIT_WIDTH-1:0] pitch_target,
	input wire [REC_VAL_BIT_WIDTH-1:0] roll_target,
	input wire [RATE_BIT_WIDTH-1:0] pitch_actual,
	input wire [RATE_BIT_WIDTH-1:0] roll_actual,
	input wire resetn,
	input wire start_signal,
	input wire us_clk);

	// TODO: Use decimal values instead to avoid having to comment
	// rate limits (16-bit, 2's complement, 12-bit integer, 4-bit fractional)
	localparam signed
		THROTTLE_MAX = 16'h0fc0, // 60
		YAW_MAX =      16'h0190, // 25
		YAW_MIN =      16'hfe70, // -25
		PITCH_MAX =    16'h0190, // 25
		PITCH_MIN =    16'hfe70, // -25
		ROLL_MAX =     16'h0190, // 25
		ROLL_MIN =     16'hfe70; // -25

	// scale factors (16-bit, 2's complement, 12-bit integer, 4-bit fractional)
	localparam signed
		THROTTLE_SCALE = 16'h0001, // 1
		YAW_SCALE =      16'h0001, // 1
		ROLL_SCALE =     16'h0001, // 1
		PITCH_SCALE =    16'h0001; // 1

	// working registers
	reg signed [RATE_BIT_WIDTH-1:0] mapped_throttle, mapped_yaw, mapped_roll, mapped_pitch;
	reg signed [RATE_BIT_WIDTH-1:0] scaled_throttle, scaled_yaw, scaled_roll, scaled_pitch;

	// state names
	localparam
		STATE_WAITING =  5'b00001,
		STATE_MAPPING =  5'b00010,
		STATE_SCALING =  5'b00100,
		STATE_LIMITING = 5'b01000,
		STATE_COMPLETE = 5'b10000;

	// state variables
	reg [4:0] state, next_state;

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= STATE_WAITING;
		else
			state <= next_state;
	end

	// next state logic
	always @(*) begin
		case(state)
			STATE_WAITING: begin
				if(start_signal) next_state = STATE_MAPPING;
				else next_state = STATE_WAITING;
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
		endcase
	end

	// TODO change non-blocking to blocking
	// output logic
	always @(state) begin
		case(state)
			STATE_WAITING: begin
				complete_signal = 1'b0;
				active_signal = 1'b0;
				if(!resetn) begin
					yaw_rate_out = 16'h0000;
					roll_rate_out = 16'h0000;
					pitch_rate_out = 16'h0000;
				end
			end
			STATE_MAPPING: begin
				complete_signal = 1'b0;
				active_signal = 1'b1;

        // TODO Change these value ranges!
				// input value mapped from  0 - 250 to 0 - 62.5
				mapped_throttle = {6'b000000, throttle_target, 2'b00}; // ???
				// input values mapped from 0 - 250 to -31.25 - 31.25
				mapped_yaw = {6'b000000, yaw_target, 2'b00} - 500;
				mapped_roll = ({6'b000000, roll_target, 2'b00} - 500) - roll_actual;
				mapped_pitch = ({6'b000000, pitch_target, 2'b00} - 500) - pitch_actual;
			end
			STATE_SCALING: begin
				complete_signal = 1'b0;
				active_signal = 1'b1;

				// the decimal point should be shifted...
				scaled_throttle = mapped_throttle * THROTTLE_SCALE;
				scaled_yaw = mapped_yaw * YAW_SCALE;
				scaled_roll = mapped_roll * ROLL_SCALE;
				scaled_pitch = mapped_pitch * PITCH_SCALE;
			end
			STATE_LIMITING: begin
				complete_signal = 1'b0;
				active_signal = 1'b1;

				// apply rate limits
				if(mapped_throttle > THROTTLE_MAX)
					throttle_rate_out = THROTTLE_MAX;
				else
					throttle_rate_out = mapped_throttle;

				if(mapped_yaw > YAW_MAX)
					yaw_rate_out = YAW_MAX;
				else if(mapped_yaw < YAW_MIN)
					yaw_rate_out = YAW_MIN;
				else
					yaw_rate_out = mapped_yaw;

				if(mapped_roll > ROLL_MAX)
					roll_rate_out = ROLL_MAX;
				else if(mapped_roll < ROLL_MIN)
					roll_rate_out = ROLL_MIN;
				else
					roll_rate_out = mapped_roll;

				if(mapped_pitch > PITCH_MAX)
					pitch_rate_out = PITCH_MAX;
				else if(mapped_pitch < PITCH_MIN)
					pitch_rate_out = PITCH_MIN;
				else
					pitch_rate_out = mapped_pitch;

				pitch_angle_error = mapped_pitch;
				roll_angle_error = mapped_roll;
			end
			STATE_COMPLETE: begin
				complete_signal = 1'b1;
				active_signal = 1'b0;
			end
		endcase
	end

endmodule

