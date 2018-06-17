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
`include "pid_parameters.v"

module angle_controller (
	output reg  signed [`RATE_BIT_WIDTH-1:0] throttle_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] pitch_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] roll_rate_out,
	output reg  signed [`RATE_BIT_WIDTH-1:0] yaw_angle_error,
	output reg  signed [`RATE_BIT_WIDTH-1:0] pitch_angle_error,
	output reg  signed [`RATE_BIT_WIDTH-1:0] roll_angle_error,
	output reg  active_signal,
	output reg  complete_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] yaw_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] pitch_target,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] roll_target,
	input  wire signed [`RATE_BIT_WIDTH-1:0] yaw_actual,
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


	// Mapping input range to other
	localparam signed [`OPS_BIT_WIDTH-1:0]
		MAPPING_SUBS 		= 500;

	// Mapping input Padding Zeros
	localparam signed
		END_PAD				= 2'b0,
		FRONT_PAD 			= 6'b0,
		THROTTLE_F_PAD		= 4'b0,
		THROTTLE_R_PAD		= 4'b0;

	// working registers
	reg signed [`OPS_BIT_WIDTH-1:0]		mapped_throttle, mapped_yaw, mapped_roll, mapped_pitch;
	reg signed [`OPS_BIT_WIDTH-1:0] 	scaled_throttle, scaled_yaw, scaled_roll, scaled_pitch;
	reg signed [`OPS_BIT_WIDTH-1:0] 	yaw_target_angle_tracking, yaw_target_angle;
	reg signed [`REC_VAL_BIT_WIDTH-1:0]	latched_throttle, latched_pitch, latched_roll;
	reg [`REC_VAL_BIT_WIDTH-1:0]		latched_yaw;

	// state names
	localparam
		STATE_WAITING  = 5'b00001,
		STATE_MAPPING  = 5'b00010,
		STATE_SCALING  = 5'b00100,
		STATE_LIMITING = 5'b01000,
		STATE_COMPLETE = 5'b10000;
		
	// angle value aliases
	// 5760/4 = 1440 = 90˚ and 4320 = 270˚
	localparam
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
		if(!resetn) begin
			state 				<= STATE_WAITING;
			latched_throttle 	<= `ALL_ZERO_2BYTE;
			latched_yaw 		<= `ALL_ZERO_2BYTE;
			latched_pitch 		<= `ALL_ZERO_2BYTE;
			latched_roll 		<= `ALL_ZERO_2BYTE;
			yaw_target_angle_tracking <= `ALL_ZERO_2BYTE;
		end
		else begin
			state 				<= next_state;
			latched_yaw 		<= yaw_target;
			latched_roll 		<= roll_target;
			latched_pitch 		<= pitch_target;
			latched_throttle 	<= throttle_target;
			yaw_target_angle_tracking <= yaw_target_angle_tracking;
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
			yaw_rate_out 	 <= `ALL_ZERO_2BYTE;
			roll_rate_out 	 <= `ALL_ZERO_2BYTE;
			pitch_rate_out	 <= `ALL_ZERO_2BYTE;

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
					
					//Track change in yaw angle
					if (latched_throttle < 10) begin //Throttle is off or nearly off, use current IMU angle as the tracked angle
						$display("Latching IMU rotation value %d", yaw_actual);
						yaw_target_angle_tracking <= yaw_actual;
					end
					else if ( ( yaw_target_angle_tracking + $signed(latched_yaw - 125)) > (ANGLE_360_DEG*4) ) begin // Which means target angle will be > 360˚ and needs to wrap around to something  > 0˚
						yaw_target_angle_tracking <= yaw_target_angle_tracking + $signed(latched_yaw - 125) - ANGLE_360_DEG*4;
						$display("Wrap around in positive direction, tracking angle: %d", yaw_target_angle_tracking);
					end
					else if ( ( yaw_target_angle_tracking + $signed(latched_yaw - 125)) < ANGLE_0_DEG ) begin // Which means target angle will be < 0˚ and needs to wrap around to something  < 360˚
						yaw_target_angle_tracking <= ANGLE_360_DEG*4 + yaw_target_angle_tracking + $signed(latched_yaw - 125);
						$display("Wrap around in negative direction, tracking angle: %d", yaw_target_angle_tracking);
					end
					else begin 
						yaw_target_angle_tracking <= yaw_target_angle_tracking + $signed(latched_yaw - 125);
						$display("Normal update, tracking angle: %d", yaw_target_angle_tracking);
					end
					
					//Divide by 4 to scale this larger number down to ANGLE_360_DEG again, to compare to IMU yaw rotation values
					yaw_target_angle 		<= yaw_target_angle_tracking>>>2;

					mapped_throttle 		<= {THROTTLE_F_PAD, latched_throttle, THROTTLE_R_PAD};
					if((yaw_target_angle > ANGLE_270_DEG) && (yaw_actual < ANGLE_90_DEG)) begin
						mapped_yaw 			<= (ANGLE_360_DEG - yaw_target_angle) + yaw_actual;
						$display("Wrap around in the positive direction, mapped_yaw: %d", mapped_yaw);
					end
					else if( (yaw_actual > ANGLE_270_DEG) && (yaw_target_angle < ANGLE_90_DEG)) begin
						mapped_yaw 			<= (yaw_actual - ANGLE_360_DEG) - yaw_target_angle;
						$display("Wrap around in the negative direction, mapped_yaw: %d", mapped_yaw);
					end
					else begin
						mapped_yaw 			<= yaw_target_angle - yaw_actual;
						$display("Normal update, mapped_yaw: %d", mapped_yaw);
					end
					mapped_roll 			<= $signed({FRONT_PAD, latched_roll,  END_PAD}) - MAPPING_SUBS + roll_actual; // roll value from IMU is flipped, add instead of subtract
					mapped_pitch 			<= $signed({FRONT_PAD, latched_pitch, END_PAD}) - MAPPING_SUBS - pitch_actual;
				end
				STATE_SCALING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					// Apply scaler: (axis_val * scale_multiplier) / Scale_divisor
					scaled_yaw				<= scale_val(mapped_yaw, `YAW_SCALE_MULT, `YAW_SCALE_SHIFT);
					scaled_roll				<= scale_val(mapped_roll, `ROLL_SCALE_MULT, `ROLL_SCALE_SHIFT);
					scaled_pitch 			<= scale_val(mapped_pitch, `PITCH_SCALE_MULT, `PITCH_SCALE_SHIFT);
					scaled_throttle			<= scale_val(mapped_throttle, `THROTTLE_SCALE_MULT, `THROTTLE_SCALE_SHIFT);
				end
				STATE_LIMITING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;

					// Throttle rate limits
					if(scaled_throttle > THROTTLE_MAX)
						throttle_rate_out 	<= THROTTLE_MAX;
					else if(scaled_throttle < `MOTOR_VAL_MIN)
						throttle_rate_out 	<= `MOTOR_VAL_MIN;
					else
						throttle_rate_out	<= scaled_throttle;

					// Yaw rate limits
					if(scaled_yaw > YAW_MAX)
						yaw_rate_out 		<= YAW_MAX;
					else if(scaled_yaw < YAW_MIN)
						yaw_rate_out 		<= YAW_MIN;
					else
						yaw_rate_out 		<= scaled_yaw;

					// Roll rate limits
					if(scaled_roll > ROLL_MAX)
						roll_rate_out 		<= ROLL_MAX;
					else if(scaled_roll < ROLL_MIN)
						roll_rate_out 		<= ROLL_MIN;
					else
						roll_rate_out 		<= scaled_roll;

					// Pitch rate limits
					if(scaled_pitch > PITCH_MAX)
						pitch_rate_out		<= PITCH_MAX;
					else if(scaled_pitch < PITCH_MIN)
						pitch_rate_out 		<= PITCH_MIN;
					else
						pitch_rate_out		<= scaled_pitch;

					yaw_angle_error			<= mapped_yaw;
					pitch_angle_error		<= mapped_pitch;
					roll_angle_error		<= mapped_roll;
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

/**
 *	scale_val function is created to operate the:
 *		output = (input_value x scaling_value) / scaling_shift
 *	this mitigates intermediate register overflow.
 */
function automatic signed [`RATE_BIT_WIDTH-1:0] scale_val;
	input reg signed [`RATE_BIT_WIDTH-1:0]		val;
	input reg signed [`OPS_BIT_WIDTH-1:0]	  	k_mult;
	input reg signed [`SHIFT_OP_BIT_WIDTH-1:0]	k_shift;

	reg signed [31:0]
		val_32,
		scaled;

	localparam
		SHIFT_BACK 	= 7'd16,
		ZERO_PAD	= 16'd0;

	localparam signed [31:0]
		OVERFLOW_PROTECTION_MIN = 32'shFFFF8000,
		OVERFLOW_PROTECTION_MAX = 32'sh00007FFF;

	begin
		// cast values to the 32 bits
		val_32 = $signed({val, ZERO_PAD}) >>> SHIFT_BACK;
		// apply the scalar
		scaled = (val_32 * k_mult) >>> k_shift;

		// make sure we don't output an overflowed value
		if (scaled <= OVERFLOW_PROTECTION_MIN)
			scale_val = OVERFLOW_PROTECTION_MIN;
		else if (scaled >= OVERFLOW_PROTECTION_MAX)
			scale_val = OVERFLOW_PROTECTION_MAX;
		else
			scale_val = $signed(scaled[`RATE_BIT_WIDTH-1:0]);
	end
endfunction

endmodule
