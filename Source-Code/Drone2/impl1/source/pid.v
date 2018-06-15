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
 * module pid - sub-module implementing a pid controller for  rotation rate
 * around a single axis.
 *
 * Outputs:
 *
 */
 `timescale 1ns / 1ns
 `include "common_defines.v"

 module pid #(parameter signed [15:0] RATE_MIN = 16'h8000,
	 		  parameter signed [15:0] RATE_MAX = 16'h7FFF,
			  parameter signed [15:0] K_P = 1,
			  parameter signed [15:0] K_I = 1,
			  parameter signed [15:0] K_D = 1,
			  parameter signed K_P_SHIFT = 4'h4,
			  parameter signed K_I_SHIFT = 4'h4,
			  parameter signed K_D_SHIFT = 4'h4)
 			 (output reg pid_complete,
			  output reg pid_active,
			  output reg signed  [`PID_RATE_BIT_WIDTH-1:0] 	rate_out,
			  output wire 		 [`DEBUG_WIRE_BIT_WIDTH-1:0]DEBUG_WIRE, /*DEBUG LEDs*/
			  input  wire signed [`RATE_BIT_WIDTH-1:0] 	  	angle_error,
			  input  wire signed [`RATE_BIT_WIDTH-1:0] 	  	target_rotation,
 			  input  wire signed [`IMU_VAL_BIT_WIDTH-1:0]  	actual_rotation,
			  input  wire start_flag,
			  input  wire wait_flag,
			  input  wire resetn,
			  input  wire us_clk);

	// working registers
	reg signed [`OPS_BIT_WIDTH-1:0]
		error_change,
		rotation_total,
		rotation_error,
		prev_rotation_error,
		scaled_rotation,
		rotation_integral,
		rotation_derivative,
		rotation_proportional;

	// state names
	localparam STATE_BIT_WIDTH = 4'd7;
	localparam [STATE_BIT_WIDTH-1:0]
		STATE_WAIT     		= 7'b0000001,
		STATE_EXTEND_32BIT	= 7'b0000010,
		STATE_CALC1    		= 7'b0000100,
		STATE_CALC2    		= 7'b0001000,
		STATE_CALC3    		= 7'b0010000,
		STATE_CALC4    		= 7'b0100000,
		STATE_COMPLETE 		= 7'b1000000;
	// state variables
	reg [STATE_BIT_WIDTH-1:0] state, next_state;

	// Debug wire assign to monitor values on 16 led daughter board
	assign DEBUG_WIRE = rotation_total[`DEBUG_WIRE_BIT_WIDTH-1:0];

	// update state
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn)
			state <= STATE_WAIT;
		else
			state <= next_state;
	end

  // calculation / output logic
  always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			pid_active 		<= `FALSE;
			pid_complete 	<= `FALSE;
			rate_out 		<= `ALL_ZERO_2BYTE;
		end
		else begin
			case(state)
				STATE_WAIT: begin
					pid_active				<= `FALSE;
					pid_complete			<= `TRUE;
				end
				STATE_CALC1: begin
					pid_active 				<= `TRUE;
					pid_complete 			<= `FALSE;
					prev_rotation_error 	<= rotation_error;
					rotation_error 			<= target_rotation - actual_rotation;
					rotation_integral 		<= scale_val(angle_error, K_I, K_I_SHIFT);
				end
				STATE_CALC2: begin
					pid_active 				<= `TRUE;
					pid_complete 			<= `FALSE;
					rotation_proportional 	<= scale_val(rotation_error, K_P, K_P_SHIFT);
					error_change 			<= rotation_error - prev_rotation_error;
				end
				STATE_CALC3: begin
					pid_active 				<= `TRUE;
					pid_complete 			<= `FALSE;
					rotation_derivative 	<= scale_val(error_change, K_D, K_D_SHIFT);
				end
				STATE_CALC4: begin
					pid_active 				<= `TRUE;
					pid_complete 			<= `FALSE;
					rotation_total 			<= rotation_proportional + rotation_integral + rotation_derivative;
				end
				STATE_COMPLETE: begin
					pid_active 				<= `TRUE;
					pid_complete 			<= `TRUE;
					if(rotation_total < RATE_MIN)
						rate_out 			<= RATE_MIN;
					else if(rotation_total > RATE_MAX)
						rate_out 			<= RATE_MAX;
					else
						rate_out 			<= rotation_total;
				end
				default: begin
					pid_active 				<= `FALSE;
					pid_complete 			<= `FALSE;
					rate_out 				<= `ALL_ZERO_2BYTE;
				end
			endcase
		end
	end

	// next state logic
	always @(*) begin
		if(!resetn) begin
		  next_state = STATE_WAIT;
		end
		else begin
			case(state)
				STATE_WAIT: begin
					if(start_flag)
						next_state 	= STATE_CALC1;
					else
						next_state	= STATE_WAIT;
				end
				STATE_CALC1: begin
				   next_state		= STATE_CALC2;
				end
				STATE_CALC2: begin
					next_state 		= STATE_CALC3;
				end
				STATE_CALC3: begin
						next_state 	= STATE_CALC4;
				end
				STATE_CALC4: begin
					next_state 		= STATE_COMPLETE;
				end
				STATE_COMPLETE: begin
					if(wait_flag)
						next_state 	= STATE_WAIT;
					else
						next_state 	= STATE_COMPLETE;
				end
				default: begin
					next_state 		= STATE_WAIT;
				end
			endcase
		end
	end


/**
 * scale_val function is created to operate the:
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
