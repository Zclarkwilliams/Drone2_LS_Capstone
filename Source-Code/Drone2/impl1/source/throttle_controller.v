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

 */
`timescale 1ns / 1ns

`include "common_defines.v"

module throttle_controller
	(
	output reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_out,
	output reg complete_signal,
	output reg active_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_in,
	input  wire tc_enable_n,
	input  wire [2:0] switch_a,
	input  wire imu_ready,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk);

	// working registers
	reg [`REC_VAL_BIT_WIDTH-1:0]	latched_throttle;
	reg [`OPS_BIT_WIDTH-1:0]	 	debounced_throttle;
	reg [`OPS_BIT_WIDTH-1:0]	    scaled_throttle;
	reg signed [`OPS_BIT_WIDTH-1:0]	limited_throttle;
	reg [`OPS_BIT_WIDTH-`REC_VAL_BIT_WIDTH-1:0]	trash_bits;
	reg [`REC_VAL_BIT_WIDTH-1:0] 	prev_latched_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0]    prev_throttle_pwm_value_out;
	//reg [15:0]	trash_bits;

	// state names
	localparam
		STATE_INIT          = 7'b0000001,
		STATE_WAITING       = 7'b0000010,
		STATE_DEGLITCH      = 7'b0000100,
		STATE_LINEAR_SCALE  = 7'b0001000,
		STATE_CHANGE_LIMIT  = 7'b0010000,
		STATE_ASSIGN_OUTPUT = 7'b0100000,
		STATE_COMPLETE      = 7'b1000000;

	// state variables
	reg [6:0] state, next_state;

	reg start_flag = `FALSE;
	
	localparam signed [`OPS_BIT_WIDTH-1:0]
		THROTTLE_MID_RANGE_LOW_END = 42,   // The point where the throttle linear scale changes from high change rate to low change rate
		THROTTLE_MID_RANGE_HIGH_END = 209, // The point where the throttle linear scale changes from low change rate back to high change rate
		THROTTLE_CHANGE_LIMIT = 5;         // Sets the amount that throttle can change each iteration (Throttle acceleration limit)

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
			state 				<= STATE_INIT;
		end
		else begin
			state 				<= next_state;
		end
	end

	// next state logic
	always @(state or start_flag or imu_ready or resetn) begin
		if (!resetn)
			next_state = STATE_INIT;
		else begin
			case(state)
				STATE_INIT: begin
					if(imu_ready == `TRUE)
						next_state = STATE_WAITING;
					else
						next_state = STATE_INIT;
				end
				STATE_WAITING: begin
					if(start_flag)
						next_state = STATE_DEGLITCH;
					else
						next_state = STATE_WAITING;
				end
				STATE_DEGLITCH: begin
					next_state = STATE_LINEAR_SCALE;
				end
				STATE_LINEAR_SCALE: begin
					next_state = STATE_CHANGE_LIMIT;
				end
				STATE_CHANGE_LIMIT: begin
					next_state = STATE_ASSIGN_OUTPUT;
				end
				STATE_ASSIGN_OUTPUT: begin
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
	end

	// output logic
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			// reset values
			throttle_pwm_value_out	    <= `BYTE_ALL_ZERO;
			latched_throttle 		    <= `BYTE_ALL_ZERO;
			debounced_throttle 		    <= 0;
			limited_throttle 		    <= 0;
			prev_latched_throttle       <= `BYTE_ALL_ZERO;
			prev_throttle_pwm_value_out <= `BYTE_ALL_ZERO;

		end
		else begin
			case(state)
				STATE_INIT: begin
					complete_signal 		<= `TRUE; // Signal next module to start but set this module output to 0
					active_signal 			<= `FALSE;
					throttle_pwm_value_out  <=  0;
				end
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
					if(next_state == STATE_DEGLITCH) begin
						if(throttle_pwm_value_in < 8'd10)
							latched_throttle 	<= 8'd0;
						else if(throttle_pwm_value_in > 8'd250)
							latched_throttle 	<= 8'd250;
						else
							latched_throttle 	<= throttle_pwm_value_in;
					end
				end
				STATE_DEGLITCH: begin //De-glitch idle value, was seeing sporadic 0 or low throttle while running
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;
					if(latched_throttle <= 8'd10) begin//idle throttle, power off
						//Was <= 10 and is now <= 10, does not need de-glitching
						if(prev_latched_throttle <= 8'd10) begin
							debounced_throttle    <= latched_throttle;
							prev_latched_throttle <= latched_throttle;
						end
						//Was > 10 and is now <= 10, and there is a previous value, de-glitch this value
						else if (prev_latched_throttle > 8'd10) begin
							debounced_throttle    <= prev_latched_throttle;
							prev_latched_throttle <= latched_throttle;
						end
						//No previous value to de-glitch with
						else begin
							debounced_throttle    <= latched_throttle;
							prev_latched_throttle <= latched_throttle;
						end
					end
					//Latched value is not idle
					else begin
						prev_latched_throttle <= latched_throttle;
						debounced_throttle    <= latched_throttle;
					end
				end
				STATE_LINEAR_SCALE: begin 
					complete_signal      <= `FALSE; 
					active_signal        <= `TRUE;
					if (switch_a == 3'b001) begin //Easy mode throttle
						//Low throttle value, gets 4x slope 
						if(debounced_throttle < 8'd35)
							scaled_throttle  <= (8'd4*debounced_throttle); 
						else
						//Mid range throttle gets x/4 slope, throttle = input/4+98 
							scaled_throttle  <= ((debounced_throttle>>2)+8'd128); 
					end
					else begin //Normal/Acro mode throttle
						//Low throttle value, gets 2x slope 
						if(debounced_throttle < THROTTLE_MID_RANGE_LOW_END) 
							scaled_throttle  <= (8'd2*debounced_throttle); 
						//High throttle value also gets 2x slope, throttle = 2*input-252 
						else if(debounced_throttle > THROTTLE_MID_RANGE_HIGH_END) 
							scaled_throttle  <= ((8'd2*debounced_throttle)-8'd252); 
						//Mid range throttle gets x/2 slope, throttle = input/2+61 
						else 
							scaled_throttle  <= ((debounced_throttle>>>1)+8'd61); 
						end
					end
				STATE_CHANGE_LIMIT: begin
					complete_signal          <= `FALSE;
					active_signal            <= `TRUE;
					//Throttle is less than 0, just stop motor
					if(scaled_throttle < 8'd10)
						limited_throttle     <= 16'd0;
					//Throttle change is to great in the increasing direction
					else if(scaled_throttle > (prev_throttle_pwm_value_out + THROTTLE_CHANGE_LIMIT)) begin
						if((prev_throttle_pwm_value_out + THROTTLE_CHANGE_LIMIT) > 'd250)
							limited_throttle <= 16'd250;
						else
							limited_throttle <= prev_throttle_pwm_value_out + THROTTLE_CHANGE_LIMIT;
					end
					//Throttle change is to great in the decreasing direction
					else if( prev_throttle_pwm_value_out > (scaled_throttle + THROTTLE_CHANGE_LIMIT)) begin
						if((prev_throttle_pwm_value_out - THROTTLE_CHANGE_LIMIT) < 'd10)
							limited_throttle <= 16'd0;
						else
							limited_throttle <= prev_throttle_pwm_value_out - THROTTLE_CHANGE_LIMIT;
					end
					//Throttle change not outside of limit
					else begin
						if(scaled_throttle > 250)
							limited_throttle <= 16'd250;
						if(scaled_throttle < 10)
							limited_throttle <= 16'd0;
						else
							limited_throttle <= scaled_throttle;
					end
				end
				STATE_ASSIGN_OUTPUT: begin
					complete_signal 	                 <= `FALSE;
					active_signal 		                 <= `TRUE;
					if(tc_enable_n)
						throttle_pwm_value_out <= latched_throttle;
					else
						{trash_bits, throttle_pwm_value_out} <= limited_throttle;
				end
				STATE_COMPLETE: begin
					complete_signal 	                 <= `TRUE;
					active_signal 		                 <= `FALSE;
					prev_throttle_pwm_value_out          <= throttle_pwm_value_out;
				end
				default: begin
					throttle_pwm_value_out <= `BYTE_ALL_ZERO;
				end
			endcase
		end
	end

endmodule
