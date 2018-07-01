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
		STATE_WAITING       = 6'b000001,
		STATE_DEBOUNCE      = 6'b000010,
		STATE_LINEAR_SCALE  = 6'b000100,
		STATE_CHANGE_LIMIT  = 6'b001000,
		STATE_ASSIGN_OUTPUT = 6'b010000,
		STATE_COMPLETE      = 6'b100000;

	// state variables
	reg [5:0] state, next_state;

	reg start_flag = `FALSE;
	
	localparam
		THROTTLE_MID_RANGE_LOW_END = 42,   // The point where the throttle linear scale changes from high change rate to low change rate
		THROTTLE_MID_RANGE_HIGH_END = 209; // The point where the throttle linear scale changes from low change rate back to high change rate
	localparam signed [`OPS_BIT_WIDTH-1:0]
		THROTTLE_CHANGE_LIMIT = 4; // Sets the amount that throttle can change each iteration (Throttle acceleration limit)

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
		end
		else begin
			state 				<= next_state;
		end
	end

	// next state logic
	always @(state or start_flag) begin
		case(state)
			STATE_WAITING: begin
				if(start_flag)
					next_state = STATE_DEBOUNCE;
				else
					next_state = STATE_WAITING;
			end
			STATE_DEBOUNCE: begin
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

	// output logic
	always @(posedge us_clk or negedge resetn) begin
		if(!resetn) begin
			// reset values
			throttle_pwm_value_out	    <= `BYTE_ALL_ZERO;
			latched_throttle 		    <= `BYTE_ALL_ZERO;
			debounced_throttle 		    <= 0;
			limited_throttle 		    <= 0;
			prev_latched_throttle       <=`BYTE_ALL_ZERO;
			prev_throttle_pwm_value_out <=`BYTE_ALL_ZERO;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
					if(next_state == STATE_DEBOUNCE) begin
						if(throttle_pwm_value_in < 10)
							latched_throttle 	<= 8'd0;
						else if(throttle_pwm_value_in > 250)
							latched_throttle 	<= 8'd250;
						else
							latched_throttle 	<= throttle_pwm_value_in;
					end
				end
				STATE_DEBOUNCE: begin //Debounce idle value, was seeing sporadic 0 or low throttle
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;
					/*if(latched_throttle <= 10) begin//idle throttle, power off
						//Was < 10 and is now < 10, not a bounce
						if(prev_latched_throttle <= 10)
							debounced_throttle <= latched_throttle;
						//Was < 10 and is now >= 10, debounce this value
						else
							debounced_throttle <= prev_latched_throttle;
					end
					//Latched value is not idle
					else begin
						//Was >= 10 and is now >= 10, not a bounce
						if(prev_latched_throttle > 10)
							debounced_throttle <= latched_throttle;
						//Was >= 10 and is now < 10, debounce this value
						else
							debounced_throttle <= prev_latched_throttle;
					end*/
					debounced_throttle <= latched_throttle;
				end
				STATE_LINEAR_SCALE: begin 
				complete_signal   <= `FALSE; 
				active_signal    <= `TRUE;
				prev_latched_throttle <= debounced_throttle;
				//Low throttle value, gets 2x slope 
				if(debounced_throttle < THROTTLE_MID_RANGE_LOW_END) 
					scaled_throttle  <= (2'd2*debounced_throttle); 
				//High throttle value also gets 2x slope, throttle = 2*input-252 
				else if(debounced_throttle > THROTTLE_MID_RANGE_HIGH_END) 
					scaled_throttle  <= ((2'd2*debounced_throttle)-8'd252); 
				//Mid range throttle gets x/2 slope, throttle = input/2+61 
				else 
					scaled_throttle  <= ((debounced_throttle>>>1)+6'd61); 
				end
				STATE_CHANGE_LIMIT: begin
					complete_signal          <= `FALSE;
					active_signal            <= `TRUE;
					//Throttle is less than 0, just stop motor
					if(scaled_throttle < 10) begin
						$display("Throttle too low, setting to 0");
						limited_throttle     <= 16'd0;
					end
					//Throttle change is to great in the increasing direction
					else if($signed({1'b0,scaled_throttle}) - $signed({1'b0,prev_throttle_pwm_value_out}) > THROTTLE_CHANGE_LIMIT) begin
						$display("scaled-throttle - prev_throttle_pwm_value_out = %d", ($signed({1'b0,scaled_throttle}) - $signed({1'b0,prev_throttle_pwm_value_out})));
						if(($signed({1'b0,scaled_throttle}) + THROTTLE_CHANGE_LIMIT) > 250) begin
							$display("Limiting to 250");
							limited_throttle <= 16'd250;
							end
						else begin
							$display("Limiting to scaled_throttle + THROTTLE_CHANGE_LIMIT");
							limited_throttle <= prev_throttle_pwm_value_out + THROTTLE_CHANGE_LIMIT;
						end
					end
					//Throttle change is to great in the decreasing direction
					else if($signed({1'b0,prev_throttle_pwm_value_out}) - $signed({1'b0,scaled_throttle}) > THROTTLE_CHANGE_LIMIT) begin
						$display("prev_throttle_pwm_value_out - scaled-throttle = %d", ($signed({1'b0,prev_throttle_pwm_value_out}) - $signed({1'b0,scaled_throttle})));
						if(($signed({1'b0,scaled_throttle}) - THROTTLE_CHANGE_LIMIT) < 10) begin
							$display("Limiting to 0");
							limited_throttle <= 16'd0;
						end
						else begin
							$display("Limiting to scaled_throttle - THROTTLE_CHANGE_LIMIT");
							limited_throttle <= prev_throttle_pwm_value_out - THROTTLE_CHANGE_LIMIT;
						end
					end
					//Throttle change not outside of limit
					else begin
						$display("Not limiting range");
						if(scaled_throttle > 250) begin
							$display("High cutoff at 250");
							limited_throttle <= 16'd250;
						end
						if(scaled_throttle < 10) begin
							$display("Low cutoff at ");
							limited_throttle <= 16'd0;
						end
						else begin
							$display("limited_throttle <= scaled_throttle");
							limited_throttle <= scaled_throttle;
						end
					end
				end
				STATE_ASSIGN_OUTPUT: begin
					complete_signal 	<= `FALSE;
					active_signal 		<= `TRUE;
					{trash_bits, throttle_pwm_value_out}      <= limited_throttle;
				end
				STATE_COMPLETE: begin
					complete_signal 	<= `TRUE;
					active_signal 		<= `FALSE;
					prev_throttle_pwm_value_out <= throttle_pwm_value_out;
					$display("Throttle PWM value out=%d", throttle_pwm_value_out);
				end
				default: begin
					throttle_pwm_value_out <= `BYTE_ALL_ZERO;
				end
			endcase
		end
	end

endmodule