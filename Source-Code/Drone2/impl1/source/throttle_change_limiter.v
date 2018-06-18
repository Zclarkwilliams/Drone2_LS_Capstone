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

module throttle_change_limiter (
	output reg [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_out,
	output reg complete_signal,
	output reg active_signal,
	input  wire [`REC_VAL_BIT_WIDTH-1:0] throttle_pwm_value_in,
	input  wire start_signal,
	input  wire resetn,
	input  wire us_clk);

	// working registers
	reg [`REC_VAL_BIT_WIDTH-1:0] 	scaled_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0]	latched_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0] 	latched_throttle_buffer[31:0]; // 32x 20ms (0.64 second) buffer of throttle values
	reg [`OPS_BIT_WIDTH-1:0]	 	summed_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0]	average_throttle;

	// state names
	localparam
		STATE_WAITING     = 5'b00001,
		STATE_BUFFERING   = 5'b00010,
		STATE_ADDING      = 5'b00100,
		STATE_AVERAGING   = 5'b01000,
		STATE_COMPLETE    = 5'b10000;
		
	localparam
		BUFFER_MAX     = 8, //Power of 2 size of buffer
		BUFFER_SHIFT_N = 3;  //Number of bits to count to BUFFER_MAX

	// state variables
	reg [4:0] state, next_state;

	reg start_flag = `FALSE;
	integer i;

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
					next_state = STATE_BUFFERING;
				else
					next_state = STATE_WAITING;
			end
			STATE_BUFFERING: begin
				next_state = STATE_ADDING;
			end
			STATE_ADDING: begin
				next_state = STATE_AVERAGING;
			end
			STATE_AVERAGING: begin
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
			zero_buffer();
			throttle_pwm_value_out	<= `BYTE_ALL_ZERO;
			latched_throttle 		<= `BYTE_ALL_ZERO;
			average_throttle 		<= 0;
			summed_throttle 		<= 0;

		end
		else begin
			case(state)
				STATE_WAITING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `FALSE;
					if(next_state == STATE_BUFFERING)
						latched_throttle 	<= throttle_pwm_value_in;
				end
				STATE_BUFFERING: begin
					complete_signal 		<= `FALSE;
					active_signal 			<= `TRUE;
					if(latched_throttle < 10) //idle throttle, power off
						zero_buffer();
					else
						push_in_buffer(latched_throttle);
				end
				STATE_ADDING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					add_buffer_contents();
				end
				STATE_AVERAGING: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					average_throttle 		= summed_throttle>>BUFFER_SHIFT_N;
					throttle_pwm_value_out  = average_throttle;
				end
				STATE_COMPLETE: begin
					complete_signal 		<= `TRUE;
					active_signal 			<= `FALSE;
					//$display("Throttle input = %d, throttle output = %d", throttle_pwm_value_in, throttle_pwm_value_out);
					$display("%d\t%d", throttle_pwm_value_in, throttle_pwm_value_out);
				end
				default: begin
					throttle_pwm_value_out <= `BYTE_ALL_ZERO;
				end
			endcase
		end
	end

task zero_buffer;
	begin
		//$display("Zeroing throttle buffer");
		for(i = 0; i < BUFFER_MAX; i=i+1) begin
			latched_throttle_buffer[i] <= `BYTE_ALL_ZERO;
			//$display("Throttle buffer @%d = %d", i, latched_throttle_buffer[i]);
		end
	end
endtask

task push_in_buffer;
	input reg [7:0]task_latched_throttle;
	begin
		//$display("Pushing a value into throttle buffer");
		latched_throttle_buffer[0] <= task_latched_throttle;
		//$display("Throttle buffer pushing to buffer head = %d", latched_throttle_buffer[0]);
		for(i = 1; i < BUFFER_MAX; i=i+1) begin
			//$display("Throttle buffer @%d = %d", i, latched_throttle_buffer[i]);
			latched_throttle_buffer[i] <= latched_throttle_buffer[i-1];
		end
	end
endtask

task add_buffer_contents;
	reg [7:0]task_latched_throttle;
	begin
		summed_throttle = 0;
		//$display("Adding the values stored in the throttle buffer");
		for(i = 0; i < BUFFER_MAX; i=i+1) begin
			summed_throttle = summed_throttle+latched_throttle_buffer[i];
			//$display("Throttle buffer adding value @%d = %d, sum = %d", i, latched_throttle_buffer[i], summed_throttle);
		end
		//$display("Buffer sum = %d", summed_throttle);
	end
endtask

endmodule