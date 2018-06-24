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

//Defined outside of the module so BUFFER_MAX is known before port list initialization
localparam
	BUFFER_MAX     = 8, //Power of 2 size of buffer
	BUFFER_SHIFT_N = $clog2(BUFFER_MAX);  //Number of bits to count to BUFFER_MAX

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
	reg [`REC_VAL_BIT_WIDTH-1:0] 	latched_throttle_buffer[BUFFER_MAX-1:0];
	reg [`OPS_BIT_WIDTH-1:0]	 	summed_throttle;
	reg [`REC_VAL_BIT_WIDTH-1:0]	average_throttle;

	// state names
	localparam
		STATE_WAITING      = 6'b000001,
		STATE_BUFFERING    = 6'b000010,
		STATE_ADDING       = 6'b000100,
		STATE_AVERAGING    = 6'b001000,
		STATE_LINEAR_SCALE = 6'b010000,
		STATE_COMPLETE     = 6'b100000;

	// state variables
	reg [5:0] state, next_state;

	reg start_flag = `FALSE;
	
	localparam
		THROTTLE_MID_RANGE_LOW_END = 42,
		THROTTLE_MID_RANGE_HIGH_END = 209;

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
				next_state = STATE_LINEAR_SCALE;
			end
			STATE_LINEAR_SCALE: begin
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
					average_throttle 		<= summed_throttle>>BUFFER_SHIFT_N;
				end
				STATE_LINEAR_SCALE: begin
					complete_signal 		<= `FALSE;
					active_signal			<= `TRUE;
					//Low throttle value, gets 2x slope
					if(average_throttle < THROTTLE_MID_RANGE_LOW_END)
						throttle_pwm_value_out  <= (2*average_throttle);
					 //High throttle value also gets 2x slope, throttle = 2*input-252
					else if(average_throttle > THROTTLE_MID_RANGE_HIGH_END)
						throttle_pwm_value_out  <= ((2*average_throttle)-252);
					//Mid range throttle gets x/2 slope, throttle = input/2+61
					else
						throttle_pwm_value_out  <= ((average_throttle>>>1)+61);
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
		latched_throttle_buffer[0] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[1] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[2] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[3] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[4] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[5] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[6] <= `BYTE_ALL_ZERO;
		latched_throttle_buffer[7] <= `BYTE_ALL_ZERO;
	end
endtask

task push_in_buffer;
	input reg [7:0]task_latched_throttle;
	begin
		latched_throttle_buffer[7] <= latched_throttle_buffer[6];
		latched_throttle_buffer[6] <= latched_throttle_buffer[5];
		latched_throttle_buffer[5] <= latched_throttle_buffer[4];
		latched_throttle_buffer[4] <= latched_throttle_buffer[3];
		latched_throttle_buffer[3] <= latched_throttle_buffer[2];
		latched_throttle_buffer[2] <= latched_throttle_buffer[1];
		latched_throttle_buffer[1] <= latched_throttle_buffer[0];
		latched_throttle_buffer[0] <= task_latched_throttle;
	end
endtask

task add_buffer_contents;
	reg [7:0]task_latched_throttle;
	begin
		summed_throttle <=	latched_throttle_buffer[0] +
							latched_throttle_buffer[1] +
							latched_throttle_buffer[2] +
							latched_throttle_buffer[3] +
							latched_throttle_buffer[4] +
							latched_throttle_buffer[5] +
							latched_throttle_buffer[6] +
							latched_throttle_buffer[7];
	end
endtask

endmodule