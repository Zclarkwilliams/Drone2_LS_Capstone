`timescale 1ns/1ns

/*
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * pwm_generator_block - Generates a single PWM signal.
 *
 * Outputs:
 * @motor_pwm: signal to drive a ESC/motor
 *
 * Inputs:
 * @motor_val: ?
 * @period_counter: ?
 * @high_counter: ?
 * @resetn: module reset signal
 * @us_clk: 1 MHz clock (1 us period)
 */

`include "common_defines.v"

module pwm_generator_block #(parameter INPUT_BIT_WIDTH = 10)
							(output wire motor_pwm,
							 output wire [2:0] state_out,
							 input wire [INPUT_BIT_WIDTH-1:0] motor_val,
							 input wire [15:0] period_counter,
							 input wire [INPUT_BIT_WIDTH-1:0] high_counter,
							 input wire resetn,
							 input wire us_clk);

	assign state_out = state;

	localparam // State names
		STATE_MIN_COUNT = 3'b001,
		STATE_PWM_COUNT = 3'b010,
		STATE_LOW_COUNT = 3'b100;

	// State variables
	reg [2:0] state, next_state;

	// Update state
	always @(posedge us_clk or negedge resetn) begin
		if (!resetn)
			state <= STATE_MIN_COUNT;
		else
			state <= next_state;
	end

	// Next state logic
	always @(*) begin
		case (state)
			STATE_MIN_COUNT: begin
				if (!resetn)
					next_state = STATE_MIN_COUNT;
				else if (period_counter == `MIN_PWM_TIME_HIGH_US)
					next_state = STATE_PWM_COUNT;
				else
					next_state = STATE_MIN_COUNT;
			end
			STATE_PWM_COUNT: begin
				if (!resetn)
					next_state = STATE_MIN_COUNT;
				else if ((high_counter == motor_val) || (period_counter == `MAX_PWM_TIME_HIGH_US))
					next_state = STATE_LOW_COUNT;
				else next_state = STATE_PWM_COUNT;
			end
			STATE_LOW_COUNT: begin
				if (!resetn || (period_counter == `PWM_PERIOD_US))
					next_state = STATE_MIN_COUNT;
				else next_state = STATE_LOW_COUNT;
			end
			default:
				next_state = STATE_MIN_COUNT;
		endcase
	end

	// Output logic
	assign motor_pwm = ((state == STATE_MIN_COUNT) || (state == STATE_PWM_COUNT));

endmodule
