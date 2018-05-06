`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * pwm_to_value - Translates a pwm time high in us to a value between 0 and
 *				  2^(OUTPUT_BIT_WIDTH-1)
 *
 * Outputs:
 * @value: whether or not the address specified is valid
 *
 * Inputs:
 * @pwm_time_high_us: value between MIN_PWM_TIME_HIGH_US and MAX_PWM_TIME_HIGH_US
 * @us_clk: output is updated on every positive edge of the input clock
 */

`include "common_defines.v"

module pwm_to_value (output wire [`PWM_VALUE_BIT_WIDTH - 1:0] value_out,
					 input wire [`PWM_TIME_BIT_WIDTH - 1:0] pwm_time_high_us,
					 input wire us_clk);

			// Ranges from 0 to MIN_PWM_TIME_HIGH_US
			reg [15:0] adjusted_value;

			always @(posedge us_clk) begin
				// Slide the pwm_time_high_us to a value between 0 and MIN_PWM_TIME_HIGH_US
				adjusted_value <= pwm_time_high_us - `MIN_PWM_TIME_HIGH_US;
			end

			// Use truncation as a simplified map() function
			assign value_out = adjusted_value[9:2];

endmodule

