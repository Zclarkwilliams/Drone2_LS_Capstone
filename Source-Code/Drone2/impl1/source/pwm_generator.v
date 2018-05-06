`timescale 1ns/1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * pwm_generator - Generates PWM signals for the ESC(s).
 *
 * Outputs:
 * @motor_1_pwm: signal to drive motor 1
 * @motor_2_pwm: signal to drive motor 2
 * @motor_3_pwm: signal to drive motor 3
 * @motor_4_pwm: signal to drive motor 4
 *
 * Inputs:
 * @motor_1_rate: rate to run motor 1 at (units?)
 * @motor_2_rate: rate to run motor 2 at (units?)
 * @motor_3_rate: rate to run motor 3 at (units?)
 * @motor_4_rate: rate to run motor 4 at (units?)
 */

`include "common_defines.v"

module pwm_generator #(parameter INPUT_BIT_WIDTH = 8)
					  (output wire motor_1_pwm,
					   output wire motor_2_pwm,
					   output wire motor_3_pwm,
					   output wire motor_4_pwm,
					   output wire [2:0] state_out,
					   input wire [INPUT_BIT_WIDTH - 1:0] motor_1_rate,
					   input wire [INPUT_BIT_WIDTH - 1:0] motor_2_rate,
					   input wire [INPUT_BIT_WIDTH - 1:0] motor_3_rate,
					   input wire [INPUT_BIT_WIDTH - 1:0] motor_4_rate,
					   input wire resetn,
					   input wire us_clk);

	// Internal counters

	reg [15:0] period_counter;
	reg [INPUT_BIT_WIDTH + 1:0] high_counter;
	reg high_counter_en;

	// Latched PWM values
	reg [INPUT_BIT_WIDTH + 1:0] m1_rate, m2_rate, m3_rate, m4_rate;

	// PWM gen blocks
	pwm_generator_block pwm1 (
		.state_out(state_out),
		.motor_pwm(motor_1_pwm),
		.motor_val(m1_rate),
		.period_counter(period_counter),
		.high_counter(high_counter),
		.resetn(resetn),
		.us_clk(us_clk));
	pwm_generator_block pwm2 (
		.motor_pwm(motor_2_pwm),
		.motor_val(m2_rate),
		.period_counter(period_counter),
		.high_counter(high_counter),
		.resetn(resetn),
		.us_clk(us_clk));
	pwm_generator_block pwm3 (
		.motor_pwm(motor_3_pwm),
		.motor_val(m3_rate),
		.period_counter(period_counter),
		.high_counter(high_counter),
		.resetn(resetn),
		.us_clk(us_clk));
	pwm_generator_block pwm4 (
		.motor_pwm(motor_4_pwm),
		.motor_val(m4_rate),
		.period_counter(period_counter),
		.high_counter(high_counter),
		.resetn(resetn),
		.us_clk(us_clk));



	// Control counters, latch new values
	always @(posedge us_clk or negedge resetn) begin
		if (!resetn) begin
			// Reset counters
			period_counter <= 16'h0000;
			high_counter <= 10'h000;
			high_counter_en <= 1'b0;
			// Start with default values
			m1_rate <= {8'h00};
			m2_rate <= {8'h00};
			m3_rate <= {8'h00};
			m4_rate <= {8'h00};
		end
		else if (period_counter == `PWM_PERIOD_US) begin
			// Reset counters
			period_counter <= 16'h0000;
			high_counter <= 10'h000;
			high_counter_en <= 1'b0;
			// Latch in new values
			m1_rate <= {motor_1_rate, 2'b00};
			m2_rate <= {motor_2_rate, 2'b00};
			m3_rate <= {motor_3_rate, 2'b00};
			m4_rate <= {motor_4_rate, 2'b00};
		end
		else if (period_counter == `MIN_PWM_TIME_HIGH_US) begin
			high_counter_en <= 1'b1;
			period_counter <= period_counter + 1'b1;
		end
		else begin
			period_counter <= period_counter + 1'b1;
			if (high_counter_en)
				high_counter <= high_counter + 1'b1;
		end
	end

endmodule
