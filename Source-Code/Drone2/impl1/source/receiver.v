`timescale 1ns / 1ns

/**
 * ECE 412-413 Capstone Winter/Spring 2018
 * Team 32 Drone2 SOC
 * Ethan Grinnell, Brett Creeley, Daniel Christiansen, Kirk Hooper, Zachary Clark-Williams
 */

/**
 * receiver - Receive pwm input from the hardware receiver and output values.  The pwm inputs have a
 *			  period of 20ms and a duty cycle rangeing between ~5% to ~10% which equals ~1ms to ~2ms.
 *
 * Outputs:
 * @throttle_val: Value between 0 and 2^(PWM_VALUE_BIT_WIDTH-1 that corresponds to between 1ms and 2ms pwm
 * @yaw_val: Value between 0 and 2^(PWM_VALUE_BIT_WIDTH-1 that corresponds to between 1ms and 2ms pwm
 * @roll_val: Value between 0 and 2^(PWM_VALUE_BIT_WIDTH-1 that corresponds to between 1ms and 2ms pwm
 * @pitch_val: Value between 0 and 2^(PWM_VALUE_BIT_WIDTH-1 that corresponds to between 1ms and 2ms pwm
 *
 * Inputs:
 * @throttle_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @yaw_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @roll_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @pitch_pwm: from the hardware receiver (duty cycle between ~5% and 10%)
 * @sys_clk: system clock
 */

`include "common_defines.v"

module receiver (output wire [`PWM_VALUE_BIT_WIDTH - 1:0] throttle_val,
				 output wire [`PWM_VALUE_BIT_WIDTH - 1:0] yaw_val,
				 output wire [`PWM_VALUE_BIT_WIDTH - 1:0] roll_val,
				 output wire [`PWM_VALUE_BIT_WIDTH - 1:0] pitch_val,
				 input wire throttle_pwm,
				 input wire yaw_pwm,
				 input wire roll_pwm,
				 input wire pitch_pwm,
				 input wire us_clk,
				 input wire resetn);


	wire [`PWM_TIME_BIT_WIDTH - 1:0] throttle_pwm_pulse_length_us,
									 yaw_pwm_pulse_length_us,
									 roll_pwm_pulse_length_us,
									 pitch_pwm_pulse_length_us;

	// THROTTLE input/output module instances
	pwm_reader #(`THROTTLE_DEFAULT_PULSE_TIME_HIGH_US) throttle_reader (
		.pwm(throttle_pwm),
		.pwm_pulse_length_us(throttle_pwm_pulse_length_us),
		.us_clk(us_clk),
		.resetn(resetn));

	pwm_to_value throttle_pwm_to_value (
		.pwm_time_high_us(throttle_pwm_pulse_length_us),
		.value_out(throttle_val),
		.us_clk(us_clk));

	// YAW input/output module instances
	pwm_reader #(`YAW_DEFAULT_PULSE_TIME_HIGH_US) yaw_reader (
		.pwm(yaw_pwm),
		.pwm_pulse_length_us(yaw_pwm_pulse_length_us),
		.us_clk(us_clk),
		.resetn(resetn));

	pwm_to_value yaw_pwm_to_value (
		.pwm_time_high_us(yaw_pwm_pulse_length_us),
		.value_out(yaw_val),
		.us_clk(us_clk));

	// ROLL input/output module instances
	pwm_reader #(`ROLL_DEFAULT_PULSE_TIME_HIGH_US) roll_reader (
		.pwm(roll_pwm),
		.pwm_pulse_length_us(roll_pwm_pulse_length_us),
		.us_clk(us_clk),
		.resetn(resetn));

	pwm_to_value roll_pwm_to_value (
		.pwm_time_high_us(roll_pwm_pulse_length_us),
		.value_out(roll_val),
		.us_clk(us_clk));

	// PITCH input/output module instances
	pwm_reader #(`PITCH_DEFAULT_PULSE_TIME_HIGH_US) pitch_reader (
		.pwm(pitch_pwm),
		.pwm_pulse_length_us(pitch_pwm_pulse_length_us),
		.us_clk(us_clk),
		.resetn(resetn));

	pwm_to_value pitch_pwm_to_value (
		.pwm_time_high_us(pitch_pwm_pulse_length_us),
		.value_out(pitch_val),
		.us_clk(us_clk));

endmodule

